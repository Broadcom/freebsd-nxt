/*-
 * Broadcom NetXtreme-C/E network driver.
 *
 * Copyright (c) 2016 Broadcom, All Rights Reserved.
 * The term Broadcom refers to Broadcom Limited and/or its subsidiaries
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/socket.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/iflib.h>

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#include "ifdi_if.h"

#include "bnxt.h"
#include "hsi_struct_def.h"

/*
 * PCI Device ID Table
 */

static pci_vendor_info_t bnxt_vendor_info_array[] =
{
    PVID(BROADCOM_VENDOR_ID, BCM57301,
	"Broadcom BCM57301 NetXtreme-C Single-port 10Gb Ethernet"),
    PVID(BROADCOM_VENDOR_ID, BCM57302,
	"Broadcom BCM57302 NetXtreme-C Dual-port 10Gb/25Gb Ethernet"),
    PVID(BROADCOM_VENDOR_ID, BCM57304,
	"Broadcom BCM57304 NetXtreme-C Dual-port 10Gb/25Gb/40Gb/50Gb Ethernet"),
    PVID(BROADCOM_VENDOR_ID, BCM57402,
	"Broadcom BCM57402 NetXtreme-E Dual-port 10Gb Ethernet"),
    PVID(BROADCOM_VENDOR_ID, BCM57404,
	"Broadcom BCM57404 NetXtreme-E Dual-port 10Gb/25Gb Ethernet"),
    PVID(BROADCOM_VENDOR_ID, BCM57406,
	"Broadcom BCM57406 NetXtreme-E Dual-port 10GBase-T Ethernet"),

    /* required last entry */
    PVID_END
};

/*
 * Function prototypes
 */

static void *bnxt_register(device_t dev);

/* Soft queue setup and teardown */
static int bnxt_if_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int nrxqs, int nrxqsets);
static int bnxt_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int nrxqs, int nrxqsets);

/* Device setup and teardown */
static int bnxt_if_attach_pre(if_ctx_t ctx);
static int bnxt_if_attach_post(if_ctx_t ctx);
static int bnxt_if_detach(if_ctx_t ctx);

/* Device configuration */
static void bnxt_if_init(if_ctx_t ctx);
static void bnxt_if_stop(if_ctx_t ctx);
static void bnxt_if_multi_set(if_ctx_t ctx);
static int bnxt_if_mtu_set(if_ctx_t ctx, uint32_t mtu);
static void bnxt_if_media_status(if_ctx_t ctx, struct ifmediareq * ifmr);
static int bnxt_if_media_change(if_ctx_t ctx);
static int bnxt_if_promisc_set(if_ctx_t ctx, int flags);
static uint64_t	bnxt_if_get_counter(if_ctx_t, ift_counter);
static void bnxt_if_update_admin_status(if_ctx_t ctx);

/* Interrupt enable / disable */
void bnxt_if_enable_intr(if_ctx_t ctx);
static void bnxt_if_queue_intr_enable(if_ctx_t ctx, uint16_t qid);
static void bnxt_if_disable_intr(if_ctx_t ctx);

/*
 * Device Interface Declaration
 */

static device_method_t bnxt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_register, bnxt_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(device_suspend, iflib_device_suspend),
	DEVMETHOD(device_resume, iflib_device_resume),
	DEVMETHOD_END
};

static driver_t bnxt_driver = {
	"bnxt", bnxt_methods, sizeof(struct bnxt_softc),
};

devclass_t bnxt_devclass;
DRIVER_MODULE(bnxt, pci, bnxt_driver, bnxt_devclass, 0, 0);

MODULE_DEPEND(bnxt, pci, 1, 1, 1);
MODULE_DEPEND(bnxt, ether, 1, 1, 1);
MODULE_DEPEND(bnxt, iflib, 1, 1, 1);

static device_method_t bnxt_if_methods[] = {
	DEVMETHOD(ifdi_tx_queues_alloc, bnxt_if_tx_queues_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, bnxt_if_rx_queues_alloc),

	DEVMETHOD(ifdi_attach_pre, bnxt_if_attach_pre),
	DEVMETHOD(ifdi_attach_post, bnxt_if_attach_post),
	DEVMETHOD(ifdi_detach, bnxt_if_detach),

	DEVMETHOD(ifdi_init, bnxt_if_init),
	DEVMETHOD(ifdi_stop, bnxt_if_stop),
	DEVMETHOD(ifdi_multi_set, bnxt_if_multi_set),
	DEVMETHOD(ifdi_mtu_set, bnxt_if_mtu_set),
	DEVMETHOD(ifdi_media_status, bnxt_if_media_status),
	DEVMETHOD(ifdi_media_change, bnxt_if_media_change),
	DEVMETHOD(ifdi_promisc_set, bnxt_if_promisc_set),
	DEVMETHOD(ifdi_get_counter, bnxt_if_get_counter),
	DEVMETHOD(ifdi_update_admin_status, bnxt_if_update_admin_status),

	DEVMETHOD(ifdi_intr_enable, bnxt_if_enable_intr),
	DEVMETHOD(ifdi_queue_intr_enable, bnxt_if_queue_intr_enable),
	DEVMETHOD(ifdi_intr_disable, bnxt_if_disable_intr),

	DEVMETHOD_END
};

static driver_t bnxt_if_driver = {
	"bnxt", bnxt_if_methods, sizeof(struct bnxt_softc)
};

/*
 * iflib shared context
 */

extern struct if_txrx bnxt_txrx;
static struct if_shared_ctx bnxt_sctx_init = {
	.isc_magic = IFLIB_MAGIC,
	.isc_txrx = &bnxt_txrx,
	.isc_driver = &bnxt_if_driver,
	.isc_nfl = 1,
	.isc_q_align = PAGE_SIZE,
	.isc_tx_maxsize = BNXT_TSO_SIZE,
	.isc_tx_maxsegsize = PAGE_SIZE*4,
	.isc_rx_maxsize = PAGE_SIZE*4,
	.isc_rx_maxsegsize = PAGE_SIZE*4,
	.isc_rx_nsegments = 1,
	.isc_ntxqs = 1,
	.isc_nrxqs = 1,
	.isc_admin_intrcnt = 1,
	.isc_vendor_info = bnxt_vendor_info_array,
};

if_shared_ctx_t bnxt_sctx = &bnxt_sctx_init;

/*
 * Device Methods
 */

static void *bnxt_register(device_t dev)
{
	return NULL;
}

/*
 * Device Dependent Configuration Functions
*/

/* Soft queue setup and teardown */
static int bnxt_if_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int nrxqs, int nrxqsets)
{
	return ENOSYS;
}

static int bnxt_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs,
    uint64_t *paddrs, int nrxqs, int nrxqsets)
{
	return ENOSYS;
}

/* Device setup and teardown */
static int bnxt_if_attach_pre(if_ctx_t ctx)
{
	return ENOSYS;
}

static int bnxt_if_attach_post(if_ctx_t ctx)
{
	return ENOSYS;
}

static int bnxt_if_detach(if_ctx_t ctx)
{
	return ENOSYS;
}

/* Device configuration */
static void bnxt_if_init(if_ctx_t ctx)
{
	return;
}

static void bnxt_if_stop(if_ctx_t ctx)
{
	return;
}

static void bnxt_if_multi_set(if_ctx_t ctx)
{
	return;
}

static int bnxt_if_mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	return ENOSYS;
}

static void bnxt_if_media_status(if_ctx_t ctx, struct ifmediareq * ifmr)
{
	return;
}

static int bnxt_if_media_change(if_ctx_t ctx)
{
	return ENOSYS;
}

static int bnxt_if_promisc_set(if_ctx_t ctx, int flags)
{
	return ENOSYS;
}

static uint64_t	bnxt_if_get_counter(if_ctx_t ctx, ift_counter cnt)
{
	if_t ifp = iflib_get_ifp(ctx);

	return if_get_counter_default(ifp, cnt);
}

static void bnxt_if_update_admin_status(if_ctx_t ctx)
{
	return;
}

/* Interrupt enable / disable */
void bnxt_if_enable_intr(if_ctx_t ctx)
{
	return;
}

static void bnxt_if_queue_intr_enable(if_ctx_t ctx, uint16_t qid)
{
	return;
}

static void bnxt_if_disable_intr(if_ctx_t ctx)
{
	return;
}

