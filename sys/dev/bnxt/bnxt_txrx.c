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

#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/iflib.h>

#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#include "bnxt.h"

/*
 * Function prototypes
 */

static int bnxt_isc_txd_encap(void *sc, if_pkt_info_t pi);
static void bnxt_isc_txd_flush(void *sc, uint16_t txqid, uint32_t pidx);
static int bnxt_isc_txd_credits_update(void *sc, uint16_t txqid, uint32_t cidx,
    bool clear);

static void bnxt_isc_rxd_refill(void *sc, uint16_t rxqid, uint8_t flid,
    uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs, uint16_t count);
static void bnxt_isc_rxd_flush(void *sc, uint16_t rxqid, uint8_t flid,
    uint32_t pidx);
static int bnxt_isc_rxd_available(void *sc, uint16_t rxqid, uint32_t idx);
static int bnxt_isc_rxd_pkt_get(void *sc, if_rxd_info_t ri);

extern void bnxt_if_enable_intr(if_ctx_t ctx);
static int bnxt_intr(void *sc);

struct if_txrx bnxt_txrx  = {
	bnxt_isc_txd_encap,
	bnxt_isc_txd_flush,
	bnxt_isc_txd_credits_update,
	bnxt_isc_rxd_available,
	bnxt_isc_rxd_pkt_get,
	bnxt_isc_rxd_refill,
	bnxt_isc_rxd_flush,
	bnxt_intr
};

/*
 * Device Dependent Packet Transmit and Receive Functions
 */

static int
bnxt_isc_txd_encap(void *sc, if_pkt_info_t pi)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return ENOSYS;
}

static void
bnxt_isc_txd_flush(void *sc, uint16_t txqid, uint32_t pidx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return;
}

static int
bnxt_isc_txd_credits_update(void *sc, uint16_t txqid, uint32_t cidx, bool clear)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return 0;	// No error return?
}

static void
bnxt_isc_rxd_refill(void *sc, uint16_t rxqid, uint8_t flid,
				uint32_t pidx, uint64_t *paddrs,
				caddr_t *vaddrs, uint16_t count)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return;
}

static void
bnxt_isc_rxd_flush(void *sc, uint16_t rxqid, uint8_t flid,
    uint32_t pidx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return;
}

static int
bnxt_isc_rxd_available(void *sc, uint16_t rxqid, uint32_t idx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return 0;	// No error return?
}

static int
bnxt_isc_rxd_pkt_get(void *sc, if_rxd_info_t ri)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return ENOSYS;
}

static int
bnxt_intr(void *sc)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return ENOSYS;
}
