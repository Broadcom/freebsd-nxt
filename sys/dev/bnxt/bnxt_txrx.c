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
#include <sys/endian.h>
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

static const uint16_t bnxt_tx_lhint[] = {
	TX_BD_SHORT_FLAGS_LHINT_LT512,
	TX_BD_SHORT_FLAGS_LHINT_LT1K,
	TX_BD_SHORT_FLAGS_LHINT_LT2K,
	TX_BD_SHORT_FLAGS_LHINT_LT2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
	TX_BD_SHORT_FLAGS_LHINT_GTE2K,
};

static int
bnxt_isc_txd_encap(void *sc, if_pkt_info_t pi)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_tx_ring *txr = &softc->tx_rings[pi->ipi_qsidx];
	struct tx_bd_long *tbd;
	struct tx_bd_long_hi *tbdh;
	bool need_hi = false;
	uint16_t flags_type;
	uint16_t lflags;
	uint32_t cfa_meta;
	int seg = 0;

	/* If we have offloads enabled, we need to use two BDs. */
	if ((pi->ipi_csum_flags & (CSUM_OFFLOAD | CSUM_TSO | CSUM_IP)) ||
	    pi->ipi_mflags & M_VLANTAG)
		need_hi = true;

	pi->ipi_new_pidx = pi->ipi_pidx;
	tbd = &((struct tx_bd_long *)txr->ring.vaddr)[pi->ipi_new_pidx];
	pi->ipi_ndescs = 1;
	tbd->opaque = pi->ipi_new_pidx;
	tbd->len = htole16(pi->ipi_segs[seg].ds_len);
	tbd->addr = htole64(pi->ipi_segs[seg++].ds_addr);
	flags_type = (pi->ipi_nsegs + need_hi) << TX_BD_SHORT_FLAGS_BD_CNT_SFT;
	flags_type |= bnxt_tx_lhint[pi->ipi_len >> 9];

	if (need_hi) {
		flags_type |= TX_BD_LONG_TYPE_TX_BD_LONG;
		tbd->flags_type = htole16(flags_type);

		pi->ipi_new_pidx = RING_NEXT(&txr->ring, pi->ipi_new_pidx);
		pi->ipi_ndescs++;
		tbdh = &((struct tx_bd_long_hi *)txr->ring.vaddr)[pi->ipi_new_pidx];
		tbdh->mss = htole16(pi->ipi_tso_segsz);
		tbdh->hdr_size = htole16(pi->ipi_ehdrlen + pi->ipi_ip_hlen + pi->ipi_tcp_hlen);
		tbdh->cfa_action = 0;
		lflags = 0;
		cfa_meta = 0;
		if (pi->ipi_mflags & M_VLANTAG) {
			/* TODO: Do we need to byte-swap the vtag here? */
			cfa_meta = TX_BD_LONG_CFA_META_KEY_VLAN_TAG | pi->ipi_vtag;
			cfa_meta |= TX_BD_LONG_CFA_META_VLAN_TPID_TPID8100;
		}
		tbdh->cfa_meta = htole32(cfa_meta);
		if (pi->ipi_csum_flags & CSUM_TSO) {
			lflags |= TX_BD_LONG_LFLAGS_LSO | TX_BD_LONG_LFLAGS_T_IPID;
		}
		else if(pi->ipi_csum_flags & CSUM_OFFLOAD) {
			lflags |= TX_BD_LONG_LFLAGS_TCP_UDP_CHKSUM;
		}
		else if(pi->ipi_csum_flags & CSUM_IP) {
			lflags |= TX_BD_LONG_LFLAGS_T_IP_CHKSUM;
		}
		tbdh->lflags = htole16(lflags);
	}
	else {
		flags_type |= TX_BD_SHORT_TYPE_TX_BD_SHORT;
	}

	for (; seg < pi->ipi_nsegs; seg++) {
		tbd->flags_type = htole16(flags_type);
		pi->ipi_new_pidx = RING_NEXT(&txr->ring, pi->ipi_new_pidx);
		pi->ipi_ndescs++;
		tbd = &((struct tx_bd_long *)txr->ring.vaddr)[pi->ipi_new_pidx];
		tbd->len = htole16(pi->ipi_segs[seg].ds_len);
		tbd->addr = htole64(pi->ipi_segs[seg].ds_addr);
		flags_type = TX_BD_SHORT_TYPE_TX_BD_SHORT;
	}
	flags_type |= TX_BD_SHORT_FLAGS_PACKET_END;
	tbd->flags_type = htole16(flags_type);
	pi->ipi_new_pidx = RING_NEXT(&txr->ring, pi->ipi_new_pidx);

	return 0;
}

static void
bnxt_isc_txd_flush(void *sc, uint16_t txqid, uint32_t pidx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_tx_ring *tx_ring = &softc->tx_rings[txqid];

	tx_ring->prod += pidx;
	tx_ring->prod &= tx_ring->ring.ring_mask;
	BNXT_TX_DB(tx_ring->ring.doorbell, tx_ring->prod);
	return;
}

static int
bnxt_isc_txd_credits_update(void *sc, uint16_t txqid, uint32_t idx, bool clear)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_cp_ring *cpr = &softc->tx_cp_rings[txqid];
	struct tx_cmpl *tcp;
	int avail;
	uint32_t raw = cpr->raw_cons;
	uint32_t cons;

	/* This should never do anything, so is a candidate for removal */
	for (raw = cpr->raw_cons; RING_CMP(&cpr->ring, raw) != idx; raw++)
		device_printf(softc->dev, "TXD chasing IDX raw=%u idx=%u\n", raw, idx);

	for (avail = 0 ; ; avail++) {
		cons = RING_CMP(&cpr->ring, raw);
		tcp = &((struct tx_cmpl *)cpr->ring.vaddr)[cons];

		if (!CMP_VALID(tcp, raw, &cpr->ring))
			break;

		if (clear)
			cpr->raw_cons = raw;
		raw++;
	}

	if (clear)
		BNXT_CP_DB(cpr, cpr->raw_cons);

	return avail;
}

static void
bnxt_isc_rxd_refill(void *sc, uint16_t rxqid, uint8_t flid,
				uint32_t pidx, uint64_t *paddrs,
				caddr_t *vaddrs, uint16_t count)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_rx_ring *rx_ring;
	struct rx_prod_pkt_bd *rxbd;
	uint16_t i;

	if (flid == 0) {
		rx_ring = &softc->rx_rings[rxqid];
		rxbd = (void *)rx_ring->ring.vaddr;
	}
	else {
		rx_ring = &softc->ag_rings[rxqid];
		rxbd = (void *)rx_ring->ring.vaddr;
	}
	for (i=0; i<count; i++) {
		rxbd[rx_ring->prod].addr = htole64(paddrs[pidx + i]);
		rx_ring->prod = RING_NEXT(&rx_ring->ring, rx_ring->prod);
	}
	return;
}

static void
bnxt_isc_rxd_flush(void *sc, uint16_t rxqid, uint8_t flid,
    uint32_t pidx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	struct bnxt_rx_ring *rx_ring;

	if (flid == 0)
		rx_ring = &softc->rx_rings[rxqid];
	else
		rx_ring = &softc->ag_rings[rxqid];

	BNXT_RX_DB(rx_ring->ring.doorbell, rx_ring->prod);
	return;
}

static int
bnxt_isc_rxd_available(void *sc, uint16_t rxqid, uint32_t idx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_cp_ring *cpr = &softc->rx_cp_rings[rxqid];
	struct rx_pkt_cmpl *rcp;
	struct rx_pkt_cmpl_hi *rcph;
	int avail;
	uint32_t raw = cpr->raw_cons;
	uint32_t cons;

	for (raw = cpr->raw_cons; RING_CMP(&cpr->ring, raw) != idx; raw++)
		;

	for (avail = 0 ; ; avail++) {
		cons = RING_CMP(&cpr->ring, raw);
		rcp = &((struct rx_pkt_cmpl *)cpr->ring.vaddr)[cons];

		if (!CMP_VALID(rcp, raw, &cpr->ring))
			break;

		raw++;
		cons = RING_CMP(&cpr->ring, raw);
		rcph = &((struct rx_pkt_cmpl_hi *)cpr->ring.vaddr)[cons];
		if (!CMP_VALID(rcp, raw, &cpr->ring))
			break;
	}
	return avail;
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
