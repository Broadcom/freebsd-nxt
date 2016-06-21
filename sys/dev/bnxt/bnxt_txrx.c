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
	tbd->opaque = htole32(pi->ipi_new_pidx);
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
			lflags |= TX_BD_LONG_LFLAGS_TCP_UDP_CHKSUM | TX_BD_LONG_LFLAGS_IP_CHKSUM;
		}
		else if(pi->ipi_csum_flags & CSUM_IP) {
			lflags |= TX_BD_LONG_LFLAGS_IP_CHKSUM;
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

	/* TODO: If we could actually get the pidx here, we could use that */
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
	struct bnxt_tx_ring *txr = &softc->tx_rings[txqid];
	struct tx_cmpl *tcp;
	struct tx_bd_long *tbd;
	int avail = 0;
	uint32_t raw = cpr->raw_cons;
	uint32_t cons;

	for (;;) {
		raw++;
		cons = RING_CMP(&cpr->ring, raw);
		tcp = &((struct tx_cmpl *)cpr->ring.vaddr)[cons];

		if (!CMP_VALID(tcp, raw, &cpr->ring)) {
			raw--;
			break;
		}

		/* Get the BD that this completes */
		tbd = &((struct tx_bd_long *)txr->ring.vaddr)[le32toh(tcp->opaque)];

		/* And extract how many BDs were used */
		avail += (tbd->flags_type & TX_BD_SHORT_FLAGS_BD_CNT_MASK) >> TX_BD_SHORT_FLAGS_BD_CNT_SFT;
	}

	if (clear) {
		cpr->raw_cons = raw;
		BNXT_CP_DB(&cpr->ring, cpr->raw_cons);
	}

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

device_printf(softc->dev, "Refilling q=%hu fl=%hhu p=%u c=%hu\n", rxqid, flid, pidx, count);
	if (flid == 0) {
		rx_ring = &softc->rx_rings[rxqid];
		rxbd = (void *)rx_ring->ring.vaddr;
	}
	else {
		rx_ring = &softc->ag_rings[rxqid];
		rxbd = (void *)rx_ring->ring.vaddr;
	}
	for (i=0; i<count; i++) {
		rxbd[rx_ring->prod].opaque = htole32(pidx + i);
		rxbd[rx_ring->prod].addr = htole64(paddrs[i]);
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

device_printf(softc->dev, "Flushing q=%hu fl=%hhu p=%u (%u)\n", rxqid, flid, pidx, rx_ring->prod);
	BNXT_RX_DB(rx_ring->ring.doorbell, rx_ring->prod);
	return;
}

static int
bnxt_isc_rxd_available(void *sc, uint16_t rxqid, uint32_t idx)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_cp_ring *cpr = &softc->rx_cp_rings[rxqid];
	struct rx_pkt_cmpl *rcp;
	struct cmpl_base *cmp;
	int avail = 0;
	uint32_t raw = cpr->raw_cons;
	uint32_t last_valid;
	uint32_t cons;
	uint8_t ags;
	int i;

	for (;;) {
		last_valid = raw;
		raw++;
		cons = RING_CMP(&cpr->ring, raw);
		rcp = &((struct rx_pkt_cmpl *)cpr->ring.vaddr)[cons];

device_printf(softc->dev, "Checking RAW: %u (%u:%u)\n", raw, cons, rcp->agg_bufs_v1 & RX_PKT_CMPL_V1);
		if (!CMP_VALID(rcp, raw, &cpr->ring))
			break;
device_printf(softc->dev, "RAW %u valid (%02x)\n", raw, rcp->flags_type & RX_PKT_CMPL_TYPE_MASK);

		if ((rcp->flags_type & RX_PKT_CMPL_TYPE_MASK) == RX_PKT_CMPL_TYPE_RX_L2) {
			ags = (rcp->agg_bufs_v1 & RX_PKT_CMPL_AGG_BUFS_MASK) >> RX_PKT_CMPL_AGG_BUFS_SFT;
			raw++;
			cons = RING_CMP(&cpr->ring, raw);
			cmp = &((struct cmpl_base *)cpr->ring.vaddr)[cons];
device_printf(softc->dev, "Checking RAW2: %u (%u:%u)\n", raw, cons, cmp->info3_v & CMPL_BASE_V);
			if (!CMP_VALID(cmp, raw, &cpr->ring))
				break;
device_printf(softc->dev, "RAW2 %u valid\n", raw);

			/* Now account for all the AG completions */
			for (i=0; i<ags; i++) {
				raw++;
				cons = RING_CMP(&cpr->ring, raw);
				cmp = &((struct cmpl_base *)cpr->ring.vaddr)[cons];
device_printf(softc->dev, "Checking RAWAGG%d: %u\n", i, raw);
				if (!CMP_VALID(cmp, raw, &cpr->ring))
					break;
device_printf(softc->dev, "Checking RAWAGG%d valid (%02x)\n", i, cmp->type & CMPL_BASE_TYPE_MASK);
			}
			avail++;
		}
	}
	cpr->enable_at = last_valid;

	/* Surrious interrupt? */
	if (!avail) {
		device_printf(softc->dev, "rxd_available() called with nothing available\n");
		BNXT_CP_ARM_DB(&cpr->ring, cpr->raw_cons);
	}
device_printf(softc->dev, "RXd Avail: %d\n", avail);
	return avail;
}

/* If we return anything but zero, iflib will assert... */
static int
bnxt_isc_rxd_pkt_get(void *sc, if_rxd_info_t ri)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;
	struct bnxt_cp_ring *cpr = &softc->rx_cp_rings[ri->iri_qsidx];
	struct rx_pkt_cmpl *rcp;
	struct rx_pkt_cmpl_hi *rcph;
	struct rx_abuf_cmpl *acp;
	uint16_t flags_type;
	uint32_t flags2;
	uint32_t errors;
	uint32_t raw = cpr->raw_cons;
	uint32_t cons;
	uint8_t	ags;
	int i;

	raw++;
	cons = RING_CMP(&cpr->ring, raw);
	rcp = &((struct rx_pkt_cmpl *)cpr->ring.vaddr)[cons];
	if (!CMP_VALID(rcp, raw, &cpr->ring)) {
		device_printf(softc->dev, "No RX completion\n");
		return EINVAL;
	}
	if (!CMP_VALID(rcp, raw+1, &cpr->ring)) {
		device_printf(softc->dev, "No second RX completion\n");
		return EINVAL;
	}
	flags_type = le16toh(rcp->flags_type);
	if ((flags_type & RX_PKT_CMPL_TYPE_MASK) != RX_PKT_CMPL_TYPE_RX_L2) {
		device_printf(softc->dev, "Invalid RX completion type %u\n", flags_type & RX_PKT_CMPL_TYPE_MASK);
		return EINVAL;
	}
	if (flags_type & RX_PKT_CMPL_FLAGS_ERROR) {
		raw++;
		cons = RING_CMP(&cpr->ring, raw);
		rcph = &((struct rx_pkt_cmpl_hi *)cpr->ring.vaddr)[cons];
		device_printf(softc->dev, "RX completion error %04x\n", le16toh(rcph->errors_v2) & RX_PKT_CMPL_ERRORS_BUFFER_ERROR_MASK);
		return EINVAL;
	}

	/* Extract from the first 16-byte BD */
	if (flags_type & RX_PKT_CMPL_FLAGS_RSS_VALID) {
		ri->iri_flowid = le32toh(rcp->rss_hash);
		/* TODO: Extract something useful from rcp->rss_hash_type (undocumented) */
		// May be documented in the "LSI ES" -- also check the firmware code.
		device_printf(softc->dev, "TODO: Mystery RSS hash type: %02hhx\n", rcp->rss_hash_type);
		ri->iri_rsstype = M_HASHTYPE_OPAQUE;
	}
	else {
		ri->iri_rsstype = M_HASHTYPE_NONE;
	}
	ags = (rcp->agg_bufs_v1 & RX_PKT_CMPL_AGG_BUFS_MASK) >> RX_PKT_CMPL_AGG_BUFS_SFT;
	ri->iri_nfrags = ags + 1;
	ri->iri_frags[0].irf_flid = 0;
	ri->iri_frags[0].irf_idx = le32toh(rcp->opaque);
	ri->iri_len = le16toh(rcp->len);

	/* Now the second 16-byte BD */
	raw++;
	cons = RING_CMP(&cpr->ring, raw);
	rcph = &((struct rx_pkt_cmpl_hi *)cpr->ring.vaddr)[cons];
	flags2 = le32toh(rcph->flags2);
	errors = le16toh(rcph->errors_v2);
	if ((flags2 & RX_PKT_CMPL_FLAGS2_META_FORMAT_MASK) == RX_PKT_CMPL_FLAGS2_META_FORMAT_VLAN) {
		ri->iri_flags |= M_VLANTAG;
		/* TODO: Should this be the entire 16-bits? */
		ri->iri_vtag = le32toh(rcph->metadata) & (RX_PKT_CMPL_METADATA_VID_MASK | RX_PKT_CMPL_METADATA_DE | RX_PKT_CMPL_METADATA_PRI_MASK);
	}
	if (flags2 & RX_PKT_CMPL_FLAGS2_IP_CS_CALC) {
		ri->iri_csum_flags |= CSUM_IP_CHECKED;
		if (!(errors & RX_PKT_CMPL_ERRORS_IP_CS_ERROR))
			ri->iri_csum_flags |= CSUM_IP_VALID;
	}
	if (flags2 & RX_PKT_CMPL_FLAGS2_L4_CS_CALC) {
		ri->iri_csum_flags |= CSUM_L4_CALC;
		if (!(errors & RX_PKT_CMPL_ERRORS_L4_CS_ERROR))
			ri->iri_csum_flags |= CSUM_L4_VALID;
	}

	/* And finally the ag ring stuff. */
device_printf(softc->dev, "Handing %d AG bufs\n", ags);
	for (i=1; i < ri->iri_nfrags; i++) {
		raw++;
		cons = RING_CMP(&cpr->ring, raw);
		acp = &((struct rx_abuf_cmpl *)cpr->ring.vaddr)[cons];

		ri->iri_frags[i].irf_flid = 1;
		ri->iri_frags[i].irf_idx = le32toh(acp->opaque);
		ri->iri_len += le16toh(acp->len);
	}

	/* Notify the hardware we've handled the completion */
	if (cpr->enable_at == raw) {
device_printf(softc->dev, "Ringing doorbell at %u\n", raw-1);
		BNXT_CP_ARM_DB(&cpr->ring, raw);
	}
	else {
		BNXT_CP_DB(&cpr->ring, raw);
	}
	cpr->raw_cons = raw;

device_printf(softc->dev, "RXed a packet.\n");
	return 0;
}

static int
bnxt_intr(void *sc)
{
	struct bnxt_softc *softc = (struct bnxt_softc *)sc;

	device_printf(softc->dev, "STUB: %s @ %s:%d\n", __func__, __FILE__, __LINE__);
	return ENOSYS;
}
