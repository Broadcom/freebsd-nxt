/******************************************************************************

  Copyright (c) 2001-2015, Intel Corporation 
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
  
   2. Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution.
  
   3. Neither the name of the Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/
/*$FreeBSD$*/


#ifndef IXGBE_STANDALONE_BUILD
#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"
#endif

#include "ixgbe.h"

#ifdef	RSS
#include <net/rss_config.h>
#include <netinet/in_rss.h>
#endif

/*********************************************************************
 *  Local Function prototypes
 *********************************************************************/
static int ixgbe_isc_txd_encap(void *arg, if_pkt_info_t pi);
static void ixgbe_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx);
static int ixgbe_isc_txd_credits_update(void *arg, uint16_t txqid, uint16_t *cidx, bool clear);

static void ixgbe_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count, uint16_t buf_len);
static void ixgbe_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx);
static int ixgbe_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx);
static int ixgbe_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri);

static void ixgbe_rx_checksum(u32 staterr, if_rxd_info_t ri, u32 ptype);

extern void ixgbe_if_enable_intr(if_ctx_t ctx);
extern int ixgbe_intr(void *arg);
static int ixgbe_determine_rsstype(u16 pkt_info);

struct if_txrx ixgbe_txrx  = {
	ixgbe_isc_txd_encap,
	ixgbe_isc_txd_flush,
	ixgbe_isc_txd_credits_update,
	ixgbe_isc_rxd_available,
	ixgbe_isc_rxd_pkt_get,
	ixgbe_isc_rxd_refill,
	ixgbe_isc_rxd_flush,
	ixgbe_intr
};

extern if_shared_ctx_t ixgbe_sctx;

void
ixgbe_init_tx_ring(struct ix_tx_queue *que)
{
	struct tx_ring *txr = &que->txr;
	if_softc_ctx_t scctx = que->adapter->shared;
	struct ixgbe_tx_buf *buf;

	buf = txr->tx_buffers;
	for (int i = 0; i < scctx->isc_ntxd[0]; i++, buf++) {
		buf->eop = -1;
	}
}
/*********************************************************************
 *
 *  Advanced Context Descriptor setup for VLAN, CSUM or TSO
 *
 **********************************************************************/
static int
ixgbe_tx_ctx_setup(struct ixgbe_adv_tx_context_desc *TXD, if_pkt_info_t pi)
{
	u32 vlan_macip_lens, type_tucmd_mlhl;
	u32 olinfo_status, mss_l4len_idx, pktlen, offload;

	offload = TRUE;
	olinfo_status = mss_l4len_idx = vlan_macip_lens = type_tucmd_mlhl = 0;
	/* VLAN MACLEN IPLEN */
	vlan_macip_lens |= (htole16(pi->ipi_vtag) << IXGBE_ADVTXD_VLAN_SHIFT);
	vlan_macip_lens |= pi->ipi_ehdrlen << IXGBE_ADVTXD_MACLEN_SHIFT;

	pktlen = pi->ipi_len;
	/* First check if TSO is to be used */
	if (pi->ipi_csum_flags & CSUM_TSO) {
		/* This is used in the transmit desc in encap */
		pktlen = pi->ipi_len - pi->ipi_ehdrlen - pi->ipi_ip_hlen - pi->ipi_tcp_hlen;
		mss_l4len_idx |= (pi->ipi_tso_segsz << IXGBE_ADVTXD_MSS_SHIFT);
		mss_l4len_idx |= (pi->ipi_tcp_hlen << IXGBE_ADVTXD_L4LEN_SHIFT);
	}

	olinfo_status |= pktlen << IXGBE_ADVTXD_PAYLEN_SHIFT;

	if (pi->ipi_flags & IPI_TX_IPV4) {
		type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV4;
		/* Tell transmit desc to also do IPv4 checksum. */
		if (pi->ipi_csum_flags & (CSUM_IP|CSUM_TSO))
			olinfo_status |= IXGBE_TXD_POPTS_IXSM << 8;
	} else if (pi->ipi_flags & IPI_TX_IPV6)
		type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV6;
	else
		offload = FALSE;

	vlan_macip_lens |= pi->ipi_ip_hlen;

	switch (pi->ipi_ipproto) {
	case IPPROTO_TCP:
		if (pi->ipi_csum_flags & CSUM_TCP)
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_TCP;
		else
			offload = FALSE;
		break;
	case IPPROTO_UDP:
		if (pi->ipi_csum_flags & CSUM_UDP)
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_UDP;
		else
			offload = FALSE;
		break;
	case IPPROTO_SCTP:
		if (pi->ipi_csum_flags & CSUM_SCTP)
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_SCTP;
		else
			offload = FALSE;
		break;
	default:
		offload = FALSE;
		break;
	}
/* Insert L4 checksum into data descriptors */
	if (offload)
		olinfo_status |= IXGBE_TXD_POPTS_TXSM << 8;

	type_tucmd_mlhl |= IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT;

	/* Now copy bits into descriptor */
	TXD->vlan_macip_lens = htole32(vlan_macip_lens);
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);
	TXD->seqnum_seed = htole32(0);
	TXD->mss_l4len_idx = htole32(mss_l4len_idx);

        return (olinfo_status);
}

static int
ixgbe_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
	struct adapter *sc       = arg;
	if_softc_ctx_t scctx = sc->shared;
	struct ix_tx_queue *que     = &sc->tx_queues[pi->ipi_qsidx];
	struct tx_ring *txr      = &que->txr;
	struct ixgbe_tx_buf *buf;
	int         nsegs        = pi->ipi_nsegs;
	bus_dma_segment_t *segs  = pi->ipi_segs;
	union ixgbe_adv_tx_desc *txd = NULL;
	struct ixgbe_adv_tx_context_desc *TXD;
	int                     i, j, first, cidx_last;
	u32                     olinfo_status, cmd, flags;

	cmd =  (IXGBE_ADVTXD_DTYP_DATA |
		IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT);

	if (pi->ipi_mflags & M_VLANTAG)
		cmd |= IXGBE_ADVTXD_DCMD_VLE;
  
	i = first = pi->ipi_pidx;
	flags = (pi->ipi_flags & IPI_TX_INTR) ? IXGBE_TXD_CMD_RS : 0;

	TXD = (struct ixgbe_adv_tx_context_desc *) &txr->tx_base[first];
	if (pi->ipi_csum_flags & CSUM_OFFLOAD || IXGBE_IS_X550VF(sc) || pi->ipi_vtag) {
		/*********************************************
		 * Set up the appropriate offload context
		 * this will consume the first descriptor
		 *********************************************/
		olinfo_status = ixgbe_tx_ctx_setup(TXD, pi);
		if (pi->ipi_csum_flags & CSUM_TSO) {
			cmd |= IXGBE_ADVTXD_DCMD_TSE;
			++txr->tso_tx;
		}

		if (++i == scctx->isc_ntxd[0])
			i = 0;
	} else {
		/* Indicate the whole packet as payload when not doing TSO */
		olinfo_status = pi->ipi_len << IXGBE_ADVTXD_PAYLEN_SHIFT;
	}

	olinfo_status |= IXGBE_ADVTXD_CC;
	for (j = 0; j < nsegs; j++) {
		bus_size_t seglen;
		bus_addr_t segaddr;

		txd = &txr->tx_base[i];
		buf = &txr->tx_buffers[i];
		seglen = segs[j].ds_len;
		segaddr = htole64(segs[j].ds_addr);

		txd->read.buffer_addr = segaddr;
		txd->read.cmd_type_len = htole32(flags | cmd | seglen);
		txd->read.olinfo_status = htole32(olinfo_status);

		cidx_last = i;
		if (++i == scctx->isc_ntxd[0]) {
			i = 0;
		}
	}

	txd->read.cmd_type_len |=
		htole32(IXGBE_TXD_CMD_EOP | IXGBE_TXD_CMD_RS);

	/* Set the EOP descriptor that will be marked done */
	buf = &txr->tx_buffers[first];
	buf->eop = cidx_last;

	txr->bytes += pi->ipi_len;
	pi->ipi_new_pidx = i;

	++txr->total_packets;
  
	return (0);
}
  
static void
ixgbe_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx)
{
	struct adapter *sc       = arg;
	struct ix_tx_queue *que     = &sc->tx_queues[txqid];
	struct tx_ring *txr      = &que->txr;
  
	IXGBE_WRITE_REG(&sc->hw, txr->tail, pidx);
}

static int
ixgbe_isc_txd_credits_update(void *arg, uint16_t txqid, uint16_t *cidx_init, bool clear)
{
	struct adapter   *sc = arg;
	if_softc_ctx_t scctx = sc->shared;
	struct ix_tx_queue  *que = &sc->tx_queues[txqid];
	struct tx_ring   *txr = &que->txr;
	
	u32		    cidx, ntxd, processed = 0;
	u32               limit = sc->tx_process_limit;

	struct ixgbe_tx_buf	*buf;
	union ixgbe_adv_tx_desc *txd;

	cidx = *cidx_init;

	buf = &txr->tx_buffers[cidx];
	txd = &txr->tx_base[cidx];
	ntxd = scctx->isc_ntxd[0];
	do {
		int delta, eop = buf->eop;
		union ixgbe_adv_tx_desc *eopd;

		if (eop == -1) { /* No work */
			break;
		}

		eopd = &txr->tx_base[eop];
		if ((eopd->wb.status & IXGBE_TXD_STAT_DD) == 0) {
			break;	/* I/O not complete */
		} else if (clear)
			buf->eop = -1; /* clear indicate processed */

		/*
		 *
		 * update for multi segment case
		 */
		if (eop != cidx) {
			delta = eop - cidx;
			if (eop < cidx)
				delta += ntxd;
			processed += delta;
			txd = eopd;
			buf = &txr->tx_buffers[eop];
			cidx = eop;
		}
		processed++;
		if (clear)
			++txr->packets;

		/* Try the next packet */
		txd++;
		buf++;
		cidx++;
		/* reset with a wrap */
		if (__predict_false(cidx == scctx->isc_ntxd[0])) {
			cidx = 0;
			buf = txr->tx_buffers;
			txd = txr->tx_base;
		}
		prefetch(txd);
		prefetch(txd+1);
	} while (__predict_true(--limit) && cidx != *cidx_init);

	return (processed);
}

static void
ixgbe_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				 uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count, uint16_t buf_len)
{
	struct adapter *sc       = arg;
	struct ix_rx_queue *que     = &sc->rx_queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;
	int			i;
	uint32_t next_pidx;

	for (i = 0, next_pidx = pidx; i < count; i++) {
		rxr->rx_base[next_pidx].read.pkt_addr = htole64(paddrs[i]);
		if (++next_pidx == sc->shared->isc_nrxd[0])
			next_pidx = 0;
	}
}

static void
ixgbe_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx)
{
	struct adapter *sc       = arg;
	struct ix_rx_queue *que     = &sc->rx_queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;

	IXGBE_WRITE_REG(&sc->hw, rxr->tail, pidx);
}

static int
ixgbe_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx)
{
	struct adapter *sc       = arg;
	struct ix_rx_queue *que     = &sc->rx_queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;
	union ixgbe_adv_rx_desc *rxd;
	u32                      staterr;
	int                      cnt, i, nrxd;

	nrxd = sc->shared->isc_nrxd[0];
	for (cnt = 0, i = idx; cnt < nrxd-1;) {
		rxd = &rxr->rx_base[i];
		staterr = le32toh(rxd->wb.upper.status_error);

		if ((staterr & IXGBE_RXD_STAT_DD) == 0)
			break;
		if (++i == nrxd)
			i = 0;
		if (staterr & IXGBE_RXD_STAT_EOP)
			cnt++;
	}

	return (cnt);
}

/****************************************************************
 * Routine sends data which has been dma'ed into host memory
 * to upper layer. Initialize ri structure. 
 *
 * Returns 0 upon success, errno on failure
 ***************************************************************/

static int
ixgbe_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	struct adapter           *adapter = arg;
	struct ix_rx_queue          *que = &adapter->rx_queues[ri->iri_qsidx];
	struct rx_ring           *rxr = &que->rxr;
	struct ifnet             *ifp = iflib_get_ifp(adapter->ctx);
	union ixgbe_adv_rx_desc  *rxd;

	u16                      pkt_info, len, cidx, i;
	u16                      vtag = 0;
	u32                      ptype;
	u32                      staterr = 0;
	bool                     eop;

	i = 0;
	cidx = ri->iri_cidx;
	do {
		rxd = &rxr->rx_base[cidx];
		staterr = le32toh(rxd->wb.upper.status_error);
		pkt_info = le16toh(rxd->wb.lower.lo_dword.hs_rss.pkt_info);

		/* Error Checking then decrement count */
		MPASS ((staterr & IXGBE_RXD_STAT_DD) != 0);

		len = le16toh(rxd->wb.upper.length);
		ptype = le32toh(rxd->wb.lower.lo_dword.data) &
			IXGBE_RXDADV_PKTTYPE_MASK;

		ri->iri_len += len;
		rxr->bytes += len;

		rxd->wb.upper.status_error = 0;
		eop = ((staterr & IXGBE_RXD_STAT_EOP) != 0);
		if (staterr & IXGBE_RXD_STAT_VP) {
			vtag = le16toh(rxd->wb.upper.vlan);
		} else {
			vtag = 0;
		}
	
		/* Make sure bad packets are discarded */
		if (eop && (staterr & IXGBE_RXDADV_ERR_FRAME_ERR_MASK) != 0) {

#if __FreeBSD_version >= 1100036
			if (IXGBE_IS_VF(adapter))
				if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
#endif

			rxr->rx_discarded++;
			return (EBADMSG);
		}
		ri->iri_frags[i].irf_flid = 0;
		ri->iri_frags[i].irf_idx = cidx;
		ri->iri_frags[i].irf_len = len;
		if (++cidx == adapter->shared->isc_nrxd[0])
			cidx = 0;
		i++;
		/* even a 16K packet shouldn't consume more than 8 clusters */
		MPASS(i < 9);
	} while (!eop);

	rxr->rx_packets++;
	rxr->packets++;

	if ((ifp->if_capenable & IFCAP_RXCSUM) != 0)
		ixgbe_rx_checksum(staterr, ri,  ptype);

	ri->iri_flowid = le32toh(rxd->wb.lower.hi_dword.rss);
	ri->iri_rsstype = ixgbe_determine_rsstype(pkt_info);
	ri->iri_vtag = vtag;
	ri->iri_nfrags = i;
	if (vtag)
		ri->iri_flags |= M_VLANTAG;
	return (0);
}

/*********************************************************************
 *
 *  Verify that the hardware indicated that the checksum is valid.
 *  Inform the stack about the status of checksum so that stack
 *  doesn't spend time verifying the checksum.
 *
 *********************************************************************/
static void
ixgbe_rx_checksum(u32 staterr, if_rxd_info_t ri, u32 ptype)
{
	u16	status = (u16) staterr;
	u8	errors = (u8) (staterr >> 24);
	bool	sctp = FALSE;

	if ((ptype & IXGBE_RXDADV_PKTTYPE_ETQF) == 0 &&
	    (ptype & IXGBE_RXDADV_PKTTYPE_SCTP) != 0)
		sctp = TRUE;

	if (status & IXGBE_RXD_STAT_IPCS) {
		if (!(errors & IXGBE_RXD_ERR_IPE)) {
			/* IP Checksum Good */
			ri->iri_csum_flags = CSUM_IP_CHECKED | CSUM_IP_VALID;
		} else
			ri->iri_csum_flags = 0;
	}
	if (status & IXGBE_RXD_STAT_L4CS) {
		u64 type = (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
#if __FreeBSD_version >= 800000
		if (sctp)
			type = CSUM_SCTP_VALID;
#endif
		if (!(errors & IXGBE_RXD_ERR_TCPE)) {
			ri->iri_csum_flags |= type;
			if (!sctp)
				ri->iri_csum_data = htons(0xffff);
		} 
	}
}

/********************************************************************
 *
 *  Parse the packet type to determine the appropriate hash
 *
 ******************************************************************/
static int 
ixgbe_determine_rsstype(u16 pkt_info)
{
	switch (pkt_info & IXGBE_RXDADV_RSSTYPE_MASK) {
	case IXGBE_RXDADV_RSSTYPE_IPV4_TCP:
		return M_HASHTYPE_RSS_TCP_IPV4;
	case IXGBE_RXDADV_RSSTYPE_IPV4:
		return M_HASHTYPE_RSS_IPV4;
	case IXGBE_RXDADV_RSSTYPE_IPV6_TCP:
		return M_HASHTYPE_RSS_TCP_IPV6;
	case IXGBE_RXDADV_RSSTYPE_IPV6_EX:
		return M_HASHTYPE_RSS_IPV6_EX;
	case IXGBE_RXDADV_RSSTYPE_IPV6:
		return M_HASHTYPE_RSS_IPV6;
	case IXGBE_RXDADV_RSSTYPE_IPV6_TCP_EX:
		return M_HASHTYPE_RSS_TCP_IPV6_EX;
	case IXGBE_RXDADV_RSSTYPE_IPV4_UDP:
		return M_HASHTYPE_RSS_UDP_IPV4;
	case IXGBE_RXDADV_RSSTYPE_IPV6_UDP:
		return M_HASHTYPE_RSS_UDP_IPV6;
	case IXGBE_RXDADV_RSSTYPE_IPV6_UDP_EX:
		return M_HASHTYPE_RSS_UDP_IPV6_EX;
	default:
		return M_HASHTYPE_OPAQUE;
     }
}
