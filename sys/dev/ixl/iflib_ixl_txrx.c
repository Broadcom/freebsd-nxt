/******************************************************************************

  Copyright (c) 2013-2015, Intel Corporation 
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

/*
**	IXL driver TX/RX Routines:
**	    This was seperated to allow usage by
** 	    both the BASE and the VF drivers.
*/

#ifndef IXL_STANDALONE_BUILD
#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"
#endif

#include "ixl.h"

#ifdef RSS
#include <net/rss_config.h>
#endif

/* Local Prototypes */
static void ixl_rx_checksum(if_rxd_info_t ri, u32 status, u32 error, u8 ptype);

static int ixl_isc_txd_encap(void *arg, if_pkt_info_t pi);
static void ixl_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx);
static int ixl_isc_txd_credits_update(void *arg, uint16_t qid, uint32_t cidx, bool clear);

static void ixl_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count, uint16_t buf_len);
static void ixl_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx);
static int ixl_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx,
				 int budget);
static int ixl_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri);

extern int ixl_intr(void *arg);

struct if_txrx ixl_txrx  = {
	ixl_isc_txd_encap,
	ixl_isc_txd_flush,
	ixl_isc_txd_credits_update,
	ixl_isc_rxd_available,
	ixl_isc_rxd_pkt_get,
	ixl_isc_rxd_refill,
	ixl_isc_rxd_flush,
	ixl_intr
};


/*
** Find mbuf chains passed to the driver 
** that are 'sparse', using more than 8
** segments to deliver an mss-size chunk of data
*/
static int
ixl_tso_detect_sparse(bus_dma_segment_t *segs, int nsegs, int segsz)
{
	int		i, count, curseg;

	if (nsegs <= IXL_MAX_TX_SEGS-2)
		return (0);
	for (curseg = count = i = 0; i < nsegs; i++) {
		curseg += segs[i].ds_len;
		count++;
		if (__predict_false(count == IXL_MAX_TX_SEGS-2))
			return (1);
		if (curseg > segsz) {
			curseg -= segsz;
			count = 1;
		}
		if (curseg == segsz)
			curseg = count = 0;
	}
	return (0);
}

/*********************************************************************
 *
 *  Setup descriptor for hw offloads 
 *
 **********************************************************************/

static void
ixl_tx_setup_offload(struct ixl_tx_queue *que,
    if_pkt_info_t pi, u32 *cmd, u32 *off)
{


	switch (pi->ipi_etype) {
#ifdef INET
		case ETHERTYPE_IP:
			*cmd |= I40E_TX_DESC_CMD_IIPT_IPV4_CSUM;
			break;
#endif
#ifdef INET6
		case ETHERTYPE_IPV6:
			*cmd |= I40E_TX_DESC_CMD_IIPT_IPV6;
			break;
#endif
		default:
			break;
	}

	*off |= (pi->ipi_ehdrlen >> 1) << I40E_TX_DESC_LENGTH_MACLEN_SHIFT;
	*off |= (pi->ipi_ip_hlen >> 2) << I40E_TX_DESC_LENGTH_IPLEN_SHIFT;

	switch (pi->ipi_ipproto) {
		case IPPROTO_TCP:
			if (pi->ipi_csum_flags & (CSUM_TCP|CSUM_TCP_IPV6)) {
				*cmd |= I40E_TX_DESC_CMD_L4T_EOFT_TCP;
				*off |= (pi->ipi_tcp_hlen >> 2) <<
				    I40E_TX_DESC_LENGTH_L4_FC_LEN_SHIFT;
			}
#ifdef IXL_FDIR
			ixl_atr(que, pi->ipi_tcp_hflags, pi->ipi_etype);
#endif
			break;
		case IPPROTO_UDP:
			if (pi->ipi_csum_flags & (CSUM_UDP|CSUM_UDP_IPV6)) {
				*cmd |= I40E_TX_DESC_CMD_L4T_EOFT_UDP;
				*off |= (sizeof(struct udphdr) >> 2) <<
				    I40E_TX_DESC_LENGTH_L4_FC_LEN_SHIFT;
			}
			break;

		case IPPROTO_SCTP:
			if (pi->ipi_csum_flags & (CSUM_SCTP|CSUM_SCTP_IPV6)) {
				*cmd |= I40E_TX_DESC_CMD_L4T_EOFT_SCTP;
				*off |= (sizeof(struct sctphdr) >> 2) <<
				    I40E_TX_DESC_LENGTH_L4_FC_LEN_SHIFT;
			}
			/* Fall Thru */
		default:
			break;
	}

}

/**********************************************************************
 *
 *  Setup context for hardware segmentation offload (TSO)
 *
 **********************************************************************/
static int
ixl_tso_setup(struct tx_ring *txr, if_pkt_info_t pi)
{
	if_softc_ctx_t			scctx;
	struct i40e_tx_context_desc	*TXD;
	u32				cmd, mss, type, tsolen;
	int				idx;
	u64				type_cmd_tso_mss;

	scctx = txr->que->vsi->shared;
	idx = pi->ipi_pidx;
	TXD = (struct i40e_tx_context_desc *) &txr->tx_base[idx];
	tsolen = pi->ipi_len - (pi->ipi_ehdrlen + pi->ipi_ip_hlen + pi->ipi_tcp_hlen);

	type = I40E_TX_DESC_DTYPE_CONTEXT;
	cmd = I40E_TX_CTX_DESC_TSO;
	mss = pi->ipi_tso_segsz;

	type_cmd_tso_mss = ((u64)type << I40E_TXD_CTX_QW1_DTYPE_SHIFT) |
	    ((u64)cmd << I40E_TXD_CTX_QW1_CMD_SHIFT) |
	    ((u64)tsolen << I40E_TXD_CTX_QW1_TSO_LEN_SHIFT) |
	    ((u64)mss << I40E_TXD_CTX_QW1_MSS_SHIFT);
	TXD->type_cmd_tso_mss = htole64(type_cmd_tso_mss);

	TXD->tunneling_params = htole32(0);

	return ((idx + 1) & (scctx->isc_ntxd[0]-1));
}

/*********************************************************************
 *
 *  This routine maps the mbufs to tx descriptors, allowing the
 *  TX engine to transmit the packets. 
 *  	- return 0 on success, positive on failure
 *
 **********************************************************************/
#define IXL_TXD_CMD (I40E_TX_DESC_CMD_EOP | I40E_TX_DESC_CMD_RS)

static int
ixl_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
	struct ixl_vsi		*vsi = arg;
	if_softc_ctx_t		scctx = vsi->shared;
	struct ixl_tx_queue	*que = &vsi->tx_queues[pi->ipi_qsidx];
	struct tx_ring		*txr = &que->txr;
	int			nsegs = pi->ipi_nsegs;
	bus_dma_segment_t *segs = pi->ipi_segs;
	struct i40e_tx_desc	*txd = NULL;
	int             	i, j, mask;
	u32			cmd, off;

	cmd = off = 0;
	i = pi->ipi_pidx;

	if (pi->ipi_flags & IPI_TX_INTR)
		cmd |= (I40E_TX_DESC_CMD_RS << I40E_TXD_QW1_CMD_SHIFT);

	/* Set up the TSO/CSUM offload */
	if (pi->ipi_csum_flags & CSUM_OFFLOAD) {
		/* Set up the TSO context descriptor if required */
		if (pi->ipi_csum_flags & CSUM_TSO) {
			if (ixl_tso_detect_sparse(segs, nsegs, pi->ipi_tso_segsz))
				return (EFBIG);

			i = ixl_tso_setup(txr, pi);
		}
		ixl_tx_setup_offload(que, pi, &cmd, &off);
	}

	if (pi->ipi_mflags & M_VLANTAG)
		cmd |= I40E_TX_DESC_CMD_IL2TAG1;

	cmd |= I40E_TX_DESC_CMD_ICRC;
	mask = scctx->isc_ntxd[0]-1;
	for (j = 0; j < nsegs; j++) {
		bus_size_t seglen;

		txd = &txr->tx_base[i];
		seglen = segs[j].ds_len;

		txd->buffer_addr = htole64(segs[j].ds_addr);
		txd->cmd_type_offset_bsz =
		    htole64(I40E_TX_DESC_DTYPE_DATA
		    | ((u64)cmd  << I40E_TXD_QW1_CMD_SHIFT)
		    | ((u64)off << I40E_TXD_QW1_OFFSET_SHIFT)
		    | ((u64)seglen  << I40E_TXD_QW1_TX_BUF_SZ_SHIFT)
	            | ((u64)htole16(pi->ipi_vtag)  << I40E_TXD_QW1_L2TAG1_SHIFT));

		i = (i+1) & mask;
	}
	/* Set the last descriptor for report */
	txd->cmd_type_offset_bsz |=
	    htole64(((u64)IXL_TXD_CMD << I40E_TXD_QW1_CMD_SHIFT));
	pi->ipi_new_pidx = i;

	++txr->total_packets;
	return (0);
}

static void
ixl_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx)
{
	struct ixl_vsi *vsi = arg;
	struct tx_ring *txr = &vsi->tx_queues[txqid].txr;
	/*
	 * Advance the Transmit Descriptor Tail (Tdt), this tells the
	 * hardware that this frame is available to transmit.
	 */
	wr32(vsi->hw, txr->tail, pidx);
}

/*********************************************************************
 *
 *  (Re)Initialize a queue transmit ring.
 *	- called by init, it clears the descriptor ring,
 *	  and frees any stale mbufs 
 *
 **********************************************************************/
void
ixl_init_tx_ring(struct ixl_vsi *vsi, struct ixl_tx_queue *que)
{
	struct tx_ring *txr = &que->txr;
	if_softc_ctx_t		scctx = vsi->shared;

	/* Clear the old ring contents */
	bzero((void *)txr->tx_base,
	      (sizeof(struct i40e_tx_desc)) * scctx->isc_ntxd[0]);

#ifdef IXL_FDIR
	/* Initialize flow director */
	txr->atr_rate = ixl_atr_rate;
	txr->atr_count = 0;
#endif
	wr32(vsi->hw, I40E_QTX_TAIL(que->txr.me), 0);
	wr32(vsi->hw, I40E_QTX_HEAD(que->txr.me), 0);
}

/*             
** ixl_get_tx_head - Retrieve the value from the 
**    location the HW records its HEAD index
*/
static inline u32
ixl_get_tx_head(struct ixl_tx_queue *que)
{
	if_softc_ctx_t		scctx = que->vsi->shared;
	struct tx_ring  *txr = &que->txr;
	void *head = &txr->tx_base[scctx->isc_ntxd[0]];

	return LE32_TO_CPU(*(volatile __le32 *)head);
}

/**********************************************************************
 *
 *  Examine each tx_buffer in the used queue. If the hardware is done
 *  processing the packet then free associated resources. The
 *  tx_buffer is put back on the free queue.
 *
 **********************************************************************/
static int
ixl_isc_txd_credits_update(void *arg, uint16_t qid, uint32_t cidx, bool clear)
{
	struct ixl_vsi		*vsi = arg;
	if_softc_ctx_t		scctx = vsi->shared;
	struct ixl_tx_queue	*que = &vsi->tx_queues[qid];

	int head, credits;

	/* Get the Head WB value */
	head = ixl_get_tx_head(que);

	credits = head - cidx;
	if (credits < 0)
		credits += scctx->isc_ntxd[0];
	return (credits);
}

/*********************************************************************
 *
 *  Refresh mbuf buffers for RX descriptor rings
 *   - now keeps its own state so discards due to resource
 *     exhaustion are unnecessary, if an mbuf cannot be obtained
 *     it just returns, keeping its placeholder, thus it can simply
 *     be recalled to try again.
 *
 **********************************************************************/
static void
ixl_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count, uint16_t buf_len)

{
	struct ixl_vsi		*vsi = arg;
	struct rx_ring		*rxr = &vsi->rx_queues[rxqid].rxr;
	int			i, mask;
	uint32_t next_pidx;

	mask = vsi->shared->isc_nrxd[0]-1;
	for (i = 0, next_pidx = pidx; i < count; i++) {
		rxr->rx_base[next_pidx].read.pkt_addr = htole64(paddrs[i]);
		next_pidx = (next_pidx + 1) & mask;
	}
}

static void
ixl_isc_rxd_flush(void * arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx)
{
	struct ixl_vsi		*vsi = arg;
	struct rx_ring		*rxr = &vsi->rx_queues[rxqid].rxr;

	wr32(vsi->hw, rxr->tail, pidx);
}

static int
ixl_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx, int budget)
{
	struct ixl_vsi *vsi = arg;
	struct rx_ring *rxr = &vsi->rx_queues[rxqid].rxr;
	union i40e_rx_desc	*cur;
	u64 qword;
	uint32_t status;
	int cnt, i, mask;

	mask = vsi->shared->isc_nrxd[0]-1;
	for (cnt = 0, i = idx; cnt < vsi->shared->isc_nrxd[0] && cnt <= budget;) {
		cur = &rxr->rx_base[i];
		qword = le64toh(cur->wb.qword1.status_error_len);
		status = (qword & I40E_RXD_QW1_STATUS_MASK)
			>> I40E_RXD_QW1_STATUS_SHIFT;
		if ((status & (1 << I40E_RX_DESC_STATUS_DD_SHIFT)) == 0)
			break;
		if (status & (1 << I40E_RX_DESC_STATUS_EOF_SHIFT))
			cnt++;
		i = (i + 1) & mask;
	}
	return (cnt);
}

/*
** i40e_ptype_to_hash: parse the packet type
** to determine the appropriate hash.
*/
static inline int
ixl_ptype_to_hash(u8 ptype)
{
        struct i40e_rx_ptype_decoded	decoded;
	u8				ex = 0;

	decoded = decode_rx_desc_ptype(ptype);
	ex = decoded.outer_frag;

	if (!decoded.known)
		return M_HASHTYPE_OPAQUE;

	if (decoded.outer_ip == I40E_RX_PTYPE_OUTER_L2) 
		return M_HASHTYPE_OPAQUE;

	/* Note: anything that gets to this point is IP */
        if (decoded.outer_ip_ver == I40E_RX_PTYPE_OUTER_IPV6) { 
		switch (decoded.inner_prot) {
		case I40E_RX_PTYPE_INNER_PROT_TCP:
			if (ex)
				return M_HASHTYPE_RSS_TCP_IPV6_EX;
			else
				return M_HASHTYPE_RSS_TCP_IPV6;
		case I40E_RX_PTYPE_INNER_PROT_UDP:
			if (ex)
				return M_HASHTYPE_RSS_UDP_IPV6_EX;
			else
				return M_HASHTYPE_RSS_UDP_IPV6;
		default:
			if (ex)
				return M_HASHTYPE_RSS_IPV6_EX;
			else
				return M_HASHTYPE_RSS_IPV6;
		}
	}
        if (decoded.outer_ip_ver == I40E_RX_PTYPE_OUTER_IPV4) { 
		switch (decoded.inner_prot) {
		case I40E_RX_PTYPE_INNER_PROT_TCP:
			return M_HASHTYPE_RSS_TCP_IPV4;
		case I40E_RX_PTYPE_INNER_PROT_UDP:
			if (ex)
				return M_HASHTYPE_RSS_UDP_IPV4_EX;
			else
				return M_HASHTYPE_RSS_UDP_IPV4;
		default:
			return M_HASHTYPE_RSS_IPV4;
		}
	}
	/* We should never get here!! */
	return M_HASHTYPE_OPAQUE;
}

/*********************************************************************
 *
 *  This routine executes in ithread context. It sends data which has been
 *  dma'ed into host memory to upper layer.
 *
 *  Returns 0 upon success, errno on failure
 *********************************************************************/

static int
ixl_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	struct ixl_vsi		*vsi = arg;
	if_softc_ctx_t		scctx = vsi->shared;
	struct ixl_rx_queue	*que = &vsi->rx_queues[ri->iri_qsidx];
	struct rx_ring		*rxr = &que->rxr;
	union i40e_rx_desc	*cur;
	u32		status, error;
	u16		hlen, plen, vtag;
	u64		qword;
	u8		ptype;
	bool		eop;
	int i, cidx;

	cidx = ri->iri_cidx;
	i = 0;
	do {
		cur = &rxr->rx_base[cidx];
		qword = le64toh(cur->wb.qword1.status_error_len);
		status = (qword & I40E_RXD_QW1_STATUS_MASK)
			>> I40E_RXD_QW1_STATUS_SHIFT;
		error = (qword & I40E_RXD_QW1_ERROR_MASK)
			>> I40E_RXD_QW1_ERROR_SHIFT;
		plen = (qword & I40E_RXD_QW1_LENGTH_PBUF_MASK)
			>> I40E_RXD_QW1_LENGTH_PBUF_SHIFT;
		hlen = (qword & I40E_RXD_QW1_LENGTH_HBUF_MASK)
			>> I40E_RXD_QW1_LENGTH_HBUF_SHIFT;
		ptype = (qword & I40E_RXD_QW1_PTYPE_MASK)
			>> I40E_RXD_QW1_PTYPE_SHIFT;

		/* we should never be called without a valid descriptor */
		MPASS((status & (1 << I40E_RX_DESC_STATUS_DD_SHIFT)) != 0);

		ri->iri_len += plen;
		rxr->rx_bytes += plen;

		cur->wb.qword1.status_error_len = 0;
		eop = (status & (1 << I40E_RX_DESC_STATUS_EOF_SHIFT));
		if (status & (1 << I40E_RX_DESC_STATUS_L2TAG1P_SHIFT))
			vtag = le16toh(cur->wb.qword0.lo_dword.l2tag1);
		else
			vtag = 0;

		/*
		** Make sure bad packets are discarded,
		** note that only EOP descriptor has valid
		** error results.
		*/
		if (eop && (error & (1 << I40E_RX_DESC_ERROR_RXE_SHIFT))) {
			rxr->discarded++;
			return (EBADMSG);
		}
		ri->iri_frags[i].irf_flid = 0;
		ri->iri_frags[i].irf_idx = cidx;
		ri->iri_frags[i].irf_len = plen;
		if (++cidx == scctx->isc_ntxd[0])
			cidx = 0;
		i++;
		/* even a 16K packet shouldn't consume more than 8 clusters */
		MPASS(i < 9);
	} while (!eop);

	rxr->rx_packets++;
	/* capture data for dynamic ITR adjustment */
	rxr->packets++;
	if ((vsi->ifp->if_capenable & IFCAP_RXCSUM) != 0)
		ixl_rx_checksum(ri, status, error, ptype);
	ri->iri_flowid = le32toh(cur->wb.qword0.hi_dword.rss);
	ri->iri_rsstype = ixl_ptype_to_hash(ptype);
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
ixl_rx_checksum(if_rxd_info_t ri, u32 status, u32 error, u8 ptype)
{
	struct i40e_rx_ptype_decoded decoded;

	decoded = decode_rx_desc_ptype(ptype);
	/* Errors? */
 	if (error & ((1 << I40E_RX_DESC_ERROR_IPE_SHIFT) |
	    (1 << I40E_RX_DESC_ERROR_L4E_SHIFT))) {
		ri->iri_csum_flags = 0;
		return;
	}

	/* IPv6 with extension headers likely have bad csum */
	if (decoded.outer_ip == I40E_RX_PTYPE_OUTER_IP &&
	    decoded.outer_ip_ver == I40E_RX_PTYPE_OUTER_IPV6)
		if (status &
		    (1 << I40E_RX_DESC_STATUS_IPV6EXADD_SHIFT)) {
			ri->iri_csum_flags = 0;
			return;
		}
 
	/* IP Checksum Good */
	ri->iri_csum_flags = CSUM_IP_CHECKED;
	ri->iri_csum_flags |= CSUM_IP_VALID;

	if (status & (1 << I40E_RX_DESC_STATUS_L3L4P_SHIFT)) {
		ri->iri_csum_flags |= 
		    (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
		ri->iri_csum_data |= htons(0xffff);
	}
}
