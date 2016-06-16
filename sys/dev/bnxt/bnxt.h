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

#ifndef _BNXT_H
#define _BNXT_H

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/bus_dma.h>
#include <sys/socket.h>
#include <sys/taskqueue.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/iflib.h>

#include "hsi_struct_def.h"

/* PCI IDs */
#define BROADCOM_VENDOR_ID	0x14E4

#define BCM57301	0x16C8
#define BCM57302	0x16C9
#define BCM57304	0x16CA
#define BCM57402	0x16D0
#define BCM57404	0x16D1
#define BCM57406	0x16D2

#define CSUM_OFFLOAD		(CSUM_IP_TSO|CSUM_IP6_TSO|CSUM_IP| \
				 CSUM_IP_UDP|CSUM_IP_TCP|CSUM_IP_SCTP| \
				 CSUM_IP6_UDP|CSUM_IP6_TCP|CSUM_IP6_SCTP)

/* Completion related defines */
#define CMP_VALID(cmp, raw_cons, ring)					    \
	(!!(((struct cmpl_base *)(cmp))->info3_v & htole32(CMPL_BASE_V)) == \
	 !((raw_cons) & ((ring)->ring_size)))

#define TX_CMP_VALID(cmp, raw_cons, ring)				    \
	(!!(((txcmp))->errors_v & htole32(TX_CMPL_V)) ==		    \
	 !((raw_cons) & ((ring)->ring_size)))

/* Test validity in the extended completion record */
#define RXT_CMP_VALID(rxcmp1, raw_cons, ring )				    \
        (!!((rxcmp1)->errors_v2 & htole32(RX_CMP_V)) ==			    \
	 !((raw_cons) & ((ring)->ring_size)))

/* Test validity of an aggregation buffer */
#define RX_AGG_CMP_VALID(agg, raw_cons, ring)                         \
	(!!((agg)->rx_agg_cmp_v & htole32(RX_AGG_CMP_V)) == \
	!((raw_cons) & ring->ring_size))

#define TX_CMP_TYPE(txcmp) (le32toh((txcmp)->flags_type) & TX_CMPL_TYPE_MASK)

#define RX_CMP_TYPE(rxcmp) (le32toh((rxcmp)->flags_type) & 		    \
	    RX_PKT_CMPL_TYPE_MASK)

#define ADV_RAW_CMP(idx, n)	((idx) + (n))
#define NEXT_RAW_CMP(idx)	ADV_RAW_CMP(idx, 1)
#define RING_CMP(ring, idx)	((idx) & (ring)->ring_mask)
#define RING_NEXT(ring, idx)	(((idx) + 1) & (ring)->ring_mask)
#define TX_CMP_THRESH(ring)	ring->ring_size / 8

/* Doorbell related defines */
#define DB_IDX_MASK	0xffffff
#define DB_IDX_VALID	(0x1<<26)
#define DB_IRQ_DIS	(0x1<<27)
#define DB_KEY_TX	(0x0<<28)
#define DB_KEY_RX	(0x1<<28)
#define DB_KEY_CP	(0x2<<28)
#define DB_KEY_ST	(0x3<<28)
#define DB_KEY_TX_PUSH	(0x4<<28)
#define DB_LONG_TX_PUSH	(0x2<<24)

#define DB_CP_REARM_FLAGS	(DB_KEY_CP | DB_IDX_VALID)
#define DB_CP_FLAGS		(DB_KEY_CP | DB_IDX_VALID | DB_IRQ_DIS)
#define DB_CP_DIS_FLAGS		(DB_KEY_CP | DB_IRQ_DIS)

#define DB_REARM(cpr, cons) (DB_CP_REARM_FLAGS | RING_CMP(&((cpr)->ring), cons))
#define DB_DISABLE(cpr, cons) (DB_CP_DIS_FLAGS | RING_CMP(&((cpr)->ring), cons))
#define DB_RING(cpr, cons) (DB_CP_FLAGS | RING_CMP(&((cpr)->ring), cons))

#define BNXT_TX_DB(db, value) *(uint32_t *)(db) = (DB_KEY_TX | value)
#define BNXT_RX_DB(db, value) *(uint32_t *)(db) = (DB_KEY_RX | value)

#define BNXT_CP_DISABLE_DB(cpr, cons)					    \
	    *(uint32_t *)((cpr)->ring.doorbell) = DB_DISABLE(cpr, cons)
#define BNXT_CP_ARM_DB(cpr, cons) *(uint32_t *)((cpr)->ring.doorbell) = DB_REARM(cpr, cons)
#define BNXT_CP_DB(cpr, cons) *(uint32_t *)((cpr)->ring.doorbell) = DB_RING(cpr, cons)

/* Lock macros */
#define BNXT_HWRM_LOCK_INIT(_softc, _name) \
	mtx_init(&(_softc)->hwrm_lock, _name, "BNXT HWRM Lock", MTX_DEF)
#define BNXT_HWRM_LOCK(_softc)		mtx_lock(&(_softc)->hwrm_lock)
#define BNXT_HWRM_UNLOCK(_softc)	mtx_unlock(&(_softc)->hwrm_lock)
#define BNXT_HWRM_LOCK_DESTROY(_softc)	mtx_destroy(&(_softc)->hwrm_lock)
#define BNXT_HWRM_LOCK_ASSERT(_softc)	mtx_assert(&(_softc)->hwrm_lock, MA_OWNED)

/* Chip info */
#define BNXT_TSO_SIZE	UINT16_MAX
#define BNXT_BARS	3

struct bnxt_bar_info {
	struct resource		*res;
	bus_space_tag_t		tag;
	bus_space_handle_t	handle;
	bus_size_t		size;
	int			rid;
	vm_offset_t		kva;
};

struct bnxt_link_info {
	uint8_t		media_type;
	uint8_t		transceiver;
	uint8_t		phy_addr;
	uint8_t		phy_link_status;
#define BNXT_LINK_NO_LINK	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_NO_LINK
#define BNXT_LINK_SIGNAL	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SIGNAL
#define BNXT_LINK_LINK		HWRM_PORT_PHY_QCFG_OUTPUT_LINK_LINK
	uint8_t		wire_speed;
	uint8_t		loop_back;
	uint8_t		link_up;
	uint8_t		duplex;
#define BNXT_LINK_DUPLEX_HALF	HWRM_PORT_PHY_QCFG_OUTPUT_DUPLEX_HALF
#define BNXT_LINK_DUPLEX_FULL	HWRM_PORT_PHY_QCFG_OUTPUT_DUPLEX_FULL
	uint8_t		pause;
#define BNXT_LINK_PAUSE_TX	HWRM_PORT_PHY_QCFG_OUTPUT_PAUSE_TX
#define BNXT_LINK_PAUSE_RX	HWRM_PORT_PHY_QCFG_OUTPUT_PAUSE_RX
#define BNXT_LINK_PAUSE_BOTH	(HWRM_PORT_PHY_QCFG_OUTPUT_PAUSE_RX |	    \
				 HWRM_PORT_PHY_QCFG_OUTPUT_PAUSE_TX)
	uint8_t		auto_pause;
	uint8_t		force_pause;
	uint8_t		duplex_setting;
	uint8_t		auto_mode;
#define BNXT_AUTO_MODE(mode)	((mode) > BNXT_LINK_AUTO_NONE &&	    \
				 (mode) <= BNXT_LINK_AUTO_MSK)
#define BNXT_LINK_AUTO_NONE	HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_NONE
#define BNXT_LINK_AUTO_ALLSPDS	HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_ALL_SPEEDS
#define BNXT_LINK_AUTO_ONESPD	HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_ONE_SPEED
#define BNXT_LINK_AUTO_ONEORBELOW HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_ONE_OR_BELOW
#define BNXT_LINK_AUTO_MSK	HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_SPEED_MASK
#define PHY_VER_LEN		3
	uint8_t		phy_ver[PHY_VER_LEN];
	uint16_t	link_speed;
#define BNXT_LINK_SPEED_100MB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_100MB
#define BNXT_LINK_SPEED_1GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_1GB
#define BNXT_LINK_SPEED_2GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_2GB
#define BNXT_LINK_SPEED_2_5GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_2_5GB
#define BNXT_LINK_SPEED_10GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_10GB
#define BNXT_LINK_SPEED_20GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_20GB
#define BNXT_LINK_SPEED_25GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_25GB
#define BNXT_LINK_SPEED_40GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_40GB
#define BNXT_LINK_SPEED_50GB	HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_50GB
	uint16_t	support_speeds;
	uint16_t	auto_link_speeds;
#define BNXT_LINK_SPEED_MSK_100MB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MB
#define BNXT_LINK_SPEED_MSK_1GB	HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GB
#define BNXT_LINK_SPEED_MSK_2GB	HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_2GB
#define BNXT_LINK_SPEED_MSK_10GB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10GB
#define BNXT_LINK_SPEED_MSK_2_5GB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_2_5GB
#define BNXT_LINK_SPEED_MSK_20GB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_20GB
#define BNXT_LINK_SPEED_MSK_25GB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_25GB
#define BNXT_LINK_SPEED_MSK_40GB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_40GB
#define BNXT_LINK_SPEED_MSK_50GB HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_50GB
	uint16_t	auto_link_speed;
	uint16_t	force_link_speed;
	uint32_t	preemphasis;

	/* copy of requested setting from ethtool cmd */
	uint8_t		autoneg;
#define BNXT_AUTONEG_SPEED	1
#define BNXT_AUTONEG_FLOW_CTRL	2
	uint8_t		req_duplex;
	uint8_t		req_flow_ctrl;
	uint16_t	req_link_speed;
	uint32_t	advertising;
	bool		force_link_chng;
	/*
	 * a copy of phy_qcfg output used
	 * to report link info to VF
	 */
	struct hwrm_port_phy_qcfg_output phy_qcfg_resp;
};

enum bnxt_cp_type {
	BNXT_DEFAULT,
	BNXT_TX,
	BNXT_RX,
	BNXT_SHARED
};

struct bnxt_cos_queue {
	uint8_t	id;
	uint8_t	profile;
};

struct bnxt_pf_info {
#define BNXT_FIRST_PF_FID	1
#define BNXT_FIRST_VF_FID	128
	uint32_t	fw_fid;
	uint8_t		port_id;
	uint8_t		mac_addr[ETHER_ADDR_LEN];
	uint16_t	max_rsscos_ctxs;
	uint16_t	max_cp_rings;
	uint16_t	max_tx_rings;
	uint16_t	max_rx_rings;
	uint16_t	max_hw_ring_grps;
	uint16_t	max_irqs;
	uint16_t	max_l2_ctxs;
	uint16_t	max_vnics;
	uint16_t	max_stat_ctxs;
	uint32_t	first_vf_id;
	uint16_t	active_vfs;
	uint16_t	max_vfs;
	uint32_t	max_encap_records;
	uint32_t	max_decap_records;
	uint32_t	max_tx_em_flows;
	uint32_t	max_tx_wm_flows;
	uint32_t	max_rx_em_flows;
	uint32_t	max_rx_wm_flows;
	unsigned long	*vf_event_bmap;
	uint16_t	hwrm_cmd_req_pages;
	void		*hwrm_cmd_req_addr[4];
	bus_addr_t	hwrm_cmd_req_dma_addr[4];
};

struct bnxt_vf_info {
	uint16_t	fw_fid;
	uint8_t		mac_addr[ETHER_ADDR_LEN];
	uint16_t	max_rsscos_ctxs;
	uint16_t	max_cp_rings;
	uint16_t	max_tx_rings;
	uint16_t	max_rx_rings;
	uint16_t	max_hw_ring_grps;
	uint16_t	max_l2_ctxs;
	uint16_t	max_irqs;
	uint16_t	max_vnics;
	uint16_t	max_stat_ctxs;
	uint32_t	vlan;
#define BNXT_VF_QOS		0x1
#define BNXT_VF_SPOOFCHK	0x2
#define BNXT_VF_LINK_FORCED	0x4
#define BNXT_VF_LINK_UP		0x8
	uint32_t	flags;
	uint32_t	func_flags; /* func cfg flags */
	uint32_t	min_tx_rate;
	uint32_t	max_tx_rate;
	void		*hwrm_cmd_req_addr;
	bus_addr_t	hwrm_cmd_req_dma_addr;
};

#define BNXT_FLAG_DCB_ENABLED	(1<<0)
#define BNXT_FLAG_VF		(1<<1)
#define BNXT_FLAG_LRO		(1<<2)
#define BNXT_FLAG_GRO		(1<<3)
#define BNXT_FLAG_TPA		(BNXT_FLAG_LRO | BNXT_FLAG_GRO)
#define BNXT_FLAG_JUMBO		(1<<4)
#define BNXT_FLAG_STRIP_VLAN	(1<<5)
#define BNXT_FLAG_AGG_RINGS	(BNXT_FLAG_JUMBO | BNXT_FLAG_GRO | \
					 BNXT_FLAG_LRO)
#define BNXT_FLAG_USING_MSIX	(1<<6)
#define BNXT_FLAG_HWRM_120	0x800
#define BNXT_FLAG_EEE_CAP	0x1000

#define BNXT_PF(softc)		(!((softc)->flags & BNXT_FLAG_VF))
#define BNXT_VF(softc)		((softc)->flags & BNXT_FLAG_VF)

struct bnxt_vnic_info {
	uint16_t	id;
	uint16_t	ring_grp;
	uint16_t	rss_rule;
	uint16_t	cos_rule;
	uint16_t	lb_rule;
	uint16_t	mru;

	uint16_t	ctx_id;

	uint32_t	rx_mask;
	struct iflib_dma_info mc_list;
	int		mc_list_size;
	int		mc_list_count;
#define BNXT_MAX_MC_ADDRS	16

	uint32_t	flags;
#define BNXT_VNIC_FLAG_DEFAULT	1
};

struct bnxt_grp_info {
	uint16_t	stats_ctx;
	uint16_t	grp_id;
	uint16_t	rx_ring_id;
	uint16_t	cp_ring_id;
	uint16_t	ag_ring_id;
};

struct bnxt_ring {
	uint64_t		paddr;
	vm_offset_t		doorbell;
	caddr_t			vaddr;
	struct bnxt_softc	*softc;
	uint32_t		ring_size;	// Must be a power of two
	uint32_t		ring_mask;	// Mask for ring_size (ring_size-1)
	uint16_t		id;		// Logical ID
	uint16_t		phys_id;
};

struct bnxt_cp_ring {
	struct bnxt_ring	ring;
	struct if_irq		irq;
	uint32_t		raw_cons;
	struct ctx_hw_stats	*stats;
	uint32_t		stats_ctx_id;
};

struct bnxt_tx_ring {
	struct bnxt_ring	ring;
	uint8_t			cos_queue_id;
	uint16_t		prod; /* Producer index */
	uint16_t		cons; /* Consumer index */
	struct bnxt_cp_ring	*cp_ring;
};

struct bnxt_rx_ring {
	struct bnxt_ring	ring;
	uint16_t		mbuf_sz;
	uint16_t		prod;
	uint16_t		cons;
};

struct bnxt_softc {
	device_t	dev;
	if_ctx_t	ctx;
	if_softc_ctx_t	scctx;
	if_shared_ctx_t	sctx;
	struct ifmedia	*media;

	struct bnxt_bar_info	bar[3]; 
	struct bnxt_link_info	link_info;
#define FW_VER_STR_LEN		32
#define BC_HWRM_STR_LEN		21
#define PHY_VER_STR_LEN		(FW_VER_STR_LEN - BC_HWRM_STR_LEN)
	char			fw_ver_str[FW_VER_STR_LEN];
	uint32_t		flags;
	uint32_t		total_msix;

	struct bnxt_pf_info	pf;
	struct bnxt_vf_info	vf;

	uint16_t		hwrm_cmd_seq;
	uint16_t		hwrm_cmd_timeo;	/* milliseconds */
	struct iflib_dma_info	hwrm_cmd_resp;
	/* Interrupt info for HWRM */
	struct if_irq		irq;
	struct mtx		hwrm_lock;
	uint16_t		hwrm_max_req_len;


#define BNXT_MAX_QUEUE		8
	uint8_t			max_tc;
	struct bnxt_cos_queue	q_info[BNXT_MAX_QUEUE];

	struct iflib_dma_info	hw_rx_port_stats;
	struct iflib_dma_info	hw_tx_port_stats;
	struct rx_port_stats	*rx_port_stats;
	struct tx_port_stats	*tx_port_stats;

	int			num_cp_rings;

	struct bnxt_tx_ring	*tx_rings;
	struct bnxt_cp_ring	*tx_cp_rings;
	struct iflib_dma_info	tx_stats;
	int			ntxqsets;

	struct bnxt_rx_ring	*ag_rings;
	struct bnxt_rx_ring	*rx_rings;
	struct bnxt_cp_ring	*rx_cp_rings;
	struct bnxt_vnic_info	*vnic_info;
	struct bnxt_grp_info	*grp_info;
	struct iflib_dma_info	rx_stats;
	int			nrxqsets;

	struct bnxt_cp_ring	def_cp_ring;
	struct iflib_dma_info	def_cp_ring_mem;
};

struct bnxt_filter_info {
	STAILQ_ENTRY(bnxt_filter_info) next;
	uint64_t	fw_l2_filter_id;
#define INVALID_MAC_INDEX ((uint16_t)-1)
	uint16_t	mac_index;

	/* Filter Characteristics */
	uint32_t	flags;
	uint32_t	enables;
	uint8_t		l2_addr[ETHER_ADDR_LEN];
	uint8_t		l2_addr_mask[ETHER_ADDR_LEN];
	uint16_t	l2_ovlan;
	uint16_t	l2_ovlan_mask;
	uint16_t	l2_ivlan;
	uint16_t	l2_ivlan_mask;
	uint8_t		t_l2_addr[ETHER_ADDR_LEN];
	uint8_t		t_l2_addr_mask[ETHER_ADDR_LEN];
	uint16_t	t_l2_ovlan;
	uint16_t	t_l2_ovlan_mask;
	uint16_t	t_l2_ivlan;
	uint16_t	t_l2_ivlan_mask;
	uint8_t		tunnel_type;
	uint16_t	mirror_vnic_id;
	uint32_t	vni;
	uint8_t		pri_hint;
	uint64_t	l2_filter_id_hint;
};

/* Function declarations */
void bnxt_report_link(struct bnxt_softc *softc);

#endif /* _BNXT_H */
