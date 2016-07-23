/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2014-2015 Broadcom Corporation.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Broadcom Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _HSI_STRUCT_DEF_EXTERNAL_H_
#define _HSI_STRUCT_DEF_EXTERNAL_H_

/*
 * per-context HW statistics -- chip view
 * Reference to stat_ctx_stat_xxx for
 */

struct ctx_hw_stats {
	uint64_t rx_ucast_pkts;
	uint64_t rx_mcast_pkts;
	uint64_t rx_bcast_pkts;
	uint64_t rx_discard_pkts;
	uint64_t rx_drop_pkts;
	uint64_t rx_ucast_bytes;
	uint64_t rx_mcast_bytes;
	uint64_t rx_bcast_bytes;
	uint64_t tx_ucast_pkts;
	uint64_t tx_mcast_pkts;
	uint64_t tx_bcast_pkts;
	uint64_t tx_discard_pkts;
	uint64_t tx_drop_pkts;
	uint64_t tx_ucast_bytes;
	uint64_t tx_mcast_bytes;
	uint64_t tx_bcast_bytes;
	uint64_t tpa_pkts;
	uint64_t tpa_bytes;
	uint64_t tpa_events;
	uint64_t tpa_aborts;
} __attribute__((packed));

/* BD Ring Structures */
/*
 * Description: This structure is used to inform the NIC of a location for and
 * an aggregation buffer that will be used for packet data that is received. An
 * aggregation buffer creates a different kind of completion operation for a
 * packet where a variable number of BDs may be used to place the packet in the
 * host. RX Rings that have aggregation buffers are known as aggregation rings
 * and must contain only aggregation buffers.
 */
/*
 * Note: BD Ring structures are written by the driver to TX Rings and RX Rings
 * to indicate to the chip there is more buffer space in the host that needs to
 * be transmitted or is available for receive data.
 */
/* BD Base (8 bytes) */

struct bd_base {
	uint8_t type;
	/* This value identifies the type of buffer descriptor. */
	#define BD_BASE_TYPE_MASK				UINT32_C(0x3f)
	#define BD_BASE_TYPE_SFT				0
	/*
	 * Indicates that this BD is 16B long and is used for normal L2
	 * packet transmission.
	 */
	#define BD_BASE_TYPE_TX_BD_SHORT			(UINT32_C(0x0) << 0)
	/*
	 * Indicates that this BD is 1BB long and is an empty TX BD. Not
	 * valid for use by the driver.
	 */
	#define BD_BASE_TYPE_TX_BD_EMPTY			(UINT32_C(0x1) << 0)
	/*
	 * Indicates that this BD is 16B long and is an RX Producer (ie.
	 * empty) buffer descriptor.
	 */
	#define BD_BASE_TYPE_RX_PROD_PKT			(UINT32_C(0x4) << 0)
	/*
	 * Indicates that this BD is 16B long and is an RX Producer
	 * Buffer BD.
	 */
	#define BD_BASE_TYPE_RX_PROD_BFR			(UINT32_C(0x5) << 0)
	/*
	 * Indicates that this BD is 16B long and is an RX Producer
	 * Assembly Buffer Descriptor.
	 */
	#define BD_BASE_TYPE_RX_PROD_AGG			(UINT32_C(0x6) << 0)
	/*
	 * Indicates that this BD is 32B long and is used for normal L2
	 * packet transmission.
	 */
	#define BD_BASE_TYPE_TX_BD_LONG			(UINT32_C(0x10) << 0)
	uint8_t unused_1[7];
} __attribute__((packed));

/* Short TX BD (16 bytes) */

struct tx_bd_short {
	uint16_t flags_type;
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Only the packet_end bit must be valid for the remaining BDs of a
	 * packet.
	 */
	/* This value identifies the type of buffer descriptor. */
	#define TX_BD_SHORT_TYPE_MASK				UINT32_C(0x3f)
	#define TX_BD_SHORT_TYPE_SFT				0
	/*
	 * Indicates that this BD is 16B long and is used for normal L2
	 * packet transmission.
	 */
	#define TX_BD_SHORT_TYPE_TX_BD_SHORT			(UINT32_C(0x0) << 0)
	/*
	 * If set to 1, the packet ends with the data in the buffer pointed to
	 * by this descriptor. This flag must be valid on every BD.
	 */
	#define TX_BD_SHORT_FLAGS_PACKET_END			UINT32_C(0x40)
	/*
	 * If set to 1, the device will not generate a completion for this
	 * transmit packet unless there is an error in it's processing. If this
	 * bit is set to 0, then the packet will be completed normally. This bit
	 * must be valid only on the first BD of a packet.
	 */
	#define TX_BD_SHORT_FLAGS_NO_CMPL			UINT32_C(0x80)
	/*
	 * This value indicates how many 16B BD locations are consumed in the
	 * ring by this packet. A value of 1 indicates that this BD is the only
	 * BD (and that the it is a short BD). A value of 3 indicates either 3
	 * short BDs or 1 long BD and one short BD in the packet. A value of 0
	 * indicates that there are 32 BD locations in the packet (the maximum).
	 * This field is valid only on the first BD of a packet.
	 */
	#define TX_BD_SHORT_FLAGS_BD_CNT_MASK			UINT32_C(0x1f00)
	#define TX_BD_SHORT_FLAGS_BD_CNT_SFT			8
	/*
	 * This value is a hint for the length of the entire packet. It is used
	 * by the chip to optimize internal processing. The packet will be
	 * dropped if the hint is too short. This field is valid only on the
	 * first BD of a packet.
	 */
	#define TX_BD_SHORT_FLAGS_LHINT_MASK			UINT32_C(0x6000)
	#define TX_BD_SHORT_FLAGS_LHINT_SFT			13
	/* indicates packet length < 512B */
	#define TX_BD_SHORT_FLAGS_LHINT_LT512			(UINT32_C(0x0) << 13)
	/* indicates 512 <= packet length < 1KB */
	#define TX_BD_SHORT_FLAGS_LHINT_LT1K			(UINT32_C(0x1) << 13)
	/* indicates 1KB <= packet length < 2KB */
	#define TX_BD_SHORT_FLAGS_LHINT_LT2K			(UINT32_C(0x2) << 13)
	/* indicates packet length >= 2KB */
	#define TX_BD_SHORT_FLAGS_LHINT_GTE2K			(UINT32_C(0x3) << 13)
	#define TX_BD_SHORT_FLAGS_LHINT_LAST	TX_BD_SHORT_FLAGS_LHINT_GTE2K
	/*
	 * If set to 1, the device immediately updates the Send Consumer Index
	 * after the buffer associated with this descriptor has been transferred
	 * via DMA to NIC memory from host memory. An interrupt may or may not
	 * be generated according to the state of the interrupt avoidance
	 * mechanisms. If this bit is set to 0, then the Consumer Index is only
	 * updated as soon as one of the host interrupt coalescing conditions
	 * has been met. This bit must be valid on the first BD of a packet.
	 */
	#define TX_BD_SHORT_FLAGS_COAL_NOW			UINT32_C(0x8000)
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Only the packet_end bit must be valid for the remaining BDs of a
	 * packet.
	 */
	#define TX_BD_SHORT_FLAGS_MASK				UINT32_C(0xffc0)
	#define TX_BD_SHORT_FLAGS_SFT				6
	uint16_t len;
	/*
	 * This is the length of the host physical buffer this BD describes in
	 * bytes. This field must be valid on all BDs of a packet.
	 */
	uint32_t opaque;
	/*
	 * The opaque data field is pass through to the completion and can be
	 * used for any data that the driver wants to associate with the
	 * transmit BD. This field must be valid on the first BD of a packet.
	 */
	uint64_t addr;
	/*
	 * This is the host physical address for the portion of the packet
	 * described by this TX BD. This value must be valid on all BDs of a
	 * packet.
	 */
} __attribute__((packed));

/* Long TX BD (32 bytes split to 2 16-byte struct) */

struct tx_bd_long {
	uint16_t flags_type;
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Only the packet_end bit must be valid for the remaining BDs of a
	 * packet.
	 */
	/* This value identifies the type of buffer descriptor. */
	#define TX_BD_LONG_TYPE_MASK				UINT32_C(0x3f)
	#define TX_BD_LONG_TYPE_SFT				0
	/*
	 * Indicates that this BD is 32B long and is used for normal L2
	 * packet transmission.
	 */
	#define TX_BD_LONG_TYPE_TX_BD_LONG			(UINT32_C(0x10) << 0)
	/*
	 * If set to 1, the packet ends with the data in the buffer pointed to
	 * by this descriptor. This flag must be valid on every BD.
	 */
	#define TX_BD_LONG_FLAGS_PACKET_END			UINT32_C(0x40)
	/*
	 * If set to 1, the device will not generate a completion for this
	 * transmit packet unless there is an error in it's processing. If this
	 * bit is set to 0, then the packet will be completed normally. This bit
	 * must be valid only on the first BD of a packet.
	 */
	#define TX_BD_LONG_FLAGS_NO_CMPL			UINT32_C(0x80)
	/*
	 * This value indicates how many 16B BD locations are consumed in the
	 * ring by this packet. A value of 1 indicates that this BD is the only
	 * BD (and that the it is a short BD). A value of 3 indicates either 3
	 * short BDs or 1 long BD and one short BD in the packet. A value of 0
	 * indicates that there are 32 BD locations in the packet (the maximum).
	 * This field is valid only on the first BD of a packet.
	 */
	#define TX_BD_LONG_FLAGS_BD_CNT_MASK			UINT32_C(0x1f00)
	#define TX_BD_LONG_FLAGS_BD_CNT_SFT			8
	/*
	 * This value is a hint for the length of the entire packet. It is used
	 * by the chip to optimize internal processing. The packet will be
	 * dropped if the hint is too short. This field is valid only on the
	 * first BD of a packet.
	 */
	#define TX_BD_LONG_FLAGS_LHINT_MASK			UINT32_C(0x6000)
	#define TX_BD_LONG_FLAGS_LHINT_SFT			13
	/* indicates packet length < 512B */
	#define TX_BD_LONG_FLAGS_LHINT_LT512			(UINT32_C(0x0) << 13)
	/* indicates 512 <= packet length < 1KB */
	#define TX_BD_LONG_FLAGS_LHINT_LT1K			(UINT32_C(0x1) << 13)
	/* indicates 1KB <= packet length < 2KB */
	#define TX_BD_LONG_FLAGS_LHINT_LT2K			(UINT32_C(0x2) << 13)
	/* indicates packet length >= 2KB */
	#define TX_BD_LONG_FLAGS_LHINT_GTE2K			(UINT32_C(0x3) << 13)
	#define TX_BD_LONG_FLAGS_LHINT_LAST	TX_BD_LONG_FLAGS_LHINT_GTE2K
	/*
	 * If set to 1, the device immediately updates the Send Consumer Index
	 * after the buffer associated with this descriptor has been transferred
	 * via DMA to NIC memory from host memory. An interrupt may or may not
	 * be generated according to the state of the interrupt avoidance
	 * mechanisms. If this bit is set to 0, then the Consumer Index is only
	 * updated as soon as one of the host interrupt coalescing conditions
	 * has been met. This bit must be valid on the first BD of a packet.
	 */
	#define TX_BD_LONG_FLAGS_COAL_NOW			UINT32_C(0x8000)
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Only the packet_end bit must be valid for the remaining BDs of a
	 * packet.
	 */
	#define TX_BD_LONG_FLAGS_MASK				UINT32_C(0xffc0)
	#define TX_BD_LONG_FLAGS_SFT				6
	uint16_t len;
	/*
	 * This is the length of the host physical buffer this BD describes in
	 * bytes. This field must be valid on all BDs of a packet.
	 */
	uint32_t opaque;
	/*
	 * The opaque data field is pass through to the completion and can be
	 * used for any data that the driver wants to associate with the
	 * transmit BD. This field must be valid on the first BD of a packet.
	 */
	uint64_t addr;
	/*
	 * This is the host physical address for the portion of the packet
	 * described by this TX BD. This value must be valid on all BDs of a
	 * packet.
	 */
} __attribute__((packed));

/* last 16 bytes of Long TX BD */

struct tx_bd_long_hi {
	uint16_t lflags;
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Their value on other BDs of the packet will be ignored.
	 */
	/*
	 * If set to 1, the controller replaces the TCP/UPD checksum fields of
	 * normal TCP/UPD checksum, or the inner TCP/UDP checksum field of the
	 * encapsulated TCP/UDP packets with the hardware calculated TCP/UDP
	 * checksum for the packet associated with this descriptor. The flag is
	 * ignored if the LSO flag is set. This bit must be valid on the first
	 * BD of a packet.
	 */
	#define TX_BD_LONG_LFLAGS_TCP_UDP_CHKSUM		UINT32_C(0x1)
	/*
	 * If set to 1, the controller replaces the IP checksum of the normal
	 * packets, or the inner IP checksum of the encapsulated packets with
	 * the hardware calculated IP checksum for the packet associated with
	 * this descriptor. This bit must be valid on the first BD of a packet.
	 */
	#define TX_BD_LONG_LFLAGS_IP_CHKSUM			UINT32_C(0x2)
	/*
	 * If set to 1, the controller will not append an Ethernet CRC to the
	 * end of the frame. This bit must be valid on the first BD of a packet.
	 * Packet must be 64B or longer when this flag is set. It is not useful
	 * to use this bit with any form of TX offload such as CSO or LSO. The
	 * intent is that the packet from the host already has a valid Ethernet
	 * CRC on the packet.
	 */
	#define TX_BD_LONG_LFLAGS_NOCRC				UINT32_C(0x4)
	/*
	 * If set to 1, the device will record the time at which the packet was
	 * actually transmitted at the TX MAC. This bit must be valid on the
	 * first BD of a packet.
	 */
	#define TX_BD_LONG_LFLAGS_STAMP				UINT32_C(0x8)
	/*
	 * If set to 1, The controller replaces the tunnel IP checksum field
	 * with hardware calculated IP checksum for the IP header of the packet
	 * associated with this descriptor. For outer UDP checksum, global outer
	 * UDP checksum TE_NIC register needs to be enabled. If the global outer
	 * UDP checksum TE_NIC register bit is set, outer UDP checksum will be
	 * calculated for the following cases: 1. Packets with tcp_udp_chksum
	 * flag set to offload checksum for inner packet AND the inner packet is
	 * TCP/UDP. If the inner packet is ICMP for example (non-TCP/UDP), even
	 * if the tcp_udp_chksum is set, the outer UDP checksum will not be
	 * calculated. 2. Packets with lso flag set which implies inner TCP
	 * checksum calculation as part of LSO operation.
	 */
	#define TX_BD_LONG_LFLAGS_T_IP_CHKSUM			UINT32_C(0x10)
	/*
	 * If set to 1, the device will treat this packet with LSO(Large Send
	 * Offload) processing for both normal or encapsulated packets, which is
	 * a form of TCP segmentation. When this bit is 1, the hdr_size and mss
	 * fields must be valid. The driver doesn't need to set t_ip_chksum,
	 * ip_chksum, and tcp_udp_chksum flags since the controller will replace
	 * the appropriate checksum fields for segmented packets. When this bit
	 * is 1, the hdr_size and mss fields must be valid.
	 */
	#define TX_BD_LONG_LFLAGS_LSO				UINT32_C(0x20)
	/*
	 * If set to zero when LSO is '1', then the IPID will be treated as a
	 * 16b number and will be wrapped if it exceeds a value of 0xffff. If
	 * set to one when LSO is '1', then the IPID will be treated as a 15b
	 * number and will be wrapped if it exceeds a value 0f 0x7fff.
	 */
	#define TX_BD_LONG_LFLAGS_IPID_FMT			UINT32_C(0x40)
	/*
	 * If set to zero when LSO is '1', then the IPID of the tunnel IP header
	 * will not be modified during LSO operations. If set to one when LSO is
	 * '1', then the IPID of the tunnel IP header will be incremented for
	 * each subsequent segment of an LSO operation. The flag is ignored if
	 * the LSO packet is a normal (non-tunneled) TCP packet.
	 */
	#define TX_BD_LONG_LFLAGS_T_IPID			UINT32_C(0x80)
	/*
	 * If set to '1', then the RoCE ICRC will be appended to the packet.
	 * Packet must be a valid RoCE format packet.
	 */
	#define TX_BD_LONG_LFLAGS_ROCE_CRC			UINT32_C(0x100)
	/*
	 * If set to '1', then the FCoE CRC will be appended to the packet.
	 * Packet must be a valid FCoE format packet.
	 */
	#define TX_BD_LONG_LFLAGS_FCOE_CRC			UINT32_C(0x200)
	uint16_t hdr_size;
	/*
	 * When LSO is '1', this field must contain the offset of the TCP
	 * payload from the beginning of the packet in as 16b words. In case of
	 * encapsulated/tunneling packet, this field contains the offset of the
	 * inner TCP payload from beginning of the packet as 16-bit words. This
	 * value must be valid on the first BD of a packet.
	 */
	#define TX_BD_LONG_HDR_SIZE_MASK			UINT32_C(0x1ff)
	#define TX_BD_LONG_HDR_SIZE_SFT				0
	uint32_t mss;
	/*
	 * This is the MSS value that will be used to do the LSO processing. The
	 * value is the length in bytes of the TCP payload for each segment
	 * generated by the LSO operation. This value must be valid on the first
	 * BD of a packet.
	 */
	#define TX_BD_LONG_MSS_MASK				UINT32_C(0x7fff)
	#define TX_BD_LONG_MSS_SFT				0
	uint16_t unused_2;
	uint16_t cfa_action;
	/*
	 * This value selects a CFA action to perform on the packet. Set this
	 * value to zero if no CFA action is desired. This value must be valid
	 * on the first BD of a packet.
	 */
	uint32_t cfa_meta;
	/*
	 * This value is action meta-data that defines CFA edit operations that
	 * are done in addition to any action editing.
	 */
	/* When key=1, This is the VLAN tag VID value. */
	#define TX_BD_LONG_CFA_META_VLAN_VID_MASK		UINT32_C(0xfff)
	#define TX_BD_LONG_CFA_META_VLAN_VID_SFT		0
	/* When key=1, This is the VLAN tag DE value. */
	#define TX_BD_LONG_CFA_META_VLAN_DE			UINT32_C(0x1000)
	/* When key=1, This is the VLAN tag PRI value. */
	#define TX_BD_LONG_CFA_META_VLAN_PRI_MASK		UINT32_C(0xe000)
	#define TX_BD_LONG_CFA_META_VLAN_PRI_SFT		13
	/* When key=1, This is the VLAN tag TPID select value. */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_MASK		UINT32_C(0x70000)
	#define TX_BD_LONG_CFA_META_VLAN_TPID_SFT		16
	/* 0x88a8 */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_TPID88A8		(UINT32_C(0x0) << 16)
	/* 0x8100 */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_TPID8100		(UINT32_C(0x1) << 16)
	/* 0x9100 */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_TPID9100		(UINT32_C(0x2) << 16)
	/* 0x9200 */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_TPID9200		(UINT32_C(0x3) << 16)
	/* 0x9300 */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_TPID9300		(UINT32_C(0x4) << 16)
	/* Value programmed in CFA VLANTPID register. */
	#define TX_BD_LONG_CFA_META_VLAN_TPID_TPIDCFG		(UINT32_C(0x5) << 16)
	#define TX_BD_LONG_CFA_META_VLAN_TPID_LAST	TX_BD_LONG_CFA_META_VLAN_TPID_TPIDCFG
	/* When key=1, This is the VLAN tag TPID select value. */
	#define TX_BD_LONG_CFA_META_VLAN_RESERVED_MASK		UINT32_C(0xff80000)
	#define TX_BD_LONG_CFA_META_VLAN_RESERVED_SFT		19
	/*
	 * This field identifies the type of edit to be performed on the packet.
	 * This value must be valid on the first BD of a packet.
	 */
	#define TX_BD_LONG_CFA_META_KEY_MASK			UINT32_C(0xf0000000)
	#define TX_BD_LONG_CFA_META_KEY_SFT			28
	/* No editing */
	#define TX_BD_LONG_CFA_META_KEY_NONE			(UINT32_C(0x0) << 28)
	/*
	 * - meta[17:16] - TPID select value (0 = 0x8100). - meta[15:12]
	 * - PRI/DE value. - meta[11:0] - VID value.
	 */
	#define TX_BD_LONG_CFA_META_KEY_VLAN_TAG		(UINT32_C(0x1) << 28)
	#define TX_BD_LONG_CFA_META_KEY_LAST	TX_BD_LONG_CFA_META_KEY_VLAN_TAG
} __attribute__((packed));

/* Empty TX BD (16 bytes) */

struct tx_bd_empty {
	uint8_t type;
	/* This value identifies the type of buffer descriptor. */
	#define TX_BD_EMPTY_TYPE_MASK				UINT32_C(0x3f)
	#define TX_BD_EMPTY_TYPE_SFT				0
	/*
	 * Indicates that this BD is 1BB long and is an empty TX BD. Not
	 * valid for use by the driver.
	 */
	#define TX_BD_EMPTY_TYPE_TX_BD_EMPTY			(UINT32_C(0x1) << 0)
	uint8_t unused_1[3];
	uint8_t unused_2;
	uint8_t unused_3[3];
	uint64_t unused_4;
} __attribute__((packed));

/* RX Producer Packet BD (16 bytes) */

struct rx_prod_pkt_bd {
	uint16_t flags_type;
	/* This value identifies the type of buffer descriptor. */
	#define RX_PROD_PKT_BD_TYPE_MASK			UINT32_C(0x3f)
	#define RX_PROD_PKT_BD_TYPE_SFT				0
	/*
	 * Indicates that this BD is 16B long and is an RX Producer (ie.
	 * empty) buffer descriptor.
	 */
	#define RX_PROD_PKT_BD_TYPE_RX_PROD_PKT		(UINT32_C(0x4) << 0)
	/*
	 * If set to 1, the packet will be placed at the address plus 2B. The 2
	 * Bytes of padding will be written as zero.
	 */
	/*
	 * This is intended to be used when the host buffer is cache-line
	 * aligned to produce packets that are easy to parse in host memory
	 * while still allowing writes to be cache line aligned.
	 */
	#define RX_PROD_PKT_BD_FLAGS_SOP_PAD			UINT32_C(0x40)
	/*
	 * If set to 1, the packet write will be padded out to the nearest
	 * cache-line with zero value padding.
	 */
	/*
	 * If receive buffers start/end on cache-line boundaries, this feature
	 * will ensure that all data writes on the PCI bus start/end on cache
	 * line boundaries.
	 */
	#define RX_PROD_PKT_BD_FLAGS_EOP_PAD			UINT32_C(0x80)
	/*
	 * This value is the number of additional buffers in the ring that
	 * describe the buffer space to be consumed for the this packet. If the
	 * value is zero, then the packet must fit within the space described by
	 * this BD. If this value is 1 or more, it indicates how many additional
	 * "buffer" BDs are in the ring immediately following this BD to be used
	 * for the same network packet. Even if the packet to be placed does not
	 * need all the additional buffers, they will be consumed anyway.
	 */
	#define RX_PROD_PKT_BD_FLAGS_BUFFERS_MASK		UINT32_C(0x300)
	#define RX_PROD_PKT_BD_FLAGS_BUFFERS_SFT		8
	#define RX_PROD_PKT_BD_FLAGS_MASK			UINT32_C(0xffc0)
	#define RX_PROD_PKT_BD_FLAGS_SFT			6
	uint16_t len;
	/*
	 * This is the length in Bytes of the host physical buffer where data
	 * for the packet may be placed in host memory.
	 */
	/*
	 * While this is a Byte resolution value, it is often advantageous to
	 * ensure that the buffers provided end on a host cache line.
	 */
	uint32_t opaque;
	/*
	 * The opaque data field is pass through to the completion and can be
	 * used for any data that the driver wants to associate with this
	 * receive buffer set.
	 */
	uint64_t addr;
	/*
	 * This is the host physical address where data for the packet may by
	 * placed in host memory.
	 */
	/*
	 * While this is a Byte resolution value, it is often advantageous to
	 * ensure that the buffers provide start on a host cache line.
	 */
} __attribute__((packed));

/* RX Producer Buffer BD (16 bytes) */

struct rx_prod_bfr_bd {
	uint16_t flags_type;
	/* This value identifies the type of buffer descriptor. */
	#define RX_PROD_BFR_BD_TYPE_MASK			UINT32_C(0x3f)
	#define RX_PROD_BFR_BD_TYPE_SFT				0
	/*
	 * Indicates that this BD is 16B long and is an RX Producer
	 * Buffer BD.
	 */
	#define RX_PROD_BFR_BD_TYPE_RX_PROD_BFR		(UINT32_C(0x5) << 0)
	#define RX_PROD_BFR_BD_FLAGS_MASK			UINT32_C(0xffc0)
	#define RX_PROD_BFR_BD_FLAGS_SFT			6
	uint16_t len;
	/*
	 * This is the length in Bytes of the host physical buffer where data
	 * for the packet may be placed in host memory.
	 */
	/*
	 * While this is a Byte resolution value, it is often advantageous to
	 * ensure that the buffers provided end on a host cache line.
	 */
	uint32_t opaque;
	/* This field is not used. */
	uint64_t addr;
	/*
	 * This is the host physical address where data for the packet may by
	 * placed in host memory.
	 */
	/*
	 * While this is a Byte resolution value, it is often advantageous to
	 * ensure that the buffers provide start on a host cache line.
	 */
} __attribute__((packed));

/* RX Producer Aggregation BD (16 bytes) */

struct rx_prod_agg_bd {
	uint16_t flags_type;
	/* This value identifies the type of buffer descriptor. */
	#define RX_PROD_AGG_BD_TYPE_MASK			UINT32_C(0x3f)
	#define RX_PROD_AGG_BD_TYPE_SFT				0
	/*
	 * Indicates that this BD is 16B long and is an RX Producer
	 * Assembly Buffer Descriptor.
	 */
	#define RX_PROD_AGG_BD_TYPE_RX_PROD_AGG		(UINT32_C(0x6) << 0)
	/*
	 * If set to 1, the packet write will be padded out to the nearest
	 * cache-line with zero value padding.
	 */
	/*
	 * If receive buffers start/end on cache-line boundaries, this feature
	 * will ensure that all data writes on the PCI bus end on cache line
	 * boundaries.
	 */
	#define RX_PROD_AGG_BD_FLAGS_EOP_PAD			UINT32_C(0x40)
	#define RX_PROD_AGG_BD_FLAGS_MASK			UINT32_C(0xffc0)
	#define RX_PROD_AGG_BD_FLAGS_SFT			6
	uint16_t len;
	/*
	 * This is the length in Bytes of the host physical buffer where data
	 * for the packet may be placed in host memory.
	 */
	/*
	 * While this is a Byte resolution value, it is often advantageous to
	 * ensure that the buffers provided end on a host cache line.
	 */
	uint32_t opaque;
	/*
	 * The opaque data field is pass through to the completion and can be
	 * used for any data that the driver wants to associate with this
	 * receive assembly buffer.
	 */
	uint64_t addr;
	/*
	 * This is the host physical address where data for the packet may by
	 * placed in host memory.
	 */
	/*
	 * While this is a Byte resolution value, it is often advantageous to
	 * ensure that the buffers provide start on a host cache line.
	 */
} __attribute__((packed));

/* Completion Ring Structures */
/* Note: This structure is used by the HWRM to communicate HWRM Error. */
/* Base Completion Record (16 bytes) */

struct cmpl_base {
	uint16_t type;
	/* unused is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define CMPL_BASE_TYPE_MASK				UINT32_C(0x3f)
	#define CMPL_BASE_TYPE_SFT				0
	/* TX L2 completion: Completion of TX packet. Length = 16B */
	#define CMPL_BASE_TYPE_TX_L2				(UINT32_C(0x0) << 0)
	/* RX L2 completion: Completion of and L2 RX packet. Length = 32B */
	#define CMPL_BASE_TYPE_RX_L2				(UINT32_C(0x11) << 0)
	/*
	 * RX Aggregation Buffer completion : Completion of an L2
	 * aggregation buffer in support of TPA, HDS, or Jumbo packet
	 * completion. Length = 16B
	 */
	#define CMPL_BASE_TYPE_RX_AGG				(UINT32_C(0x12) << 0)
	/*
	 * RX L2 TPA Start Completion: Completion at the beginning of a
	 * TPA operation. Length = 32B
	 */
	#define CMPL_BASE_TYPE_RX_TPA_START			(UINT32_C(0x13) << 0)
	/*
	 * RX L2 TPA End Completion: Completion at the end of a TPA
	 * operation. Length = 32B
	 */
	#define CMPL_BASE_TYPE_RX_TPA_END			(UINT32_C(0x15) << 0)
	/*
	 * Statistics Ejection Completion: Completion of statistics data
	 * ejection buffer. Length = 16B
	 */
	#define CMPL_BASE_TYPE_STAT_EJECT			(UINT32_C(0x1a) << 0)
	/* HWRM Command Completion: Completion of an HWRM command. */
	#define CMPL_BASE_TYPE_HWRM_DONE			(UINT32_C(0x20) << 0)
	/* Forwarded HWRM Request */
	#define CMPL_BASE_TYPE_HWRM_FWD_REQ			(UINT32_C(0x22) << 0)
	/* Forwarded HWRM Response */
	#define CMPL_BASE_TYPE_HWRM_FWD_RESP			(UINT32_C(0x24) << 0)
	/* HWRM Asynchronous Event Information */
	#define CMPL_BASE_TYPE_HWRM_ASYNC_EVENT		(UINT32_C(0x2e) << 0)
	/* CQ Notification */
	#define CMPL_BASE_TYPE_CQ_NOTIFICATION			(UINT32_C(0x30) << 0)
	/* SRQ Threshold Event */
	#define CMPL_BASE_TYPE_SRQ_EVENT			(UINT32_C(0x32) << 0)
	/* DBQ Threshold Event */
	#define CMPL_BASE_TYPE_DBQ_EVENT			(UINT32_C(0x34) << 0)
	/* QP Async Notification */
	#define CMPL_BASE_TYPE_QP_EVENT			(UINT32_C(0x38) << 0)
	/* Function Async Notification */
	#define CMPL_BASE_TYPE_FUNC_EVENT			(UINT32_C(0x3a) << 0)
	/* unused is 10 b */
	uint16_t info1;
	/* info1 is 16 b */
	uint32_t info2;
	/* info2 is 32 b */
	uint32_t info3_v;
	/* info3 is 31 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define CMPL_BASE_V					UINT32_C(0x1)
	/* info3 is 31 b */
	#define CMPL_BASE_INFO3_MASK				UINT32_C(0xfffffffe)
	#define CMPL_BASE_INFO3_SFT				1
	uint32_t info4;
	/* info4 is 32 b */
} __attribute__((packed));

/* TX Completion Record (16 bytes) */

struct tx_cmpl {
	uint16_t flags_type;
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define TX_CMPL_TYPE_MASK				UINT32_C(0x3f)
	#define TX_CMPL_TYPE_SFT				0
	/* TX L2 completion: Completion of TX packet. Length = 16B */
	#define TX_CMPL_TYPE_TX_L2				(UINT32_C(0x0) << 0)
	/*
	 * When this bit is '1', it indicates a packet that has an error of some
	 * type. Type of error is indicated in error_flags.
	 */
	#define TX_CMPL_FLAGS_ERROR				UINT32_C(0x40)
	/*
	 * When this bit is '1', it indicates that the packet completed was
	 * transmitted using the push acceleration data provided by the driver.
	 * When this bit is '0', it indicates that the packet had not push
	 * acceleration data written or was executed as a normal packet even
	 * though push data was provided.
	 */
	#define TX_CMPL_FLAGS_PUSH				UINT32_C(0x80)
	#define TX_CMPL_FLAGS_MASK				UINT32_C(0xffc0)
	#define TX_CMPL_FLAGS_SFT				6
	uint16_t unused_0;
	/* unused1 is 16 b */
	uint32_t opaque;
	/*
	 * This is a copy of the opaque field from the first TX BD of this
	 * transmitted packet.
	 */
	uint16_t errors_v;
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define TX_CMPL_V					UINT32_C(0x1)
	/*
	 * This error indicates that there was some sort of problem with the BDs
	 * for the packet.
	 */
	#define TX_CMPL_ERRORS_BUFFER_ERROR_MASK		UINT32_C(0xe)
	#define TX_CMPL_ERRORS_BUFFER_ERROR_SFT			1
	/* No error */
	#define TX_CMPL_ERRORS_BUFFER_ERROR_NO_ERROR		(UINT32_C(0x0) << 1)
	/* Bad Format: BDs were not formatted correctly. */
	#define TX_CMPL_ERRORS_BUFFER_ERROR_BAD_FMT		(UINT32_C(0x2) << 1)
	#define TX_CMPL_ERRORS_BUFFER_ERROR_LAST	TX_CMPL_ERRORS_BUFFER_ERROR_BAD_FMT
	/*
	 * When this bit is '1', it indicates that the length of the packet was
	 * zero. No packet was transmitted.
	 */
	#define TX_CMPL_ERRORS_ZERO_LENGTH_PKT			UINT32_C(0x10)
	/*
	 * When this bit is '1', it indicates that the packet was longer than
	 * the programmed limit in TDI. No packet was transmitted.
	 */
	#define TX_CMPL_ERRORS_EXCESSIVE_BD_LENGTH		UINT32_C(0x20)
	/*
	 * When this bit is '1', it indicates that one or more of the BDs
	 * associated with this packet generated a PCI error. This probably
	 * means the address was not valid.
	 */
	#define TX_CMPL_ERRORS_DMA_ERROR			UINT32_C(0x40)
	/*
	 * When this bit is '1', it indicates that the packet was longer than
	 * indicated by the hint. No packet was transmitted.
	 */
	#define TX_CMPL_ERRORS_HINT_TOO_SHORT			UINT32_C(0x80)
	/*
	 * When this bit is '1', it indicates that the packet was dropped due to
	 * Poison TLP error on one or more of the TLPs in the PXP completion.
	 */
	#define TX_CMPL_ERRORS_POISON_TLP_ERROR			UINT32_C(0x100)
	#define TX_CMPL_ERRORS_MASK				UINT32_C(0xfffe)
	#define TX_CMPL_ERRORS_SFT				1
	uint16_t unused_1;
	/* unused2 is 16 b */
	uint32_t unused_2;
	/* unused3 is 32 b */
} __attribute__((packed));

/* RX Packet Completion Record (32 bytes split to 2 16-byte struct) */

struct rx_pkt_cmpl {
	uint16_t flags_type;
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define RX_PKT_CMPL_TYPE_MASK				UINT32_C(0x3f)
	#define RX_PKT_CMPL_TYPE_SFT				0
	/* RX L2 completion: Completion of and L2 RX packet. Length = 32B */
	#define RX_PKT_CMPL_TYPE_RX_L2				(UINT32_C(0x11) << 0)
	/*
	 * When this bit is '1', it indicates a packet that has an error of some
	 * type. Type of error is indicated in error_flags.
	 */
	#define RX_PKT_CMPL_FLAGS_ERROR				UINT32_C(0x40)
	/* This field indicates how the packet was placed in the buffer. */
	#define RX_PKT_CMPL_FLAGS_PLACEMENT_MASK		UINT32_C(0x380)
	#define RX_PKT_CMPL_FLAGS_PLACEMENT_SFT			7
	/* Normal: Packet was placed using normal algorithm. */
	#define RX_PKT_CMPL_FLAGS_PLACEMENT_NORMAL		(UINT32_C(0x0) << 7)
	/* Jumbo: Packet was placed using jumbo algorithm. */
	#define RX_PKT_CMPL_FLAGS_PLACEMENT_JUMBO		(UINT32_C(0x1) << 7)
	/*
	 * Header/Data Separation: Packet was placed using Header/Data
	 * separation algorithm. The separation location is indicated by
	 * the itype field.
	 */
	#define RX_PKT_CMPL_FLAGS_PLACEMENT_HDS		(UINT32_C(0x2) << 7)
	#define RX_PKT_CMPL_FLAGS_PLACEMENT_LAST	RX_PKT_CMPL_FLAGS_PLACEMENT_HDS
	/* This bit is '1' if the RSS field in this completion is valid. */
	#define RX_PKT_CMPL_FLAGS_RSS_VALID			UINT32_C(0x400)
	/* unused is 1 b */
	/*
	 * This value indicates what the inner packet determined for the packet
	 * was.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_MASK			UINT32_C(0xf000)
	#define RX_PKT_CMPL_FLAGS_ITYPE_SFT			12
	/* Not Known: Indicates that the packet type was not known. */
	#define RX_PKT_CMPL_FLAGS_ITYPE_NOT_KNOWN		(UINT32_C(0x0) << 12)
	/*
	 * IP Packet: Indicates that the packet was an IP packet, but
	 * further classification was not possible.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_IP			(UINT32_C(0x1) << 12)
	/*
	 * TCP Packet: Indicates that the packet was IP and TCP. This
	 * indicates that the payload_offset field is valid.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_TCP			(UINT32_C(0x2) << 12)
	/*
	 * UDP Packet: Indicates that the packet was IP and UDP. This
	 * indicates that the payload_offset field is valid.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_UDP			(UINT32_C(0x3) << 12)
	/*
	 * FCoE Packet: Indicates that the packet was recognized as a
	 * FCoE. This also indicates that the payload_offset field is
	 * valid.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_FCOE			(UINT32_C(0x4) << 12)
	/*
	 * RoCE Packet: Indicates that the packet was recognized as a
	 * RoCE. This also indicates that the payload_offset field is
	 * valid.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_ROCE			(UINT32_C(0x5) << 12)
	/*
	 * ICMP Packet: Indicates that the packet was recognized as
	 * ICMP. This indicates that the payload_offset field is valid.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_ICMP			(UINT32_C(0x7) << 12)
	/*
	 * PtP packet wo/timestamp: Indicates that the packet was
	 * recognized as a PtP packet.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_PTP_WO_TIMESTAMP	(UINT32_C(0x8) << 12)
	/*
	 * PtP packet w/timestamp: Indicates that the packet was
	 * recognized as a PtP packet and that a timestamp was taken for
	 * the packet.
	 */
	#define RX_PKT_CMPL_FLAGS_ITYPE_PTP_W_TIMESTAMP	(UINT32_C(0x9) << 12)
	#define RX_PKT_CMPL_FLAGS_ITYPE_LAST	RX_PKT_CMPL_FLAGS_ITYPE_PTP_W_TIMESTAMP
	#define RX_PKT_CMPL_FLAGS_MASK				UINT32_C(0xffc0)
	#define RX_PKT_CMPL_FLAGS_SFT				6
	uint16_t len;
	/*
	 * This is the length of the data for the packet stored in the buffer(s)
	 * identified by the opaque value. This includes the packet BD and any
	 * associated buffer BDs. This does not include the the length of any
	 * data places in aggregation BDs.
	 */
	uint32_t opaque;
	/*
	 * This is a copy of the opaque field from the RX BD this completion
	 * corresponds to.
	 */
	uint8_t agg_bufs_v1;
	/* unused1 is 2 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_PKT_CMPL_V1					UINT32_C(0x1)
	/*
	 * This value is the number of aggregation buffers that follow this
	 * entry in the completion ring that are a part of this packet. If the
	 * value is zero, then the packet is completely contained in the buffer
	 * space provided for the packet in the RX ring.
	 */
	#define RX_PKT_CMPL_AGG_BUFS_MASK			UINT32_C(0x3e)
	#define RX_PKT_CMPL_AGG_BUFS_SFT			1
	/* unused1 is 2 b */
	uint8_t rss_hash_type;
	/*
	 * This is the RSS hash type for the packet. The value is packed
	 * {tuple_extrac_op[1:0],rss_profile_id[4:0],tuple_extrac_op[2]}.
	 */
	uint8_t payload_offset;
	/*
	 * This value indicates the offset in bytes from the beginning of the
	 * packet where the inner payload starts. This value is valid for TCP,
	 * UDP, FCoE, and RoCE packets. A value of zero indicates that header is
	 * 256B into the packet.
	 */
	uint8_t unused_1;
	/* unused2 is 8 b */
	uint32_t rss_hash;
	/*
	 * This value is the RSS hash value calculated for the packet based on
	 * the mode bits and key value in the VNIC.
	 */
} __attribute__((packed));

/* last 16 bytes of RX Packet Completion Record */

struct rx_pkt_cmpl_hi {
	uint32_t flags2;
	/*
	 * This indicates that the ip checksum was calculated for the inner
	 * packet and that the ip_cs_error field indicates if there was an
	 * error.
	 */
	#define RX_PKT_CMPL_FLAGS2_IP_CS_CALC			UINT32_C(0x1)
	/*
	 * This indicates that the TCP, UDP or ICMP checksum was calculated for
	 * the inner packet and that the l4_cs_error field indicates if there
	 * was an error.
	 */
	#define RX_PKT_CMPL_FLAGS2_L4_CS_CALC			UINT32_C(0x2)
	/*
	 * This indicates that the ip checksum was calculated for the tunnel
	 * header and that the t_ip_cs_error field indicates if there was an
	 * error.
	 */
	#define RX_PKT_CMPL_FLAGS2_T_IP_CS_CALC			UINT32_C(0x4)
	/*
	 * This indicates that the UDP checksum was calculated for the tunnel
	 * packet and that the t_l4_cs_error field indicates if there was an
	 * error.
	 */
	#define RX_PKT_CMPL_FLAGS2_T_L4_CS_CALC			UINT32_C(0x8)
	/* This value indicates what format the metadata field is. */
	#define RX_PKT_CMPL_FLAGS2_META_FORMAT_MASK		UINT32_C(0xf0)
	#define RX_PKT_CMPL_FLAGS2_META_FORMAT_SFT		4
	/* No metadata informtaion. Value is zero. */
	#define RX_PKT_CMPL_FLAGS2_META_FORMAT_NONE		(UINT32_C(0x0) << 4)
	/*
	 * The metadata field contains the VLAN tag and TPID value. -
	 * metadata[11:0] contains the vlan VID value. - metadata[12]
	 * contains the vlan DE value. - metadata[15:13] contains the
	 * vlan PRI value. - metadata[31:16] contains the vlan TPID
	 * value.
	 */
	#define RX_PKT_CMPL_FLAGS2_META_FORMAT_VLAN		(UINT32_C(0x1) << 4)
	#define RX_PKT_CMPL_FLAGS2_META_FORMAT_LAST	RX_PKT_CMPL_FLAGS2_META_FORMAT_VLAN
	/*
	 * This field indicates the IP type for the inner-most IP header. A
	 * value of '0' indicates IPv4. A value of '1' indicates IPv6. This
	 * value is only valid if itype indicates a packet with an IP header.
	 */
	#define RX_PKT_CMPL_FLAGS2_IP_TYPE			UINT32_C(0x100)
	uint32_t metadata;
	/*
	 * This is data from the CFA block as indicated by the meta_format
	 * field.
	 */
	/* When meta_format=1, this value is the VLAN VID. */
	#define RX_PKT_CMPL_METADATA_VID_MASK			UINT32_C(0xfff)
	#define RX_PKT_CMPL_METADATA_VID_SFT			0
	/* When meta_format=1, this value is the VLAN DE. */
	#define RX_PKT_CMPL_METADATA_DE				UINT32_C(0x1000)
	/* When meta_format=1, this value is the VLAN PRI. */
	#define RX_PKT_CMPL_METADATA_PRI_MASK			UINT32_C(0xe000)
	#define RX_PKT_CMPL_METADATA_PRI_SFT			13
	/* When meta_format=1, this value is the VLAN TPID. */
	#define RX_PKT_CMPL_METADATA_TPID_MASK			UINT32_C(0xffff0000)
	#define RX_PKT_CMPL_METADATA_TPID_SFT			16
	uint16_t errors_v2;
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_PKT_CMPL_V2					UINT32_C(0x1)
	/*
	 * This error indicates that there was some sort of problem with the BDs
	 * for the packet that was found after part of the packet was already
	 * placed. The packet should be treated as invalid.
	 */
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_MASK		UINT32_C(0xe)
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_SFT		1
	/* No buffer error */
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_NO_BUFFER	(UINT32_C(0x0) << 1)
	/*
	 * Did Not Fit: Packet did not fit into packet buffer provided.
	 * For regular placement, this means the packet did not fit in
	 * the buffer provided. For HDS and jumbo placement, this means
	 * that the packet could not be placed into 7 physical buffers
	 * or less.
	 */
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_DID_NOT_FIT	(UINT32_C(0x1) << 1)
	/*
	 * Not On Chip: All BDs needed for the packet were not on-chip
	 * when the packet arrived.
	 */
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_NOT_ON_CHIP	(UINT32_C(0x2) << 1)
	/* Bad Format: BDs were not formatted correctly. */
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_BAD_FORMAT	(UINT32_C(0x3) << 1)
	#define RX_PKT_CMPL_ERRORS_BUFFER_ERROR_LAST	RX_PKT_CMPL_ERRORS_BUFFER_ERROR_BAD_FORMAT
	/* This indicates that there was an error in the IP header checksum. */
	#define RX_PKT_CMPL_ERRORS_IP_CS_ERROR			UINT32_C(0x10)
	/*
	 * This indicates that there was an error in the TCP, UDP or ICMP
	 * checksum.
	 */
	#define RX_PKT_CMPL_ERRORS_L4_CS_ERROR			UINT32_C(0x20)
	/*
	 * This indicates that there was an error in the tunnel IP header
	 * checksum.
	 */
	#define RX_PKT_CMPL_ERRORS_T_IP_CS_ERROR		UINT32_C(0x40)
	/* This indicates that there was an error in the tunnel UDP checksum. */
	#define RX_PKT_CMPL_ERRORS_T_L4_CS_ERROR		UINT32_C(0x80)
	/*
	 * This indicates that there was a CRC error on either an FCoE or RoCE
	 * packet. The itype indicates the packet type.
	 */
	#define RX_PKT_CMPL_ERRORS_CRC_ERROR			UINT32_C(0x100)
	/*
	 * This indicates that there was an error in the tunnel portion of the
	 * packet when this field is non-zero.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_MASK		UINT32_C(0xe00)
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_SFT		9
	/*
	 * No additional error occurred on the tunnel portion of the
	 * packet of the packet does not have a tunnel.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_NO_ERROR	(UINT32_C(0x0) << 9)
	/*
	 * Indicates that IP header version does not match expectation
	 * from L2 Ethertype for IPv4 and IPv6 in the tunnel header.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_T_L3_BAD_VERSION   (UINT32_C(0x1) << 9)
	/*
	 * Indicates that header length is out of range in the tunnel
	 * header. Valid for IPv4.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_T_L3_BAD_HDR_LEN   (UINT32_C(0x2) << 9)
	/*
	 * Indicates that the physical packet is shorter than that
	 * claimed by the PPPoE header length for a tunnel PPPoE packet.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_TUNNEL_TOTAL_ERROR (UINT32_C(0x3) << 9)
	/*
	 * Indicates that physical packet is shorter than that claimed
	 * by the tunnel l3 header length. Valid for IPv4, or IPv6
	 * tunnel packet packets.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_T_IP_TOTAL_ERROR   (UINT32_C(0x4) << 9)
	/*
	 * Indicates that the physical packet is shorter than that
	 * claimed by the tunnel UDP header length for a tunnel UDP
	 * packet that is not fragmented.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_T_UDP_TOTAL_ERROR  (UINT32_C(0x5) << 9)
	/*
	 * indicates that the IPv4 TTL or IPv6 hop limit check have
	 * failed (e.g. TTL = 0) in the tunnel header. Valid for IPv4,
	 * and IPv6.
	 */
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_T_L3_BAD_TTL	(UINT32_C(0x6) << 9)
	#define RX_PKT_CMPL_ERRORS_T_PKT_ERROR_LAST	RX_PKT_CMPL_ERRORS_T_PKT_ERROR_T_L3_BAD_TTL
	/*
	 * This indicates that there was an error in the inner portion of the
	 * packet when this field is non-zero.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_MASK		UINT32_C(0xf000)
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_SFT		12
	/*
	 * No additional error occurred on the tunnel portion of the
	 * packet of the packet does not have a tunnel.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_NO_ERROR		(UINT32_C(0x0) << 12)
	/*
	 * Indicates that IP header version does not match expectation
	 * from L2 Ethertype for IPv4 and IPv6 or that option other than
	 * VFT was parsed on FCoE packet.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_L3_BAD_VERSION	(UINT32_C(0x1) << 12)
	/*
	 * indicates that header length is out of range. Valid for IPv4
	 * and RoCE
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_L3_BAD_HDR_LEN	(UINT32_C(0x2) << 12)
	/*
	 * indicates that the IPv4 TTL or IPv6 hop limit check have
	 * failed (e.g. TTL = 0). Valid for IPv4, and IPv6
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_L3_BAD_TTL	(UINT32_C(0x3) << 12)
	/*
	 * Indicates that physical packet is shorter than that claimed
	 * by the l3 header length. Valid for IPv4, IPv6 packet or RoCE
	 * packets.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_IP_TOTAL_ERROR	(UINT32_C(0x4) << 12)
	/*
	 * Indicates that the physical packet is shorter than that
	 * claimed by the UDP header length for a UDP packet that is not
	 * fragmented.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_UDP_TOTAL_ERROR	(UINT32_C(0x5) << 12)
	/*
	 * Indicates that TCP header length > IP payload. Valid for TCP
	 * packets only.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_L4_BAD_HDR_LEN	(UINT32_C(0x6) << 12)
	/* Indicates that TCP header length < 5. Valid for TCP. */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_L4_BAD_HDR_LEN_TOO_SMALL (UINT32_C(0x7) << 12)
	/*
	 * Indicates that TCP option headers result in a TCP header size
	 * that does not match data offset in TCP header. Valid for TCP.
	 */
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_L4_BAD_OPT_LEN	(UINT32_C(0x8) << 12)
	#define RX_PKT_CMPL_ERRORS_PKT_ERROR_LAST	RX_PKT_CMPL_ERRORS_PKT_ERROR_L4_BAD_OPT_LEN
	#define RX_PKT_CMPL_ERRORS_MASK				UINT32_C(0xfffe)
	#define RX_PKT_CMPL_ERRORS_SFT				1
	uint16_t cfa_code;
	/*
	 * This field identifies the CFA action rule that was used for this
	 * packet.
	 */
	uint32_t reorder;
	/*
	 * This value holds the reordering sequence number for the packet. If
	 * the reordering sequence is not valid, then this value is zero. The
	 * reordering domain for the packet is in the bottom 8 to 10b of the
	 * rss_hash value. The bottom 20b of this value contain the ordering
	 * domain value for the packet.
	 */
	#define RX_PKT_CMPL_REORDER_MASK			UINT32_C(0xffffff)
	#define RX_PKT_CMPL_REORDER_SFT				0
} __attribute__((packed));

/* RX L2 TPA Start Completion Record (32 bytes split to 2 16-byte struct) */

struct rx_tpa_start_cmpl {
	uint16_t flags_type;
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define RX_TPA_START_CMPL_TYPE_MASK			UINT32_C(0x3f)
	#define RX_TPA_START_CMPL_TYPE_SFT			0
	/*
	 * RX L2 TPA Start Completion: Completion at the beginning of a
	 * TPA operation. Length = 32B
	 */
	#define RX_TPA_START_CMPL_TYPE_RX_TPA_START		(UINT32_C(0x13) << 0)
	/* This bit will always be '0' for TPA start completions. */
	#define RX_TPA_START_CMPL_FLAGS_ERROR			UINT32_C(0x40)
	/* This field indicates how the packet was placed in the buffer. */
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_MASK		UINT32_C(0x380)
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_SFT		7
	/*
	 * Jumbo: TPA Packet was placed using jumbo algorithm. This
	 * means that the first buffer will be filled with data before
	 * moving to aggregation buffers. Each aggregation buffer will
	 * be filled before moving to the next aggregation buffer.
	 */
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_JUMBO	(UINT32_C(0x1) << 7)
	/*
	 * Header/Data Separation: Packet was placed using Header/Data
	 * separation algorithm. The separation location is indicated by
	 * the itype field.
	 */
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_HDS		(UINT32_C(0x2) << 7)
	/*
	 * GRO/Jumbo: Packet will be placed using GRO/Jumbo where the
	 * first packet is filled with data. Subsequent packets will be
	 * placed such that any one packet does not span two aggregation
	 * buffers unless it starts at the beginning of an aggregation
	 * buffer.
	 */
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_GRO_JUMBO	(UINT32_C(0x5) << 7)
	/*
	 * GRO/Header-Data Separation: Packet will be placed using
	 * GRO/HDS where the header is in the first packet. Payload of
	 * each packet will be placed such that any one packet does not
	 * span two aggregation buffers unless it starts at the
	 * beginning of an aggregation buffer.
	 */
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_GRO_HDS	(UINT32_C(0x6) << 7)
	#define RX_TPA_START_CMPL_FLAGS_PLACEMENT_LAST	RX_TPA_START_CMPL_FLAGS_PLACEMENT_GRO_HDS
	/* This bit is '1' if the RSS field in this completion is valid. */
	#define RX_TPA_START_CMPL_FLAGS_RSS_VALID		UINT32_C(0x400)
	/* unused is 1 b */
	/*
	 * This value indicates what the inner packet determined for the packet
	 * was.
	 */
	#define RX_TPA_START_CMPL_FLAGS_ITYPE_MASK		UINT32_C(0xf000)
	#define RX_TPA_START_CMPL_FLAGS_ITYPE_SFT		12
	/* TCP Packet: Indicates that the packet was IP and TCP. */
	#define RX_TPA_START_CMPL_FLAGS_ITYPE_TCP		(UINT32_C(0x2) << 12)
	#define RX_TPA_START_CMPL_FLAGS_ITYPE_LAST	RX_TPA_START_CMPL_FLAGS_ITYPE_TCP
	#define RX_TPA_START_CMPL_FLAGS_MASK			UINT32_C(0xffc0)
	#define RX_TPA_START_CMPL_FLAGS_SFT			6
	uint16_t len;
	/*
	 * This value indicates the amount of packet data written to the buffer
	 * the opaque field in this completion corresponds to.
	 */
	uint32_t opaque;
	/*
	 * This is a copy of the opaque field from the RX BD this completion
	 * corresponds to.
	 */
	uint8_t v1;
	/* unused1 is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_TPA_START_CMPL_V1				UINT32_C(0x1)
	/* unused1 is 7 b */
	uint8_t rss_hash_type;
	/*
	 * This is the RSS hash type for the packet. The value is packed
	 * {tuple_extrac_op[1:0],rss_profile_id[4:0],tuple_extrac_op[2]}.
	 */
	uint16_t agg_id;
	/*
	 * This is the aggregation ID that the completion is associated with.
	 * Use this number to correlate the TPA start completion with the TPA
	 * end completion.
	 */
	/* unused2 is 9 b */
	/*
	 * This is the aggregation ID that the completion is associated with.
	 * Use this number to correlate the TPA start completion with the TPA
	 * end completion.
	 */
	#define RX_TPA_START_CMPL_AGG_ID_MASK			UINT32_C(0xfe00)
	#define RX_TPA_START_CMPL_AGG_ID_SFT			9
	uint32_t rss_hash;
	/*
	 * This value is the RSS hash value calculated for the packet based on
	 * the mode bits and key value in the VNIC.
	 */
} __attribute__((packed));

/* last 16 bytes of RX L2 TPA Start Completion Record */

struct rx_tpa_start_cmpl_hi {
	uint32_t flags2;
	/*
	 * This indicates that the ip checksum was calculated for the inner
	 * packet and that the sum passed for all segments included in the
	 * aggregation.
	 */
	#define RX_TPA_START_CMPL_FLAGS2_IP_CS_CALC		UINT32_C(0x1)
	/*
	 * This indicates that the TCP, UDP or ICMP checksum was calculated for
	 * the inner packet and that the sum passed for all segments included in
	 * the aggregation.
	 */
	#define RX_TPA_START_CMPL_FLAGS2_L4_CS_CALC		UINT32_C(0x2)
	/*
	 * This indicates that the ip checksum was calculated for the tunnel
	 * header and that the sum passed for all segments included in the
	 * aggregation.
	 */
	#define RX_TPA_START_CMPL_FLAGS2_T_IP_CS_CALC		UINT32_C(0x4)
	/*
	 * This indicates that the UDP checksum was calculated for the tunnel
	 * packet and that the sum passed for all segments included in the
	 * aggregation.
	 */
	#define RX_TPA_START_CMPL_FLAGS2_T_L4_CS_CALC		UINT32_C(0x8)
	/* This value indicates what format the metadata field is. */
	#define RX_TPA_START_CMPL_FLAGS2_META_FORMAT_MASK	UINT32_C(0xf0)
	#define RX_TPA_START_CMPL_FLAGS2_META_FORMAT_SFT	4
	/* No metadata informtaion. Value is zero. */
	#define RX_TPA_START_CMPL_FLAGS2_META_FORMAT_NONE	(UINT32_C(0x0) << 4)
	/*
	 * The metadata field contains the VLAN tag and TPID value. -
	 * metadata[11:0] contains the vlan VID value. - metadata[12]
	 * contains the vlan DE value. - metadata[15:13] contains the
	 * vlan PRI value. - metadata[31:16] contains the vlan TPID
	 * value.
	 */
	#define RX_TPA_START_CMPL_FLAGS2_META_FORMAT_VLAN	(UINT32_C(0x1) << 4)
	#define RX_TPA_START_CMPL_FLAGS2_META_FORMAT_LAST	RX_TPA_START_CMPL_FLAGS2_META_FORMAT_VLAN
	/*
	 * This field indicates the IP type for the inner-most IP header. A
	 * value of '0' indicates IPv4. A value of '1' indicates IPv6.
	 */
	#define RX_TPA_START_CMPL_FLAGS2_IP_TYPE		UINT32_C(0x100)
	uint32_t metadata;
	/*
	 * This is data from the CFA block as indicated by the meta_format
	 * field.
	 */
	/* When meta_format=1, this value is the VLAN VID. */
	#define RX_TPA_START_CMPL_METADATA_VID_MASK		UINT32_C(0xfff)
	#define RX_TPA_START_CMPL_METADATA_VID_SFT		0
	/* When meta_format=1, this value is the VLAN DE. */
	#define RX_TPA_START_CMPL_METADATA_DE			UINT32_C(0x1000)
	/* When meta_format=1, this value is the VLAN PRI. */
	#define RX_TPA_START_CMPL_METADATA_PRI_MASK		UINT32_C(0xe000)
	#define RX_TPA_START_CMPL_METADATA_PRI_SFT		13
	/* When meta_format=1, this value is the VLAN TPID. */
	#define RX_TPA_START_CMPL_METADATA_TPID_MASK		UINT32_C(0xffff0000)
	#define RX_TPA_START_CMPL_METADATA_TPID_SFT		16
	uint16_t v2;
	/* unused4 is 15 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_TPA_START_CMPL_V2				UINT32_C(0x1)
	/* unused4 is 15 b */
	uint16_t cfa_code;
	/*
	 * This field identifies the CFA action rule that was used for this
	 * packet.
	 */
	uint32_t inner_l4_size_inner_l3_offset_inner_l2_offset_outer_l3_offset;
	/*
	 * This is the size in bytes of the inner most L4 header. This can be
	 * subtracted from the payload_offset to determine the start of the
	 * inner most L4 header.
	 */
	/*
	 * This is the offset from the beginning of the packet in bytes for the
	 * outer L3 header. If there is no outer L3 header, then this value is
	 * zero.
	 */
	#define RX_TPA_START_CMPL_OUTER_L3_OFFSET_MASK		UINT32_C(0x1ff)
	#define RX_TPA_START_CMPL_OUTER_L3_OFFSET_SFT		0
	/*
	 * This is the offset from the beginning of the packet in bytes for the
	 * inner most L2 header.
	 */
	#define RX_TPA_START_CMPL_INNER_L2_OFFSET_MASK		UINT32_C(0x3fe00)
	#define RX_TPA_START_CMPL_INNER_L2_OFFSET_SFT		9
	/*
	 * This is the offset from the beginning of the packet in bytes for the
	 * inner most L3 header.
	 */
	#define RX_TPA_START_CMPL_INNER_L3_OFFSET_MASK		UINT32_C(0x7fc0000)
	#define RX_TPA_START_CMPL_INNER_L3_OFFSET_SFT		18
	/*
	 * This is the size in bytes of the inner most L4 header. This can be
	 * subtracted from the payload_offset to determine the start of the
	 * inner most L4 header.
	 */
	#define RX_TPA_START_CMPL_INNER_L4_SIZE_MASK		UINT32_C(0xf8000000)
	#define RX_TPA_START_CMPL_INNER_L4_SIZE_SFT		27
} __attribute__((packed));

/* RX TPA End Completion Record (32 bytes split to 2 16-byte struct) */

struct rx_tpa_end_cmpl {
	uint16_t flags_type;
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define RX_TPA_END_CMPL_TYPE_MASK			UINT32_C(0x3f)
	#define RX_TPA_END_CMPL_TYPE_SFT			0
	/*
	 * RX L2 TPA End Completion: Completion at the end of a TPA
	 * operation. Length = 32B
	 */
	#define RX_TPA_END_CMPL_TYPE_RX_TPA_END		(UINT32_C(0x15) << 0)
	/*
	 * When this bit is '1', it indicates a packet that has an error of some
	 * type. Type of error is indicated in error_flags.
	 */
	#define RX_TPA_END_CMPL_FLAGS_ERROR			UINT32_C(0x40)
	/* This field indicates how the packet was placed in the buffer. */
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_MASK		UINT32_C(0x380)
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_SFT		7
	/*
	 * Jumbo: TPA Packet was placed using jumbo algorithm. This
	 * means that the first buffer will be filled with data before
	 * moving to aggregation buffers. Each aggregation buffer will
	 * be filled before moving to the next aggregation buffer.
	 */
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_JUMBO		(UINT32_C(0x1) << 7)
	/*
	 * Header/Data Separation: Packet was placed using Header/Data
	 * separation algorithm. The separation location is indicated by
	 * the itype field.
	 */
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_HDS		(UINT32_C(0x2) << 7)
	/*
	 * GRO/Jumbo: Packet will be placed using GRO/Jumbo where the
	 * first packet is filled with data. Subsequent packets will be
	 * placed such that any one packet does not span two aggregation
	 * buffers unless it starts at the beginning of an aggregation
	 * buffer.
	 */
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_GRO_JUMBO	(UINT32_C(0x5) << 7)
	/*
	 * GRO/Header-Data Separation: Packet will be placed using
	 * GRO/HDS where the header is in the first packet. Payload of
	 * each packet will be placed such that any one packet does not
	 * span two aggregation buffers unless it starts at the
	 * beginning of an aggregation buffer.
	 */
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_GRO_HDS	(UINT32_C(0x6) << 7)
	#define RX_TPA_END_CMPL_FLAGS_PLACEMENT_LAST	RX_TPA_END_CMPL_FLAGS_PLACEMENT_GRO_HDS
	/* unused is 2 b */
	/*
	 * This value indicates what the inner packet determined for the packet
	 * was. - 2 TCP Packet Indicates that the packet was IP and TCP. This
	 * indicates that the ip_cs field is valid and that the tcp_udp_cs field
	 * is valid and contains the TCP checksum. This also indicates that the
	 * payload_offset field is valid.
	 */
	#define RX_TPA_END_CMPL_FLAGS_ITYPE_MASK		UINT32_C(0xf000)
	#define RX_TPA_END_CMPL_FLAGS_ITYPE_SFT			12
	#define RX_TPA_END_CMPL_FLAGS_MASK			UINT32_C(0xffc0)
	#define RX_TPA_END_CMPL_FLAGS_SFT			6
	uint16_t len;
	/*
	 * This value is zero for TPA End completions. There is no data in the
	 * buffer that corresponds to the opaque value in this completion.
	 */
	uint32_t opaque;
	/*
	 * This is a copy of the opaque field from the RX BD this completion
	 * corresponds to.
	 */
	uint8_t agg_bufs_v1;
	/* unused1 is 1 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_TPA_END_CMPL_V1				UINT32_C(0x1)
	/*
	 * This value is the number of aggregation buffers that follow this
	 * entry in the completion ring that are a part of this aggregation
	 * packet. If the value is zero, then the packet is completely contained
	 * in the buffer space provided in the aggregation start completion.
	 */
	#define RX_TPA_END_CMPL_AGG_BUFS_MASK			UINT32_C(0x7e)
	#define RX_TPA_END_CMPL_AGG_BUFS_SFT			1
	/* unused1 is 1 b */
	uint8_t tpa_segs;
	/* This value is the number of segments in the TPA operation. */
	uint8_t payload_offset;
	/*
	 * This value indicates the offset in bytes from the beginning of the
	 * packet where the inner payload starts. This value is valid for TCP,
	 * UDP, FCoE, and RoCE packets. A value of zero indicates an offset of
	 * 256 bytes.
	 */
	uint8_t agg_id;
	/*
	 * This is the aggregation ID that the completion is associated with.
	 * Use this number to correlate the TPA start completion with the TPA
	 * end completion.
	 */
	/* unused2 is 1 b */
	/*
	 * This is the aggregation ID that the completion is associated with.
	 * Use this number to correlate the TPA start completion with the TPA
	 * end completion.
	 */
	#define RX_TPA_END_CMPL_AGG_ID_MASK			UINT32_C(0xfe)
	#define RX_TPA_END_CMPL_AGG_ID_SFT			1
	uint32_t tsdelta;
	/*
	 * For non-GRO packets, this value is the timestamp delta between
	 * earliest and latest timestamp values for TPA packet. If packets were
	 * not time stamped, then delta will be zero. For GRO packets, this
	 * field is zero except for the following sub-fields. - tsdelta[31]
	 * Timestamp present indication. When '0', no Timestamp option is in the
	 * packet. When '1', then a Timestamp option is present in the packet.
	 */
} __attribute__((packed));

/* last 16 bytes of RX TPA End Completion Record */

struct rx_tpa_end_cmpl_hi {
	uint32_t tpa_dup_acks;
	/* unused3 is 28 b */
	/*
	 * This value is the number of duplicate ACKs that have been received as
	 * part of the TPA operation.
	 */
	#define RX_TPA_END_CMPL_TPA_DUP_ACKS_MASK		UINT32_C(0xf)
	#define RX_TPA_END_CMPL_TPA_DUP_ACKS_SFT		0
	/* unused3 is 28 b */
	uint16_t tpa_seg_len;
	/*
	 * This value is the valid when TPA completion is active. It indicates
	 * the length of the longest segment of the TPA operation for LRO mode
	 * and the length of the first segment in GRO mode. This value may be
	 * used by GRO software to re-construct the original packet stream from
	 * the TPA packet. This is the length of all but the last segment for
	 * GRO. In LRO mode this value may be used to indicate MSS size to the
	 * stack.
	 */
	uint16_t unused_3;
	/* unused4 is 16 b */
	uint16_t errors_v2;
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_TPA_END_CMPL_V2				UINT32_C(0x1)
	/*
	 * This error indicates that there was some sort of problem with the BDs
	 * for the packet that was found after part of the packet was already
	 * placed. The packet should be treated as invalid.
	 */
	#define RX_TPA_END_CMPL_ERRORS_BUFFER_ERROR_MASK	UINT32_C(0xe)
	#define RX_TPA_END_CMPL_ERRORS_BUFFER_ERROR_SFT		1
	/*
	 * This error occurs when there is a fatal HW problem in the
	 * chip only. It indicates that there were not BDs on chip but
	 * that there was adequate reservation. provided by the TPA
	 * block.
	 */
	#define RX_TPA_END_CMPL_ERRORS_BUFFER_ERROR_NOT_ON_CHIP   (UINT32_C(0x2) << 1)
	/*
	 * This error occurs when TPA block was not configured to
	 * reserve adequate BDs for TPA operations on this RX ring. All
	 * data for the TPA operation was not placed. This error can
	 * also be generated when the number of segments is not
	 * programmed correctly in TPA and the 33 total aggregation
	 * buffers allowed for the TPA operation has been exceeded.
	 */
	#define RX_TPA_END_CMPL_ERRORS_BUFFER_ERROR_RSV_ERROR	(UINT32_C(0x4) << 1)
	#define RX_TPA_END_CMPL_ERRORS_BUFFER_ERROR_LAST	RX_TPA_END_CMPL_ERRORS_BUFFER_ERROR_RSV_ERROR
	#define RX_TPA_END_CMPL_ERRORS_MASK			UINT32_C(0xfffe)
	#define RX_TPA_END_CMPL_ERRORS_SFT			1
	uint16_t unused_4;
	/* unused5 is 16 b */
	uint32_t start_opaque;
	/*
	 * This is the opaque value that was completed for the TPA start
	 * completion that corresponds to this TPA end completion.
	 */
} __attribute__((packed));

/* RX Aggregation Buffer Completion Record (16 bytes) */

struct rx_abuf_cmpl {
	uint16_t type;
	/* unused is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define RX_ABUF_CMPL_TYPE_MASK				UINT32_C(0x3f)
	#define RX_ABUF_CMPL_TYPE_SFT				0
	/*
	 * RX Aggregation Buffer completion : Completion of an L2
	 * aggregation buffer in support of TPA, HDS, or Jumbo packet
	 * completion. Length = 16B
	 */
	#define RX_ABUF_CMPL_TYPE_RX_AGG			(UINT32_C(0x12) << 0)
	/* unused is 10 b */
	uint16_t len;
	/*
	 * This is the length of the data for the packet stored in this
	 * aggregation buffer identified by the opaque value. This does not
	 * include the length of any data placed in other aggregation BDs or in
	 * the packet or buffer BDs. This length does not include any space
	 * added due to hdr_offset register during HDS placement mode.
	 */
	uint32_t opaque;
	/*
	 * This is a copy of the opaque field from the RX BD this aggregation
	 * buffer corresponds to.
	 */
	uint32_t v;
	/* unused2 is 31 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define RX_ABUF_CMPL_V					UINT32_C(0x1)
	/* unused2 is 31 b */
	uint32_t unused_2;
	/* unused3 is 32 b */
} __attribute__((packed));

/* Statistics Ejection Buffer Completion Record (16 bytes) */

struct eject_cmpl {
	uint16_t type;
	/* unused is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define EJECT_CMPL_TYPE_MASK				UINT32_C(0x3f)
	#define EJECT_CMPL_TYPE_SFT				0
	/*
	 * Statistics Ejection Completion: Completion of statistics data
	 * ejection buffer. Length = 16B
	 */
	#define EJECT_CMPL_TYPE_STAT_EJECT			(UINT32_C(0x1a) << 0)
	/* unused is 10 b */
	uint16_t len;
	/* This is the length of the statistics data stored in this buffer. */
	uint32_t opaque;
	/*
	 * This is a copy of the opaque field from the RX BD this ejection
	 * buffer corresponds to.
	 */
	uint32_t v;
	/* unused2 is 31 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define EJECT_CMPL_V					UINT32_C(0x1)
	/* unused2 is 31 b */
	uint32_t unused_2;
	/* unused3 is 32 b */
} __attribute__((packed));

/* HWRM Completion Record (16 bytes) */

struct hwrm_cmpl {
	uint16_t type;
	/* unused is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_CMPL_TYPE_MASK				UINT32_C(0x3f)
	#define HWRM_CMPL_TYPE_SFT				0
	/* HWRM Command Completion: Completion of an HWRM command. */
	#define HWRM_CMPL_TYPE_HWRM_DONE			(UINT32_C(0x20) << 0)
	/* unused is 10 b */
	uint16_t sequence_id;
	/* This is the sequence_id of the HWRM command that has completed. */
	uint32_t unused_1;
	/* unused2 is 32 b */
	uint32_t v;
	/* unused3 is 31 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_CMPL_V					UINT32_C(0x1)
	/* unused3 is 31 b */
	uint32_t unused_3;
	/* unused4 is 32 b */
} __attribute__((packed));

/* HWRM Forwarded Request (16 bytes) */

struct hwrm_fwd_req_cmpl {
	uint16_t req_len_type;
	/* Length of forwarded request in bytes. */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_FWD_REQ_CMPL_TYPE_MASK			UINT32_C(0x3f)
	#define HWRM_FWD_REQ_CMPL_TYPE_SFT			0
	/* Forwarded HWRM Request */
	#define HWRM_FWD_REQ_CMPL_TYPE_HWRM_FWD_REQ		(UINT32_C(0x22) << 0)
	/* Length of forwarded request in bytes. */
	#define HWRM_FWD_REQ_CMPL_REQ_LEN_MASK			UINT32_C(0xffc0)
	#define HWRM_FWD_REQ_CMPL_REQ_LEN_SFT			6
	uint16_t source_id;
	/*
	 * Source ID of this request. Typically used in forwarding requests and
	 * responses. 0x0 - 0xFFF8 - Used for function ids 0xFFF8 - 0xFFFE -
	 * Reserved for internal processors 0xFFFF - HWRM
	 */
	uint32_t unused_0;
	/* unused1 is 32 b */
	uint64_t req_buf_addr_v;
	/* Address of forwarded request. */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_FWD_REQ_CMPL_V				UINT32_C(0x1)
	/* Address of forwarded request. */
	#define HWRM_FWD_REQ_CMPL_REQ_BUF_ADDR_MASK		UINT32_C(0xfffffffe)
	#define HWRM_FWD_REQ_CMPL_REQ_BUF_ADDR_SFT		1
} __attribute__((packed));

/* HWRM Forwarded Response (16 bytes) */

struct hwrm_fwd_resp_cmpl {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_FWD_RESP_CMPL_TYPE_MASK			UINT32_C(0x3f)
	#define HWRM_FWD_RESP_CMPL_TYPE_SFT			0
	/* Forwarded HWRM Response */
	#define HWRM_FWD_RESP_CMPL_TYPE_HWRM_FWD_RESP		(UINT32_C(0x24) << 0)
	/* unused1 is 10 b */
	uint16_t source_id;
	/*
	 * Source ID of this response. Typically used in forwarding requests and
	 * responses. 0x0 - 0xFFF8 - Used for function ids 0xFFF8 - 0xFFFE -
	 * Reserved for internal processors 0xFFFF - HWRM
	 */
	uint16_t resp_len;
	/* Length of forwarded response in bytes. */
	uint16_t unused_1;
	/* unused2 is 16 b */
	uint64_t resp_buf_addr_v;
	/* Address of forwarded response. */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_FWD_RESP_CMPL_V				UINT32_C(0x1)
	/* Address of forwarded response. */
	#define HWRM_FWD_RESP_CMPL_RESP_BUF_ADDR_MASK		UINT32_C(0xfffffffe)
	#define HWRM_FWD_RESP_CMPL_RESP_BUF_ADDR_SFT		1
} __attribute__((packed));

#define GET_EVENT_ID(x) \
	((x) == 0x30 ? "VF_FLR": \
	((x) == 0x20 ? "PF_DRVR_UNLOAD": \
	((x) == 0x10 ? "FUNC_DRVR_UNLOAD": \
	((x) == 0xff ? "HWRM_ERROR": \
	((x) == 0x32 ? "PF_VF_COMM_STATUS_CHANGE": \
	((x) == 0x33 ? "VF_CFG_CHANGE": \
	((x) == 0x11 ? "FUNC_DRVR_LOAD": \
	((x) == 0x31 ? "VF_MAC_ADDR_CHANGE": \
	((x) == 0x4 ? "PORT_CONN_NOT_ALLOWED": \
	((x) == 0x5 ? "LINK_SPEED_CFG_NOT_ALLOWED": \
	((x) == 0x6 ? "LINK_SPEED_CFG_CHANGE": \
	((x) == 0x7 ? "PORT_PHY_CFG_CHANGE": \
	((x) == 0x0 ? "LINK_STATUS_CHANGE": \
	((x) == 0x1 ? "LINK_MTU_CHANGE": \
	((x) == 0x2 ? "LINK_SPEED_CHANGE": \
	((x) == 0x3 ? "DCB_CONFIG_CHANGE": \
	((x) == 0x21 ? "PF_DRVR_LOAD": \
	"Unknown event_id")))))))))))))))))

/* HWRM Asynchronous Event Completion Record (16 bytes) */

struct hwrm_async_event_cmpl {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_TYPE_MASK			UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_TYPE_SFT			0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_TYPE_HWRM_ASYNC_EVENT	(UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Link status changed */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_STATUS_CHANGE (UINT32_C(0x0) << 0)
	/* Link MTU changed */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_MTU_CHANGE	(UINT32_C(0x1) << 0)
	/* Link speed changed */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CHANGE  (UINT32_C(0x2) << 0)
	/* DCB Configuration changed */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_DCB_CONFIG_CHANGE  (UINT32_C(0x3) << 0)
	/* Port connection not allowed */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PORT_CONN_NOT_ALLOWED (UINT32_C(0x4) << 0)
	/* Link speed configuration was not allowed */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CFG_NOT_ALLOWED (UINT32_C(0x5) << 0)
	/* Link speed configuration change */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_LINK_SPEED_CFG_CHANGE (UINT32_C(0x6) << 0)
	/* Port PHY configuration change */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PORT_PHY_CFG_CHANGE (UINT32_C(0x7) << 0)
	/* Function driver unloaded */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_FUNC_DRVR_UNLOAD   (UINT32_C(0x10) << 0)
	/* Function driver loaded */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_FUNC_DRVR_LOAD	(UINT32_C(0x11) << 0)
	/* PF driver unloaded */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PF_DRVR_UNLOAD	(UINT32_C(0x20) << 0)
	/* PF driver loaded */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PF_DRVR_LOAD	(UINT32_C(0x21) << 0)
	/* VF Function Level Reset (FLR) */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_VF_FLR		(UINT32_C(0x30) << 0)
	/* VF MAC Address Change */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_VF_MAC_ADDR_CHANGE (UINT32_C(0x31) << 0)
	/* PF-VF communication channel status change. */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_PF_VF_COMM_STATUS_CHANGE (UINT32_C(0x32) << 0)
	/* VF Configuration Change */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_VF_CFG_CHANGE	(UINT32_C(0x33) << 0)
	/* HWRM Error */
	#define HWRM_ASYNC_EVENT_CMPL_EVENT_ID_HWRM_ERROR	(UINT32_C(0xff) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_V				UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_OPAQUE_MASK		UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_OPAQUE_SFT		1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for link status change (16 bytes) */

struct hwrm_async_event_cmpl_link_status_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_TYPE_SFT  0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Link status changed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_ID_LINK_STATUS_CHANGE (UINT32_C(0x0) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* Indicates link status change */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_LINK_CHANGE UINT32_C(0x1)
	/*
	 * If this bit set to 0, then it indicates that the link was up
	 * and it went down.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_LINK_CHANGE_DOWN (UINT32_C(0x0) << 0)
	/*
	 * If this bit is set to 1, then it indicates that the link was
	 * down and it went up.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_LINK_CHANGE_UP (UINT32_C(0x1) << 0)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_LINK_CHANGE_LAST	HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_LINK_CHANGE_UP
	/* Indicates the physical port this link status change occur */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_PORT_MASK UINT32_C(0xe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_PORT_SFT 1
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff0)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_STATUS_CHANGE_EVENT_DATA1_PORT_ID_SFT 4
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for link MTU change (16 bytes) */

struct hwrm_async_event_cmpl_link_mtu_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_TYPE_MASK	UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Link MTU changed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_EVENT_ID_LINK_MTU_CHANGE (UINT32_C(0x1) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_V		UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_OPAQUE_MASK  UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_OPAQUE_SFT   1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* The new MTU of the link in bytes. */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_EVENT_DATA1_NEW_MTU_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_MTU_CHANGE_EVENT_DATA1_NEW_MTU_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for link speed change (16 bytes) */

struct hwrm_async_event_cmpl_link_speed_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_TYPE_MASK  UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_TYPE_SFT   0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Link speed changed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_ID_LINK_SPEED_CHANGE (UINT32_C(0x2) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/*
	 * When this bit is '1', the link was forced to the force_link_speed
	 * value.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_FORCE UINT32_C(0x1)
	/* The new link speed in 100 Mbps units. */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_MASK UINT32_C(0xfffe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_SFT 1
	/* 100Mb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_100MB (UINT32_C(0x1) << 1)
	/* 1Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_1GB (UINT32_C(0xa) << 1)
	/* 2Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_2GB (UINT32_C(0x14) << 1)
	/* 2.5Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_2_5GB (UINT32_C(0x19) << 1)
	/* 10Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_10GB (UINT32_C(0x64) << 1)
	/* 20Mb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_20GB (UINT32_C(0xc8) << 1)
	/* 25Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_25GB (UINT32_C(0xfa) << 1)
	/* 40Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_40GB (UINT32_C(0x190) << 1)
	/* 50Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_50GB (UINT32_C(0x1f4) << 1)
	/* 100Gb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_100GB (UINT32_C(0x3e8) << 1)
	/* 10Mb link speed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_10MB (UINT32_C(0xffff) << 1)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_LAST	HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_NEW_LINK_SPEED_100MBPS_10MB
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff0000)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CHANGE_EVENT_DATA1_PORT_ID_SFT 16
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for DCB Config change (16 bytes) */

struct hwrm_async_event_cmpl_dcb_config_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_TYPE_MASK  UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_TYPE_SFT   0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* DCB Configuration changed */
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_EVENT_ID_DCB_CONFIG_CHANGE (UINT32_C(0x3) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_DCB_CONFIG_CHANGE_EVENT_DATA1_PORT_ID_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for port connection not allowed (16 bytes) */

struct hwrm_async_event_cmpl_port_conn_not_allowed {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_TYPE_SFT 0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Port connection not allowed */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_ID_PORT_CONN_NOT_ALLOWED (UINT32_C(0x4) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_PORT_ID_SFT 0
	/*
	 * This value indicates the current port level enforcement policy for
	 * the optics module when there is an optical module mismatch and port
	 * is not connected.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_MASK UINT32_C(0xff0000)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_SFT 16
	/* No enforcement */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_NONE (UINT32_C(0x0) << 16)
	/* Disable Transmit side Laser. */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_DISABLETX (UINT32_C(0x1) << 16)
	/* Raise a warning message. */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_WARNINGMSG (UINT32_C(0x2) << 16)
	/* Power down the module. */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_PWRDOWN (UINT32_C(0x3) << 16)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_LAST	HWRM_ASYNC_EVENT_CMPL_PORT_CONN_NOT_ALLOWED_EVENT_DATA1_ENFORCEMENT_POLICY_PWRDOWN
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for link speed config not allowed (16 bytes) */

struct hwrm_async_event_cmpl_link_speed_cfg_not_allowed {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_TYPE_SFT 0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Link speed configuration was not allowed */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_EVENT_ID_LINK_SPEED_CFG_NOT_ALLOWED (UINT32_C(0x5) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_V UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_NOT_ALLOWED_EVENT_DATA1_PORT_ID_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for link speed configuration change (16 bytes) */

struct hwrm_async_event_cmpl_link_speed_cfg_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_TYPE_SFT 0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Link speed configuration change */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_EVENT_ID_LINK_SPEED_CFG_CHANGE (UINT32_C(0x6) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_EVENT_DATA1_PORT_ID_SFT 0
	/*
	 * If set to 1, it indicates that the supported link speeds
	 * configuration on the port has changed. If set to 0, then there is no
	 * change in supported link speeds configuration.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_EVENT_DATA1_SUPPORTED_LINK_SPEEDS_CHANGE UINT32_C(0x10000)
	/*
	 * If set to 1, it indicates that the link speed configuration on the
	 * port has become illegal or invalid. If set to 0, then the link speed
	 * configuration on the port is legal or valid.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_LINK_SPEED_CFG_CHANGE_EVENT_DATA1_ILLEGAL_LINK_SPEED_CFG UINT32_C(0x20000)
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for port PHY configuration change (16 bytes) */

struct hwrm_async_event_cmpl_port_phy_cfg_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_TYPE_SFT 0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Port PHY configuration change */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_EVENT_ID_PORT_PHY_CFG_CHANGE (UINT32_C(0x7) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PORT ID */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_EVENT_DATA1_PORT_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_EVENT_DATA1_PORT_ID_SFT 0
	/*
	 * If set to 1, it indicates that the FEC configuration on the port has
	 * changed. If set to 0, then there is no change in FEC configuration.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_EVENT_DATA1_FEC_CFG_CHANGE UINT32_C(0x10000)
	/*
	 * If set to 1, it indicates that the EEE configuration on the port has
	 * changed. If set to 0, then there is no change in EEE configuration on
	 * the port.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_EVENT_DATA1_EEE_CFG_CHANGE UINT32_C(0x20000)
	/*
	 * If set to 1, it indicates that the pause configuration on the PHY has
	 * changed. If set to 0, then there is no change in the pause
	 * configuration on the PHY.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PORT_PHY_CFG_CHANGE_EVENT_DATA1_PAUSE_CFG_CHANGE UINT32_C(0x40000)
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for Function Driver Unload (16 bytes) */

struct hwrm_async_event_cmpl_func_drvr_unload {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_TYPE_MASK   UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Function driver unloaded */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_EVENT_ID_FUNC_DRVR_UNLOAD (UINT32_C(0x10) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_OPAQUE_SFT  1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* Function ID */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_EVENT_DATA1_FUNC_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_UNLOAD_EVENT_DATA1_FUNC_ID_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for Function Driver load (16 bytes) */

struct hwrm_async_event_cmpl_func_drvr_load {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_TYPE_MASK	UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* Function driver loaded */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_EVENT_ID_FUNC_DRVR_LOAD (UINT32_C(0x11) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_V		UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_OPAQUE_MASK   UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_OPAQUE_SFT	1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* Function ID */
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_EVENT_DATA1_FUNC_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_FUNC_DRVR_LOAD_EVENT_DATA1_FUNC_ID_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for PF Driver Unload (16 bytes) */

struct hwrm_async_event_cmpl_pf_drvr_unload {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_TYPE_MASK	UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* PF driver unloaded */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_EVENT_ID_PF_DRVR_UNLOAD (UINT32_C(0x20) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_V		UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_OPAQUE_MASK   UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_OPAQUE_SFT	1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PF ID */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_EVENT_DATA1_FUNC_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_EVENT_DATA1_FUNC_ID_SFT 0
	/* Indicates the physical port this pf belongs to */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_EVENT_DATA1_PORT_MASK UINT32_C(0x70000)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_UNLOAD_EVENT_DATA1_PORT_SFT 16
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for PF Driver load (16 bytes) */

struct hwrm_async_event_cmpl_pf_drvr_load {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_TYPE_MASK	UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* PF driver loaded */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_EVENT_ID_PF_DRVR_LOAD (UINT32_C(0x21) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_V		UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_OPAQUE_MASK	UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_OPAQUE_SFT	1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* PF ID */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_EVENT_DATA1_FUNC_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_EVENT_DATA1_FUNC_ID_SFT 0
	/* Indicates the physical port this pf belongs to */
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_EVENT_DATA1_PORT_MASK UINT32_C(0x70000)
	#define HWRM_ASYNC_EVENT_CMPL_PF_DRVR_LOAD_EVENT_DATA1_PORT_SFT 16
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for VF FLR (16 bytes) */

struct hwrm_async_event_cmpl_vf_flr {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_TYPE_MASK		UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_TYPE_SFT		0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* VF Function Level Reset (FLR) */
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_EVENT_ID_VF_FLR	(UINT32_C(0x30) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_V			UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_OPAQUE_MASK	UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_OPAQUE_SFT		1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* VF ID */
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_EVENT_DATA1_VF_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_VF_FLR_EVENT_DATA1_VF_ID_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for VF MAC Addr change (16 bytes) */

struct hwrm_async_event_cmpl_vf_mac_addr_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_TYPE_SFT  0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* VF MAC Address Change */
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_EVENT_ID_VF_MAC_ADDR_CHANGE (UINT32_C(0x31) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_V	UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* VF ID */
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_EVENT_DATA1_VF_ID_MASK UINT32_C(0xffff)
	#define HWRM_ASYNC_EVENT_CMPL_VF_MAC_ADDR_CHANGE_EVENT_DATA1_VF_ID_SFT 0
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for PF-VF communication status change (16 bytes) */

struct hwrm_async_event_cmpl_pf_vf_comm_status_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_TYPE_MASK UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_TYPE_SFT 0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* PF-VF communication channel status change. */
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_EVENT_ID_PF_VF_COMM_STATUS_CHANGE (UINT32_C(0x32) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_V   UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_OPAQUE_MASK UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_OPAQUE_SFT 1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/*
	 * If this bit is set to 1, then it indicates that the PF-VF
	 * communication was lost and it is established. If this bit set to 0,
	 * then it indicates that the PF-VF communication was established and it
	 * is lost.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_PF_VF_COMM_STATUS_CHANGE_EVENT_DATA1_COMM_ESTABLISHED UINT32_C(0x1)
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for VF configuration change (16 bytes) */

struct hwrm_async_event_cmpl_vf_cfg_change {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_TYPE_MASK	UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* VF Configuration Change */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_EVENT_ID_VF_CFG_CHANGE (UINT32_C(0x33) << 0)
	uint32_t event_data2;
	/* Event specific data */
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_V		UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_OPAQUE_MASK	UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_OPAQUE_SFT	1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/*
	 * Each flag provided in this field indicates a specific VF
	 * configuration change. At least one of these flags shall be set to 1
	 * when an asynchronous event completion of this type is provided by the
	 * HWRM.
	 */
	/*
	 * If this bit is set to 1, then the value of MTU was changed on this
	 * VF. If set to 0, then this bit should be ignored.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_EVENT_DATA1_MTU_CHANGE UINT32_C(0x1)
	/*
	 * If this bit is set to 1, then the value of MRU was changed on this
	 * VF. If set to 0, then this bit should be ignored.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_EVENT_DATA1_MRU_CHANGE UINT32_C(0x2)
	/*
	 * If this bit is set to 1, then the value of default MAC address was
	 * changed on this VF. If set to 0, then this bit should be ignored.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_EVENT_DATA1_DFLT_MAC_ADDR_CHANGE UINT32_C(0x4)
	/*
	 * If this bit is set to 1, then the value of default VLAN was changed
	 * on this VF. If set to 0, then this bit should be ignored.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_VF_CFG_CHANGE_EVENT_DATA1_DFLT_VLAN_CHANGE UINT32_C(0x8)
} __attribute__((packed));

/* HWRM Asynchronous Event Completion Record for HWRM Error (16 bytes) */

struct hwrm_async_event_cmpl_hwrm_error {
	uint16_t type;
	/* unused1 is 10 b */
	/*
	 * This field indicates the exact type of the completion. By convention,
	 * the LSB identifies the length of the record in 16B units. Even values
	 * indicate 16B records. Odd values indicate 32B records.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_TYPE_MASK	UINT32_C(0x3f)
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_TYPE_SFT	0
	/* HWRM Asynchronous Event Information */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_TYPE_HWRM_ASYNC_EVENT (UINT32_C(0x2e) << 0)
	/* unused1 is 10 b */
	uint16_t event_id;
	/* Identifiers of events. */
	/* HWRM Error */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_ID_HWRM_ERROR (UINT32_C(0xff) << 0)
	uint32_t event_data2;
	/* Event specific data */
	/* Severity of HWRM Error */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_MASK UINT32_C(0xff)
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_SFT 0
	/* Warning */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_WARNING (UINT32_C(0x0) << 0)
	/* Non-fatal Error */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_NONFATAL (UINT32_C(0x1) << 0)
	/* Fatal Error */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_FATAL (UINT32_C(0x2) << 0)
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_LAST	HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA2_SEVERITY_FATAL
	uint8_t opaque_v;
	/* opaque is 7 b */
	/*
	 * This value is written by the NIC such that it will be different for
	 * each pass through the completion queue. The even passes will write 1.
	 * The odd passes will write 0.
	 */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_V		UINT32_C(0x1)
	/* opaque is 7 b */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_OPAQUE_MASK	UINT32_C(0xfe)
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_OPAQUE_SFT	1
	uint8_t timestamp_lo;
	/* 8-lsb timestamp from POR (100-msec resolution) */
	uint16_t timestamp_hi;
	/* 16-lsb timestamp from POR (100-msec resolution) */
	uint32_t event_data1;
	/* Event specific data */
	/* Time stamp for error event */
	#define HWRM_ASYNC_EVENT_CMPL_HWRM_ERROR_EVENT_DATA1_TIMESTAMP UINT32_C(0x1)
} __attribute__((packed));

/* Door Bell Formats */
/*
 * Description: The backup version of the packet must be valid in the TX ring
 * before the push doorbell is written to the chip. The first 32b and the BD
 * portion of the push doorbell must be written in multiples of 32b units on the
 * PCI interface. The data portion of the push doorbell may be written in
 * multiples of 8b units on the PCI interface. A push update must contain
 * exactly one push packet. The backup version of the packet must start with a
 * long (32B) BD and the BDs must be less than or equal to 16x16B units long.
 */
/*
 * Note: This door bell format is used by the driver when it wants to push a
 * packet into the chip for super-fast transmission. This pushes a partial BD
 * and the packet data into the chip. If the chip has room, it will transmit the
 * packet. If the chip dosn't have room, it will read the BD and packet data
 * from host memory as a normal packet.
 */
/* TX Door Bell Format (4 bytes) */

struct tx_doorbell {
	uint32_t key_idx;
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '0' for TX door bell operations.
	 */
	/*
	 * BD Index of next BD that will be used to transmit data on the TX ring
	 * mapped to this door bell. NIC may read and process all BDs up to, but
	 * not including this index.
	 */
	#define TX_DOORBELL_IDX_MASK				UINT32_C(0xffffff)
	#define TX_DOORBELL_IDX_SFT				0
	/* reserved is 4 b */
	#define TX_DOORBELL_RESERVED_MASK			UINT32_C(0xf000000)
	#define TX_DOORBELL_RESERVED_SFT			24
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '0' for TX door bell operations.
	 */
	#define TX_DOORBELL_KEY_MASK				UINT32_C(0xf0000000)
	#define TX_DOORBELL_KEY_SFT				28
	/* TX Operation */
	#define TX_DOORBELL_KEY_TX				(UINT32_C(0x0) << 28)
} __attribute__((packed));

/* RX Door Bell Format (4 bytes) */

struct rx_doorbell {
	uint32_t key_idx;
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '1' for RX door bell operations.
	 */
	/*
	 * BD Index of next BD that will be used for an empty receive buffer on
	 * the RX ring mapped to this door bell. NIC may read and process all
	 * BDs up to, but not including this index.
	 */
	#define RX_DOORBELL_IDX_MASK				UINT32_C(0xffffff)
	#define RX_DOORBELL_IDX_SFT				0
	/* reserved is 4 b */
	#define RX_DOORBELL_RESERVED_MASK			UINT32_C(0xf000000)
	#define RX_DOORBELL_RESERVED_SFT			24
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '1' for RX door bell operations.
	 */
	#define RX_DOORBELL_KEY_MASK				UINT32_C(0xf0000000)
	#define RX_DOORBELL_KEY_SFT				28
	/* RX Operation */
	#define RX_DOORBELL_KEY_RX				(UINT32_C(0x1) << 28)
} __attribute__((packed));

/* CMP Door Bell Format (4 bytes) */

struct cmpl_doorbell {
	uint32_t key_mask_valid_idx;
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '2' for CMP door bell operations.
	 */
	/*
	 * BD Index of the most recently handed completion record on the
	 * completion ring mapped to this door bell. NIC may write this location
	 * again with a new completion.
	 */
	#define CMPL_DOORBELL_IDX_MASK				UINT32_C(0xffffff)
	#define CMPL_DOORBELL_IDX_SFT				0
	/* reserved is 2 b */
	#define CMPL_DOORBELL_RESERVED_MASK			UINT32_C(0x3000000)
	#define CMPL_DOORBELL_RESERVED_SFT			24
	/*
	 * This indicates if the BDIDX value is valid for this update when it is
	 * '1'. When it is '0', the BDIDX value should be ignored.
	 */
	#define CMPL_DOORBELL_IDX_VALID				UINT32_C(0x4000000)
	/*
	 * This bit indicates the new interrupt mask state for the interrupt
	 * associated with the BDIDX. A '1', means the interrupt is to be
	 * masked. A '0' indicates the interrupt is to be unmasked.
	 */
	#define CMPL_DOORBELL_MASK				UINT32_C(0x8000000)
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '2' for CMP door bell operations.
	 */
	#define CMPL_DOORBELL_KEY_MASK				UINT32_C(0xf0000000)
	#define CMPL_DOORBELL_KEY_SFT				28
	/* Completion Operation */
	#define CMPL_DOORBELL_KEY_CMPL				(UINT32_C(0x2) << 28)
} __attribute__((packed));

/* Status Door Bell Format (4 bytes) */

struct status_doorbell {
	uint32_t key_idx;
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '3' for Status door bell operations.
	 */
	/*
	 * BD Index of the status record for which space is now available to the
	 * NIC.
	 */
	/*
	 * While there is no actual BD associated with the index, the similar
	 * scheme is being used to communicate to the NIC that space is
	 * available for status completions.
	 */
	#define STATUS_DOORBELL_IDX_MASK			UINT32_C(0xffffff)
	#define STATUS_DOORBELL_IDX_SFT				0
	/* reserved is 4 b */
	#define STATUS_DOORBELL_RESERVED_MASK			UINT32_C(0xf000000)
	#define STATUS_DOORBELL_RESERVED_SFT			24
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is '3' for Status door bell operations.
	 */
	#define STATUS_DOORBELL_KEY_MASK			UINT32_C(0xf0000000)
	#define STATUS_DOORBELL_KEY_SFT				28
	/* Status Operation */
	#define STATUS_DOORBELL_KEY_STAT			(UINT32_C(0x3) << 28)
} __attribute__((packed));

/* Push w/32B BD Door Bell Format (32 bytes) */

struct push32_doorbell {
	uint32_t key_sz_idx;
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is 4 for push door bell operations.
	 */
	/*
	 * This is the BD Index of last BD of the push packet that will be used
	 * to transmit data on the TX ring mapped to this door bell.
	 */
	#define PUSH32_DOORBELL_IDX_MASK			UINT32_C(0xffffff)
	#define PUSH32_DOORBELL_IDX_SFT				0
	/*
	 * This is the number of 16B BDs spaces consumed in the TX Ring by the
	 * "backup" version of the packet being pushed. A value of 1 is invalid
	 * since backup must start with a long 32B BE. A value of 2 indicates
	 * just the first 32B BE. A value of 3 indicates 32B+16B BD. etc. A
	 * value of 0 indicates 16x16B BD spaces are consumed.
	 */
	#define PUSH32_DOORBELL_SZ_MASK				UINT32_C(0xf000000)
	#define PUSH32_DOORBELL_SZ_SFT				24
	/*
	 * This value indicates the type of door bell operation that is begin
	 * requested. This value is 4 for push door bell operations.
	 */
	#define PUSH32_DOORBELL_KEY_MASK			UINT32_C(0xf0000000)
	#define PUSH32_DOORBELL_KEY_SFT				28
	/* Push Operation */
	#define PUSH32_DOORBELL_KEY_PUSH			(UINT32_C(0x4) << 28)
	uint16_t flags_type;
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Only the packet_end bit must be valid for the remaining BDs of a
	 * packet.
	 */
	/* This value identifies the type of buffer descriptor. */
	#define PUSH32_DOORBELL_TYPE_MASK			UINT32_C(0x3f)
	#define PUSH32_DOORBELL_TYPE_SFT			0
	/*
	 * Indicates that this BD is 32B long and is used for normal L2
	 * packet transmission.
	 */
	#define PUSH32_DOORBELL_TYPE_TX_BD_LONG		(UINT32_C(0x10) << 0)
	/*
	 * If set to 1, the packet ends with the data in the buffer pointed to
	 * by this descriptor. This flag must be valid on every BD. This bit
	 * must be set on all push doorbells.
	 */
	#define PUSH32_DOORBELL_FLAGS_PACKET_END		UINT32_C(0x40)
	/*
	 * If set to 1, the device will not generate a completion for this
	 * transmit packet unless there is an error in it's processing. If this
	 * bit is set to 0, then the packet will be completed normally. This bit
	 * must be valid only on the first BD of a packet.
	 */
	#define PUSH32_DOORBELL_FLAGS_NO_CMPL			UINT32_C(0x80)
	/*
	 * This value must match the sz field in the first 32b of the push
	 * operation except that if 16x16B BD locations are consumed in the ring
	 * by this packet, then this value must be 16 (not zero).
	 */
	#define PUSH32_DOORBELL_FLAGS_BD_CNT_MASK		UINT32_C(0x1f00)
	#define PUSH32_DOORBELL_FLAGS_BD_CNT_SFT		8
	/*
	 * This value is a hint for the length of the entire packet. It is used
	 * by the chip to optimize internal processing. The packet will be
	 * dropped if the hint is too short. This field is valid only on the
	 * first BD of a packet.
	 */
	#define PUSH32_DOORBELL_FLAGS_LHINT_MASK		UINT32_C(0x6000)
	#define PUSH32_DOORBELL_FLAGS_LHINT_SFT			13
	/* indicates packet length < 512B */
	#define PUSH32_DOORBELL_FLAGS_LHINT_LT512		(UINT32_C(0x0) << 13)
	/* indicates 512 <= packet length < 1KB */
	#define PUSH32_DOORBELL_FLAGS_LHINT_LT1K		(UINT32_C(0x1) << 13)
	/* indicates 1KB <= packet length < 2KB */
	#define PUSH32_DOORBELL_FLAGS_LHINT_LT2K		(UINT32_C(0x2) << 13)
	/* indicates packet length >= 2KB */
	#define PUSH32_DOORBELL_FLAGS_LHINT_GTE2K		(UINT32_C(0x3) << 13)
	#define PUSH32_DOORBELL_FLAGS_LHINT_LAST	PUSH32_DOORBELL_FLAGS_LHINT_GTE2K
	/*
	 * If set to 1, the device immediately updates the Send Consumer Index
	 * after the buffer associated with this descriptor has been transferred
	 * via DMA to NIC memory from host memory. An interrupt may or may not
	 * be generated according to the state of the interrupt avoidance
	 * mechanisms. If this bit is set to 0, then the Consumer Index is only
	 * updated as soon as one of the host interrupt coalescing conditions
	 * has been met. This bit must be valid on the first BD of a packet.
	 */
	#define PUSH32_DOORBELL_FLAGS_COAL_NOW			UINT32_C(0x8000)
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Only the packet_end bit must be valid for the remaining BDs of a
	 * packet.
	 */
	#define PUSH32_DOORBELL_FLAGS_MASK			UINT32_C(0xffc0)
	#define PUSH32_DOORBELL_FLAGS_SFT			6
	uint16_t len;
	/*
	 * This is the length of the host physical buffer this BD describes in
	 * bytes. This field must be valid on all BDs of a packet.
	 */
	uint32_t opaque;
	/*
	 * The opaque data field is pass through to the completion and can be
	 * used for any data that the driver wants to associate with the
	 * transmit BD. This field must be valid on the first BD of a packet.
	 */
	uint16_t lflags;
	/*
	 * All bits in this field must be valid on the first BD of a packet.
	 * Their value on other BDs of the packet will be ignored.
	 */
	/*
	 * If set to 1, the controller replaces the TCP/UPD checksum fields of
	 * normal TCP/UPD checksum, or the inner TCP/UDP checksum field of the
	 * encapsulated TCP/UDP packets with the hardware calculated TCP/UDP
	 * checksum for the packet associated with this descriptor. The flag is
	 * ignored if the LSO flag is set. This bit must be valid on the first
	 * BD of a packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_TCP_UDP_CHKSUM		UINT32_C(0x1)
	/*
	 * If set to 1, the controller replaces the IP checksum of the normal
	 * packets, or the inner IP checksum of the encapsulated packets with
	 * the hardware calculated IP checksum for the packet associated with
	 * this descriptor. This bit must be valid on the first BD of a packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_IP_CHKSUM		UINT32_C(0x2)
	/*
	 * If set to 1, the controller will not append an Ethernet CRC to the
	 * end of the frame. This bit must be valid on the first BD of a packet.
	 * Packet must be 64B or longer when this flag is set. It is not useful
	 * to use this bit with any form of TX offload such as CSO or LSO. The
	 * intent is that the packet from the host already has a valid Ethernet
	 * CRC on the packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_NOCRC			UINT32_C(0x4)
	/*
	 * If set to 1, the device will record the time at which the packet was
	 * actually transmitted at the TX MAC. This bit must be valid on the
	 * first BD of a packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_STAMP			UINT32_C(0x8)
	/*
	 * If set to 1, The controller replaces the tunnel IP checksum field
	 * with hardware calculated IP checksum for the IP header of the packet
	 * associated with this descriptor. For outer UDP checksum, global outer
	 * UDP checksum TE_NIC register needs to be enabled. If the global outer
	 * UDP checksum TE_NIC register bit is set, outer UDP checksum will be
	 * calculated for the following cases: 1. Packets with tcp_udp_chksum
	 * flag set to offload checksum for inner packet AND the inner packet is
	 * TCP/UDP. If the inner packet is ICMP for example (non-TCP/UDP), even
	 * if the tcp_udp_chksum is set, the outer UDP checksum will not be
	 * calculated. 2. Packets with lso flag set which implies inner TCP
	 * checksum calculation as part of LSO operation.
	 */
	#define PUSH32_DOORBELL_LFLAGS_T_IP_CHKSUM		UINT32_C(0x10)
	/*
	 * If set to 1, the device will treat this packet with LSO(Large Send
	 * Offload) processing for both normal or encapsulated packets, which is
	 * a form of TCP segmentation. When this bit is 1, the hdr_size and mss
	 * fields must be valid. The driver doesn't need to set t_ip_chksum,
	 * ip_chksum, and tcp_udp_chksum flags since the controller will replace
	 * the appropriate checksum fields for segmented packets. When this bit
	 * is 1, the hdr_size and mss fields must be valid.
	 */
	#define PUSH32_DOORBELL_LFLAGS_LSO			UINT32_C(0x20)
	/*
	 * If set to zero when LSO is '1', then the IPID will be treated as a
	 * 16b number and will be wrapped if it exceeds a value of 0xffff. If
	 * set to one when LSO is '1', then the IPID will be treated as a 15b
	 * number and will be wrapped if it exceeds a value 0f 0x7fff.
	 */
	#define PUSH32_DOORBELL_LFLAGS_IPID_FMT			UINT32_C(0x40)
	/*
	 * If set to zero when LSO is '1', then the IPID of the tunnel IP header
	 * will not be modified during LSO operations. If set to one when LSO is
	 * '1', then the IPID of the tunnel IP header will be incremented for
	 * each subsequent segment of an LSO operation. The flag is ignored if
	 * the LSO packet is a normal (non-tunneled) TCP packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_T_IPID			UINT32_C(0x80)
	/*
	 * If set to '1', then the RoCE ICRC will be appended to the packet.
	 * Packet must be a valid RoCE format packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_ROCE_CRC			UINT32_C(0x100)
	/*
	 * If set to '1', then the FCoE CRC will be appended to the packet.
	 * Packet must be a valid FCoE format packet.
	 */
	#define PUSH32_DOORBELL_LFLAGS_FCOE_CRC			UINT32_C(0x200)
	uint16_t hdr_size;
	/*
	 * When LSO is '1', this field must contain the offset of the TCP
	 * payload from the beginning of the packet in as 16b words. In case of
	 * encapsulated/tunneling packet, this field contains the offset of the
	 * inner TCP payload from beginning of the packet as 16-bit words. This
	 * value must be valid on the first BD of a packet.
	 */
	#define PUSH32_DOORBELL_HDR_SIZE_MASK			UINT32_C(0x1ff)
	#define PUSH32_DOORBELL_HDR_SIZE_SFT			0
	uint32_t mss;
	/*
	 * This is the MSS value that will be used to do the LSO processing. The
	 * value is the length in bytes of the TCP payload for each segment
	 * generated by the LSO operation. This value must be valid on the first
	 * BD of a packet.
	 */
	#define PUSH32_DOORBELL_MSS_MASK			UINT32_C(0x7fff)
	#define PUSH32_DOORBELL_MSS_SFT				0
	uint16_t unused_2;
	uint16_t cfa_action;
	/*
	 * This value selects a CFA action to perform on the packet. Set this
	 * value to zero if no CFA action is desired. This value must be valid
	 * on the first BD of a packet.
	 */
	uint32_t cfa_meta;
	/*
	 * This value is action meta-data that defines CFA edit operations that
	 * are done in addition to any action editing.
	 */
	/* When key=1, This is the VLAN tag VID value. */
	#define PUSH32_DOORBELL_CFA_META_VLAN_VID_MASK		UINT32_C(0xfff)
	#define PUSH32_DOORBELL_CFA_META_VLAN_VID_SFT		0
	/* When key=1, This is the VLAN tag DE value. */
	#define PUSH32_DOORBELL_CFA_META_VLAN_DE		UINT32_C(0x1000)
	/* When key=1, This is the VLAN tag PRI value. */
	#define PUSH32_DOORBELL_CFA_META_VLAN_PRI_MASK		UINT32_C(0xe000)
	#define PUSH32_DOORBELL_CFA_META_VLAN_PRI_SFT		13
	/* When key=1, This is the VLAN tag TPID select value. */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_MASK		UINT32_C(0x70000)
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_SFT		16
	/* 0x88a8 */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPID88A8	(UINT32_C(0x0) << 16)
	/* 0x8100 */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPID8100	(UINT32_C(0x1) << 16)
	/* 0x9100 */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPID9100	(UINT32_C(0x2) << 16)
	/* 0x9200 */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPID9200	(UINT32_C(0x3) << 16)
	/* 0x9300 */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPID9300	(UINT32_C(0x4) << 16)
	/* Value programmed in CFA VLANTPID register. */
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPIDCFG	(UINT32_C(0x5) << 16)
	#define PUSH32_DOORBELL_CFA_META_VLAN_TPID_LAST	PUSH32_DOORBELL_CFA_META_VLAN_TPID_TPIDCFG
	/* When key=1, This is the VLAN tag TPID select value. */
	#define PUSH32_DOORBELL_CFA_META_VLAN_RESERVED_MASK	UINT32_C(0xff80000)
	#define PUSH32_DOORBELL_CFA_META_VLAN_RESERVED_SFT	19
	/*
	 * This field identifies the type of edit to be performed on the packet.
	 * This value must be valid on the first BD of a packet.
	 */
	#define PUSH32_DOORBELL_CFA_META_KEY_MASK		UINT32_C(0xf0000000)
	#define PUSH32_DOORBELL_CFA_META_KEY_SFT		28
	/* No editing */
	#define PUSH32_DOORBELL_CFA_META_KEY_NONE		(UINT32_C(0x0) << 28)
	/*
	 * - meta[17:16] - TPID select value (0 = 0x8100). - meta[15:12]
	 * - PRI/DE value. - meta[11:0] - VID value.
	 */
	#define PUSH32_DOORBELL_CFA_META_KEY_VLAN_TAG		(UINT32_C(0x1) << 28)
	#define PUSH32_DOORBELL_CFA_META_KEY_LAST	PUSH32_DOORBELL_CFA_META_KEY_VLAN_TAG
	uint32_t data[25];
	/*
	 * This is the data for the push packet. If the packet data does not fit
	 * in the first pass, data writing can continue at offset 4 of the
	 * doorbell for up to 4 additional passes for a total data size of 512B
	 * maximum.
	 */
} __attribute__((packed));

/* HW Resource Manager Specification 1.3.0 */
#define HWRM_VERSION_MAJOR	1
#define HWRM_VERSION_MINOR	3
#define HWRM_VERSION_UPDATE	0

#define HWRM_VERSION_STR	"1.3.0"
/*
 * Following is the signature for HWRM message field that indicates not
 * applicable (All F's). Need to cast it the size of the field if needed.
 */
#define HWRM_NA_SIGNATURE	((uint32_t)(-1))
#define HWRM_MAX_REQ_LEN	(128)  /* hwrm_func_buf_rgtr */
#define HWRM_MAX_RESP_LEN	(176)  /* hwrm_func_qstats */
#define HW_HASH_INDEX_SIZE	0x80	/* 7 bit indirection table index. */
#define HW_HASH_KEY_SIZE	40
#define HWRM_RESP_VALID_KEY	1 /* valid key for HWRM response */
/*
 * Description: Port Rx Statistics Formats. The HWRM shall return any
 * unsupported counter with a value of 0xFFFFFFFF for 32-bit counters and
 * 0xFFFFFFFFFFFFFFFF for 64-bit counters.
 */
/*
 * Note: The Hardware Resource Manager (HWRM) manages various hardware resources
 * inside the chip. The HWRM is implemented in firmware, and runs on embedded
 * processors inside the chip. This firmware service is vital part of the chip.
 * The chip can not be used by a driver or HWRM client without the HWRM.
 */
/* Input (16 bytes) */

struct input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (8 bytes) */

struct output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
} __attribute__((packed));

#define GET_HWRM_REQ_TYPE(x) \
	((x) == 0x98 ? "HWRM_CFA_ENCAP_RECORD_FREE": \
	((x) == 0x99 ? "HWRM_CFA_NTUPLE_FILTER_ALLOC": \
	((x) == 0x90 ? "HWRM_CFA_L2_FILTER_ALLOC": \
	((x) == 0x91 ? "HWRM_CFA_L2_FILTER_FREE": \
	((x) == 0x92 ? "HWRM_CFA_L2_FILTER_CFG": \
	((x) == 0x93 ? "HWRM_CFA_L2_SET_RX_MASK": \
	((x) == 0x94 ? "RESERVED3": \
	((x) == 0x95 ? "HWRM_CFA_TUNNEL_FILTER_ALLOC": \
	((x) == 0x96 ? "HWRM_CFA_TUNNEL_FILTER_FREE": \
	((x) == 0x97 ? "HWRM_CFA_ENCAP_RECORD_ALLOC": \
	((x) == 0x10 ? "RESERVED1": \
	((x) == 0x11 ? "HWRM_FUNC_RESET": \
	((x) == 0x12 ? "HWRM_FUNC_GETFID": \
	((x) == 0x13 ? "HWRM_FUNC_VF_ALLOC": \
	((x) == 0x14 ? "HWRM_FUNC_VF_FREE": \
	((x) == 0x15 ? "HWRM_FUNC_QCAPS": \
	((x) == 0x16 ? "HWRM_FUNC_QCFG": \
	((x) == 0x17 ? "HWRM_FUNC_CFG": \
	((x) == 0x18 ? "HWRM_FUNC_QSTATS": \
	((x) == 0x19 ? "HWRM_FUNC_CLR_STATS": \
	((x) == 0xe0 ? "HWRM_TEMP_MONITOR_QUERY": \
	((x) == 0xd3 ? "HWRM_FWD_ASYNC_EVENT_CMPL": \
	((x) == 0xd2 ? "HWRM_FWD_RESP": \
	((x) == 0x1a ? "HWRM_FUNC_DRV_UNRGTR": \
	((x) == 0x1b ? "HWRM_FUNC_VF_RESC_FREE": \
	((x) == 0x1c ? "HWRM_FUNC_VF_VNIC_IDS_QUERY": \
	((x) == 0x1d ? "HWRM_FUNC_DRV_RGTR": \
	((x) == 0x1e ? "HWRM_FUNC_DRV_QVER": \
	((x) == 0x1f ? "HWRM_FUNC_BUF_RGTR": \
	((x) == 0x9a ? "HWRM_CFA_NTUPLE_FILTER_FREE": \
	((x) == 0x9b ? "HWRM_CFA_NTUPLE_FILTER_CFG": \
	((x) == 0x9c ? "HWRM_CFA_EM_FLOW_ALLOC": \
	((x) == 0x9d ? "HWRM_CFA_EM_FLOW_FREE": \
	((x) == 0x9e ? "HWRM_CFA_EM_FLOW_CFG": \
	((x) == 0xd1 ? "HWRM_REJECT_FWD_RESP": \
	((x) == 0xd0 ? "HWRM_EXEC_FWD_RESP": \
	((x) == 0xc0 ? "HWRM_FW_RESET": \
	((x) == 0xc1 ? "HWRM_FW_QSTATUS": \
	((x) == 0x70 ? "HWRM_VNIC_RSS_COS_LB_CTX_ALLOC": \
	((x) == 0x71 ? "HWRM_VNIC_RSS_COS_LB_CTX_FREE": \
	((x) == 0xb1 ? "HWRM_STAT_CTX_FREE": \
	((x) == 0xb0 ? "HWRM_STAT_CTX_ALLOC": \
	((x) == 0xb3 ? "HWRM_STAT_CTX_CLR_STATS": \
	((x) == 0xb2 ? "HWRM_STAT_CTX_QUERY": \
	((x) == 0xfff6 ? "HWRM_NVM_GET_DEV_INFO": \
	((x) == 0x61 ? "HWRM_RING_GRP_FREE": \
	((x) == 0x60 ? "HWRM_RING_GRP_ALLOC": \
	((x) == 0xf1 ? "HWRM_WOL_FILTER_FREE": \
	((x) == 0xf0 ? "HWRM_WOL_FILTER_ALLOC": \
	((x) == 0xf2 ? "HWRM_WOL_FILTER_QCFG": \
	((x) == 0xa0 ? "HWRM_TUNNEL_DST_PORT_QUERY": \
	((x) == 0xa1 ? "HWRM_TUNNEL_DST_PORT_ALLOC": \
	((x) == 0xa2 ? "HWRM_TUNNEL_DST_PORT_FREE": \
	((x) == 0xfffc ? "HWRM_NVM_RAW_DUMP": \
	((x) == 0xfffb ? "HWRM_NVM_GET_DIR_INFO": \
	((x) == 0xfffa ? "HWRM_NVM_GET_DIR_ENTRIES": \
	((x) == 0xe ? "HWRM_FUNC_BUF_UNRGTR": \
	((x) == 0xf ? "HWRM_FUNC_VF_CFG": \
	((x) == 0xffff ? "HWRM_NVM_RAW_WRITE_BLK": \
	((x) == 0xfffe ? "HWRM_NVM_WRITE": \
	((x) == 0xfffd ? "HWRM_NVM_READ": \
	((x) == 0x50 ? "HWRM_RING_ALLOC": \
	((x) == 0x51 ? "HWRM_RING_FREE": \
	((x) == 0x52 ? "HWRM_RING_CMPL_RING_QAGGINT_PARAMS": \
	((x) == 0x53 ? "HWRM_RING_CMPL_RING_CFG_AGGINT_PARAMS": \
	((x) == 0x49 ? "HWRM_VNIC_PLCMODES_QCFG": \
	((x) == 0x48 ? "HWRM_VNIC_PLCMODES_CFG": \
	((x) == 0x47 ? "HWRM_VNIC_RSS_QCFG": \
	((x) == 0x46 ? "HWRM_VNIC_RSS_CFG": \
	((x) == 0x45 ? "HWRM_VNIC_TPA_QCFG": \
	((x) == 0x44 ? "HWRM_VNIC_TPA_CFG": \
	((x) == 0x43 ? "HWRM_VNIC_QCFG": \
	((x) == 0x42 ? "HWRM_VNIC_CFG": \
	((x) == 0x41 ? "HWRM_VNIC_FREE": \
	((x) == 0x40 ? "HWRM_VNIC_ALLOC": \
	((x) == 0x0 ? "HWRM_VER_GET": \
	((x) == 0xfff9 ? "HWRM_NVM_FIND_DIR_ENTRY": \
	((x) == 0xfff8 ? "HWRM_NVM_MOD_DIR_ENTRY": \
	((x) == 0xfff7 ? "HWRM_NVM_ERASE_DIR_ENTRY": \
	((x) == 0x5e ? "HWRM_RING_RESET": \
	((x) == 0xfff5 ? "HWRM_NVM_VERIFY_UPDATE": \
	((x) == 0xfff4 ? "HWRM_NVM_MODIFY": \
	((x) == 0x2a ? "HWRM_PORT_PHY_QCAPS": \
	((x) == 0x2c ? "HWRM_PORT_PHY_I2C_READ": \
	((x) == 0x2b ? "HWRM_PORT_PHY_I2C_WRITE": \
	((x) == 0x38 ? "HWRM_QUEUE_PRI2COS_CFG": \
	((x) == 0x39 ? "HWRM_QUEUE_COS2BW_QCFG": \
	((x) == 0x32 ? "HWRM_QUEUE_CFG": \
	((x) == 0x33 ? "HWRM_QUEUE_BUFFERS_QCFG": \
	((x) == 0x30 ? "HWRM_QUEUE_QPORTCFG": \
	((x) == 0x31 ? "HWRM_QUEUE_QCFG": \
	((x) == 0x36 ? "HWRM_QUEUE_PFCENABLE_CFG": \
	((x) == 0x37 ? "HWRM_QUEUE_PRI2COS_QCFG": \
	((x) == 0x34 ? "HWRM_QUEUE_BUFFERS_CFG": \
	((x) == 0x35 ? "HWRM_QUEUE_PFCENABLE_QCFG": \
	((x) == 0xff14 ? "HWRM_DBG_DUMP": \
	((x) == 0xff12 ? "HWRM_DBG_WRITE_DIRECT": \
	((x) == 0xff13 ? "HWRM_DBG_WRITE_INDIRECT": \
	((x) == 0xff10 ? "HWRM_DBG_READ_DIRECT": \
	((x) == 0xff11 ? "HWRM_DBG_READ_INDIRECT": \
	((x) == 0x25 ? "HWRM_PORT_CLR_STATS": \
	((x) == 0x24 ? "HWRM_PORT_LPBK_QSTATS": \
	((x) == 0x27 ? "HWRM_PORT_PHY_QCFG": \
	((x) == 0x26 ? "HWRM_PORT_LPBK_CLR_STATS": \
	((x) == 0x21 ? "HWRM_PORT_MAC_CFG": \
	((x) == 0x20 ? "HWRM_PORT_PHY_CFG": \
	((x) == 0x23 ? "HWRM_PORT_QSTATS": \
	((x) == 0x22 ? "HWRM_PORT_TS_QUERY": \
	((x) == 0x29 ? "HWRM_PORT_BLINK_LED": \
	((x) == 0x28 ? "HWRM_PORT_MAC_QCFG": \
	((x) == 0x3a ? "HWRM_QUEUE_COS2BW_CFG": \
	"Unknown req_type")))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))

/* Command numbering (8 bytes) */

struct cmd_nums {
	uint16_t req_type;
	/*
	 * This version of the specification defines the commands listed in the
	 * table below. The following are general implementation requirements
	 * for these commands: # All commands listed below that are marked
	 * neither reserved nor experimental shall be implemented by the HWRM. #
	 * A HWRM client compliant to this specification should not use commands
	 * outside of the list below. # A HWRM client compliant to this
	 * specification should not use command numbers marked reserved below. #
	 * A command marked experimental below may not be implemented by the
	 * HWRM. # A command marked experimental may change in the future
	 * version of the HWRM specification. # A command not listed below may
	 * be implemented by the HWRM. The behavior of commands that are not
	 * listed below is outside the scope of this specification.
	 */
	#define HWRM_VER_GET					(UINT32_C(0x0))
	#define HWRM_FUNC_BUF_UNRGTR				(UINT32_C(0xe))
	/* Experimental */
	#define HWRM_FUNC_VF_CFG				(UINT32_C(0xf))
	/* Reserved for future use */
	#define RESERVED1					(UINT32_C(0x10))
	#define HWRM_FUNC_RESET				(UINT32_C(0x11))
	#define HWRM_FUNC_GETFID				(UINT32_C(0x12))
	#define HWRM_FUNC_VF_ALLOC				(UINT32_C(0x13))
	#define HWRM_FUNC_VF_FREE				(UINT32_C(0x14))
	#define HWRM_FUNC_QCAPS				(UINT32_C(0x15))
	#define HWRM_FUNC_QCFG					(UINT32_C(0x16))
	#define HWRM_FUNC_CFG					(UINT32_C(0x17))
	#define HWRM_FUNC_QSTATS				(UINT32_C(0x18))
	#define HWRM_FUNC_CLR_STATS				(UINT32_C(0x19))
	#define HWRM_FUNC_DRV_UNRGTR				(UINT32_C(0x1a))
	#define HWRM_FUNC_VF_RESC_FREE				(UINT32_C(0x1b))
	#define HWRM_FUNC_VF_VNIC_IDS_QUERY			(UINT32_C(0x1c))
	#define HWRM_FUNC_DRV_RGTR				(UINT32_C(0x1d))
	#define HWRM_FUNC_DRV_QVER				(UINT32_C(0x1e))
	#define HWRM_FUNC_BUF_RGTR				(UINT32_C(0x1f))
	#define HWRM_PORT_PHY_CFG				(UINT32_C(0x20))
	#define HWRM_PORT_MAC_CFG				(UINT32_C(0x21))
	/* Experimental */
	#define HWRM_PORT_TS_QUERY				(UINT32_C(0x22))
	#define HWRM_PORT_QSTATS				(UINT32_C(0x23))
	#define HWRM_PORT_LPBK_QSTATS				(UINT32_C(0x24))
	/* Experimental */
	#define HWRM_PORT_CLR_STATS				(UINT32_C(0x25))
	/* Experimental */
	#define HWRM_PORT_LPBK_CLR_STATS			(UINT32_C(0x26))
	#define HWRM_PORT_PHY_QCFG				(UINT32_C(0x27))
	/* Experimental */
	#define HWRM_PORT_MAC_QCFG				(UINT32_C(0x28))
	/* Experimental */
	#define HWRM_PORT_BLINK_LED				(UINT32_C(0x29))
	/* Experimental */
	#define HWRM_PORT_PHY_QCAPS				(UINT32_C(0x2a))
	/* Experimental */
	#define HWRM_PORT_PHY_I2C_WRITE			(UINT32_C(0x2b))
	/* Experimental */
	#define HWRM_PORT_PHY_I2C_READ				(UINT32_C(0x2c))
	#define HWRM_QUEUE_QPORTCFG				(UINT32_C(0x30))
	#define HWRM_QUEUE_QCFG				(UINT32_C(0x31))
	#define HWRM_QUEUE_CFG					(UINT32_C(0x32))
	#define HWRM_QUEUE_BUFFERS_QCFG			(UINT32_C(0x33))
	#define HWRM_QUEUE_BUFFERS_CFG				(UINT32_C(0x34))
	/* Experimental */
	#define HWRM_QUEUE_PFCENABLE_QCFG			(UINT32_C(0x35))
	/* Experimental */
	#define HWRM_QUEUE_PFCENABLE_CFG			(UINT32_C(0x36))
	/* Experimental */
	#define HWRM_QUEUE_PRI2COS_QCFG			(UINT32_C(0x37))
	/* Experimental */
	#define HWRM_QUEUE_PRI2COS_CFG				(UINT32_C(0x38))
	/* Experimental */
	#define HWRM_QUEUE_COS2BW_QCFG				(UINT32_C(0x39))
	/* Experimental */
	#define HWRM_QUEUE_COS2BW_CFG				(UINT32_C(0x3a))
	#define HWRM_VNIC_ALLOC				(UINT32_C(0x40))
	#define HWRM_VNIC_FREE					(UINT32_C(0x41))
	#define HWRM_VNIC_CFG					(UINT32_C(0x42))
	/* Experimental */
	#define HWRM_VNIC_QCFG					(UINT32_C(0x43))
	#define HWRM_VNIC_TPA_CFG				(UINT32_C(0x44))
	/* Experimental */
	#define HWRM_VNIC_TPA_QCFG				(UINT32_C(0x45))
	#define HWRM_VNIC_RSS_CFG				(UINT32_C(0x46))
	#define HWRM_VNIC_RSS_QCFG				(UINT32_C(0x47))
	#define HWRM_VNIC_PLCMODES_CFG				(UINT32_C(0x48))
	#define HWRM_VNIC_PLCMODES_QCFG			(UINT32_C(0x49))
	#define HWRM_RING_ALLOC				(UINT32_C(0x50))
	#define HWRM_RING_FREE					(UINT32_C(0x51))
	#define HWRM_RING_CMPL_RING_QAGGINT_PARAMS		(UINT32_C(0x52))
	#define HWRM_RING_CMPL_RING_CFG_AGGINT_PARAMS		(UINT32_C(0x53))
	#define HWRM_RING_RESET				(UINT32_C(0x5e))
	#define HWRM_RING_GRP_ALLOC				(UINT32_C(0x60))
	#define HWRM_RING_GRP_FREE				(UINT32_C(0x61))
	#define HWRM_VNIC_RSS_COS_LB_CTX_ALLOC			(UINT32_C(0x70))
	#define HWRM_VNIC_RSS_COS_LB_CTX_FREE			(UINT32_C(0x71))
	#define HWRM_CFA_L2_FILTER_ALLOC			(UINT32_C(0x90))
	#define HWRM_CFA_L2_FILTER_FREE			(UINT32_C(0x91))
	#define HWRM_CFA_L2_FILTER_CFG				(UINT32_C(0x92))
	#define HWRM_CFA_L2_SET_RX_MASK			(UINT32_C(0x93))
	/* Reserved for future use */
	#define RESERVED3					(UINT32_C(0x94))
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC			(UINT32_C(0x95))
	#define HWRM_CFA_TUNNEL_FILTER_FREE			(UINT32_C(0x96))
	/* Experimental */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC			(UINT32_C(0x97))
	/* Experimental */
	#define HWRM_CFA_ENCAP_RECORD_FREE			(UINT32_C(0x98))
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC			(UINT32_C(0x99))
	#define HWRM_CFA_NTUPLE_FILTER_FREE			(UINT32_C(0x9a))
	#define HWRM_CFA_NTUPLE_FILTER_CFG			(UINT32_C(0x9b))
	/* Experimental */
	#define HWRM_CFA_EM_FLOW_ALLOC				(UINT32_C(0x9c))
	/* Experimental */
	#define HWRM_CFA_EM_FLOW_FREE				(UINT32_C(0x9d))
	/* Experimental */
	#define HWRM_CFA_EM_FLOW_CFG				(UINT32_C(0x9e))
	#define HWRM_TUNNEL_DST_PORT_QUERY			(UINT32_C(0xa0))
	#define HWRM_TUNNEL_DST_PORT_ALLOC			(UINT32_C(0xa1))
	#define HWRM_TUNNEL_DST_PORT_FREE			(UINT32_C(0xa2))
	#define HWRM_STAT_CTX_ALLOC				(UINT32_C(0xb0))
	#define HWRM_STAT_CTX_FREE				(UINT32_C(0xb1))
	#define HWRM_STAT_CTX_QUERY				(UINT32_C(0xb2))
	#define HWRM_STAT_CTX_CLR_STATS			(UINT32_C(0xb3))
	#define HWRM_FW_RESET					(UINT32_C(0xc0))
	#define HWRM_FW_QSTATUS				(UINT32_C(0xc1))
	#define HWRM_EXEC_FWD_RESP				(UINT32_C(0xd0))
	#define HWRM_REJECT_FWD_RESP				(UINT32_C(0xd1))
	#define HWRM_FWD_RESP					(UINT32_C(0xd2))
	#define HWRM_FWD_ASYNC_EVENT_CMPL			(UINT32_C(0xd3))
	#define HWRM_TEMP_MONITOR_QUERY			(UINT32_C(0xe0))
	/* Experimental */
	#define HWRM_WOL_FILTER_ALLOC				(UINT32_C(0xf0))
	/* Experimental */
	#define HWRM_WOL_FILTER_FREE				(UINT32_C(0xf1))
	/* Experimental */
	#define HWRM_WOL_FILTER_QCFG				(UINT32_C(0xf2))
	/* Experimental */
	#define HWRM_DBG_READ_DIRECT				(UINT32_C(0xff10))
	/* Experimental */
	#define HWRM_DBG_READ_INDIRECT				(UINT32_C(0xff11))
	/* Experimental */
	#define HWRM_DBG_WRITE_DIRECT				(UINT32_C(0xff12))
	/* Experimental */
	#define HWRM_DBG_WRITE_INDIRECT			(UINT32_C(0xff13))
	#define HWRM_DBG_DUMP					(UINT32_C(0xff14))
	#define HWRM_NVM_MODIFY				(UINT32_C(0xfff4))
	#define HWRM_NVM_VERIFY_UPDATE				(UINT32_C(0xfff5))
	#define HWRM_NVM_GET_DEV_INFO				(UINT32_C(0xfff6))
	#define HWRM_NVM_ERASE_DIR_ENTRY			(UINT32_C(0xfff7))
	#define HWRM_NVM_MOD_DIR_ENTRY				(UINT32_C(0xfff8))
	#define HWRM_NVM_FIND_DIR_ENTRY			(UINT32_C(0xfff9))
	#define HWRM_NVM_GET_DIR_ENTRIES			(UINT32_C(0xfffa))
	#define HWRM_NVM_GET_DIR_INFO				(UINT32_C(0xfffb))
	#define HWRM_NVM_RAW_DUMP				(UINT32_C(0xfffc))
	#define HWRM_NVM_READ					(UINT32_C(0xfffd))
	#define HWRM_NVM_WRITE					(UINT32_C(0xfffe))
	#define HWRM_NVM_RAW_WRITE_BLK				(UINT32_C(0xffff))
	uint16_t unused_0[3];
} __attribute__((packed));

#define GET_HWRM_ERROR_CODE(x) \
	((x) == 0xf ? "HWRM_ERROR": \
	((x) == 0xffff ? "CMD_NOT_SUPPORTED": \
	((x) == 0xfffe ? "UNKNOWN_ERR": \
	((x) == 0x4 ? "RESOURCE_ALLOC_ERROR": \
	((x) == 0x5 ? "INVALID_FLAGS": \
	((x) == 0x6 ? "INVALID_ENABLES": \
	((x) == 0x0 ? "SUCCESS": \
	((x) == 0x1 ? "FAIL": \
	((x) == 0x2 ? "INVALID_PARAMS": \
	((x) == 0x3 ? "RESOURCE_ACCESS_DENIED": \
	"Unknown error_code"))))))))))

/* Return Codes (8 bytes) */

struct ret_codes {
	uint16_t error_code;
	/* These are numbers assigned to return/error codes. */
	/* Request was successfully executed by the HWRM. */
	#define HWRM_ERR_CODE_SUCCESS				(UINT32_C(0x0))
	/* THe HWRM failed to execute the request. */
	#define HWRM_ERR_CODE_FAIL				(UINT32_C(0x1))
	/* The request contains invalid argument(s) or input parameters. */
	#define HWRM_ERR_CODE_INVALID_PARAMS			(UINT32_C(0x2))
	/*
	 * The requester is not allowed to access the requested
	 * resource. This error code shall be provided in a response to
	 * a request to query or modify an existing resource that is not
	 * accessible by the requester.
	 */
	#define HWRM_ERR_CODE_RESOURCE_ACCESS_DENIED		(UINT32_C(0x3))
	/*
	 * The HWRM is unable to allocate the requested resource. This
	 * code only applies to requests for HWRM resource allocations.
	 */
	#define HWRM_ERR_CODE_RESOURCE_ALLOC_ERROR		(UINT32_C(0x4))
	/* Invalid combination of flags is specified in the request. */
	#define HWRM_ERR_CODE_INVALID_FLAGS			(UINT32_C(0x5))
	/*
	 * Invalid combination of enables fields is specified in the
	 * request.
	 */
	#define HWRM_ERR_CODE_INVALID_ENABLES			(UINT32_C(0x6))
	/* Generic HWRM execution error that represents an internal error. */
	#define HWRM_ERR_CODE_HWRM_ERROR			(UINT32_C(0xf))
	/* Unknown error */
	#define HWRM_ERR_CODE_UNKNOWN_ERR			(UINT32_C(0xfffe))
	/* Unsupported or invalid command */
	#define HWRM_ERR_CODE_CMD_NOT_SUPPORTED		(UINT32_C(0xffff))
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_err_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t opaque_0;
	/* debug info for this error response. */
	uint16_t opaque_1;
	/* debug info for this error response. */
	uint8_t cmd_err;
	/*
	 * In the case of an error response, command specific error code is
	 * returned in this field.
	 */
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* Port Tx Statistics Formats (408 bytes) */

struct tx_port_stats {
	uint64_t tx_64b_frames;
	/* Total Number of 64 Bytes frames transmitted */
	uint64_t tx_65b_127b_frames;
	/* Total Number of 65-127 Bytes frames transmitted */
	uint64_t tx_128b_255b_frames;
	/* Total Number of 128-255 Bytes frames transmitted */
	uint64_t tx_256b_511b_frames;
	/* Total Number of 256-511 Bytes frames transmitted */
	uint64_t tx_512b_1023b_frames;
	/* Total Number of 512-1023 Bytes frames transmitted */
	uint64_t tx_1024b_1518_frames;
	/* Total Number of 1024-1518 Bytes frames transmitted */
	uint64_t tx_good_vlan_frames;
	/*
	 * Total Number of each good VLAN (exludes FCS errors) frame transmitted
	 * which is 1519 to 1522 bytes in length inclusive (excluding framing
	 * bits but including FCS bytes).
	 */
	uint64_t tx_1519b_2047_frames;
	/* Total Number of 1519-2047 Bytes frames transmitted */
	uint64_t tx_2048b_4095b_frames;
	/* Total Number of 2048-4095 Bytes frames transmitted */
	uint64_t tx_4096b_9216b_frames;
	/* Total Number of 4096-9216 Bytes frames transmitted */
	uint64_t tx_9217b_16383b_frames;
	/* Total Number of 9217-16383 Bytes frames transmitted */
	uint64_t tx_good_frames;
	/* Total Number of good frames transmitted */
	uint64_t tx_total_frames;
	/* Total Number of frames transmitted */
	uint64_t tx_ucast_frames;
	/* Total number of unicast frames transmitted */
	uint64_t tx_mcast_frames;
	/* Total number of multicast frames transmitted */
	uint64_t tx_bcast_frames;
	/* Total number of broadcast frames transmitted */
	uint64_t tx_pause_frames;
	/* Total number of PAUSE control frames transmitted */
	uint64_t tx_pfc_frames;
	/* Total number of PFC/per-priority PAUSE control frames transmitted */
	uint64_t tx_jabber_frames;
	/* Total number of jabber frames transmitted */
	uint64_t tx_fcs_err_frames;
	/* Total number of frames transmitted with FCS error */
	uint64_t tx_control_frames;
	/* Total number of control frames transmitted */
	uint64_t tx_oversz_frames;
	/* Total number of over-sized frames transmitted */
	uint64_t tx_single_dfrl_frames;
	/* Total number of frames with single deferral */
	uint64_t tx_multi_dfrl_frames;
	/* Total number of frames with multiple deferrals */
	uint64_t tx_single_coll_frames;
	/* Total number of frames with single collision */
	uint64_t tx_multi_coll_frames;
	/* Total number of frames with multiple collisions */
	uint64_t tx_late_coll_frames;
	/* Total number of frames with late collisions */
	uint64_t tx_excessive_coll_frames;
	/* Total number of frames with excessive collisions */
	uint64_t tx_frag_frames;
	/* Total number of fragmented frames transmitted */
	uint64_t tx_err;
	/* Total number of transmit errors */
	uint64_t tx_tagged_frames;
	/* Total number of single VLAN tagged frames transmitted */
	uint64_t tx_dbl_tagged_frames;
	/* Total number of double VLAN tagged frames transmitted */
	uint64_t tx_runt_frames;
	/* Total number of runt frames transmitted */
	uint64_t tx_fifo_underruns;
	/* Total number of TX FIFO under runs */
	uint64_t tx_pfc_ena_frames_pri0;
	/* Total number of PFC frames with PFC enabled bit for Pri 0 transmitted */
	uint64_t tx_pfc_ena_frames_pri1;
	/* Total number of PFC frames with PFC enabled bit for Pri 1 transmitted */
	uint64_t tx_pfc_ena_frames_pri2;
	/* Total number of PFC frames with PFC enabled bit for Pri 2 transmitted */
	uint64_t tx_pfc_ena_frames_pri3;
	/* Total number of PFC frames with PFC enabled bit for Pri 3 transmitted */
	uint64_t tx_pfc_ena_frames_pri4;
	/* Total number of PFC frames with PFC enabled bit for Pri 4 transmitted */
	uint64_t tx_pfc_ena_frames_pri5;
	/* Total number of PFC frames with PFC enabled bit for Pri 5 transmitted */
	uint64_t tx_pfc_ena_frames_pri6;
	/* Total number of PFC frames with PFC enabled bit for Pri 6 transmitted */
	uint64_t tx_pfc_ena_frames_pri7;
	/* Total number of PFC frames with PFC enabled bit for Pri 7 transmitted */
	uint64_t tx_eee_lpi_events;
	/* Total number of EEE LPI Events on TX */
	uint64_t tx_eee_lpi_duration;
	/* EEE LPI Duration Counter on TX */
	uint64_t tx_llfc_logical_msgs;
	/* Total number of Link Level Flow Control (LLFC) messages transmitted */
	uint64_t tx_hcfc_msgs;
	/* Total number of HCFC messages transmitted */
	uint64_t tx_total_collisions;
	/* Total number of TX collisions */
	uint64_t tx_bytes;
	/* Total number of transmitted bytes */
	uint64_t tx_xthol_frames;
	/* Total number of end-to-end HOL frames */
	uint64_t tx_stat_discard;
	/* Total Tx Drops per Port reported by STATS block */
	uint64_t tx_stat_error;
	/* Total Tx Error Drops per Port reported by STATS block */
} __attribute__((packed));

/* Port Rx Statistics Formats (528 bytes) */

struct rx_port_stats {
	uint64_t rx_64b_frames;
	/* Total Number of 64 Bytes frames received */
	uint64_t rx_65b_127b_frames;
	/* Total Number of 65-127 Bytes frames received */
	uint64_t rx_128b_255b_frames;
	/* Total Number of 128-255 Bytes frames received */
	uint64_t rx_256b_511b_frames;
	/* Total Number of 256-511 Bytes frames received */
	uint64_t rx_512b_1023b_frames;
	/* Total Number of 512-1023 Bytes frames received */
	uint64_t rx_1024b_1518_frames;
	/* Total Number of 1024-1518 Bytes frames received */
	uint64_t rx_good_vlan_frames;
	/*
	 * Total Number of each good VLAN (exludes FCS errors) frame received
	 * which is 1519 to 1522 bytes in length inclusive (excluding framing
	 * bits but including FCS bytes).
	 */
	uint64_t rx_1519b_2047b_frames;
	/* Total Number of 1519-2047 Bytes frames received */
	uint64_t rx_2048b_4095b_frames;
	/* Total Number of 2048-4095 Bytes frames received */
	uint64_t rx_4096b_9216b_frames;
	/* Total Number of 4096-9216 Bytes frames received */
	uint64_t rx_9217b_16383b_frames;
	/* Total Number of 9217-16383 Bytes frames received */
	uint64_t rx_total_frames;
	/* Total number of frames received */
	uint64_t rx_ucast_frames;
	/* Total number of unicast frames received */
	uint64_t rx_mcast_frames;
	/* Total number of multicast frames received */
	uint64_t rx_bcast_frames;
	/* Total number of broadcast frames received */
	uint64_t rx_fcs_err_frames;
	/* Total number of received frames with FCS error */
	uint64_t rx_ctrl_frames;
	/* Total number of control frames received */
	uint64_t rx_pause_frames;
	/* Total number of PAUSE frames received */
	uint64_t rx_pfc_frames;
	/* Total number of PFC frames received */
	uint64_t rx_unsupported_opcode_frames;
	/* Total number of frames received with an unsupported opcode */
	uint64_t rx_unsupported_da_pausepfc_frames;
	/*
	 * Total number of frames received with an unsupported DA for pause and
	 * PFC
	 */
	uint64_t rx_wrong_sa_frames;
	/* Total number of frames received with an unsupported SA */
	uint64_t rx_align_err_frames;
	/* Total number of received packets with alignment error */
	uint64_t rx_oor_len_frames;
	/* Total number of received frames with out-of-range length */
	uint64_t rx_code_err_frames;
	/* Total number of received frames with error termination */
	uint64_t rx_false_carrier_frames;
	/*
	 * Total number of received frames with a false carrier is detected
	 * during idle, as defined by RX_ER samples active and RXD is 0xE. The
	 * event is reported along with the statistics generated on the next
	 * received frame. Only one false carrier condition can be detected and
	 * logged between frames. Carrier event, valid for 10M/100M speed modes
	 * only.
	 */
	uint64_t rx_ovrsz_frames;
	/* Total number of over-sized frames received */
	uint64_t rx_jbr_frames;
	/* Total number of jabber packets received */
	uint64_t rx_mtu_err_frames;
	/* Total number of received frames with MTU error */
	uint64_t rx_match_crc_frames;
	/* Total number of received frames with CRC match */
	uint64_t rx_promiscuous_frames;
	/* Total number of frames received promiscuously */
	uint64_t rx_tagged_frames;
	/* Total number of received frames with one or two VLAN tags */
	uint64_t rx_double_tagged_frames;
	/* Total number of received frames with two VLAN tags */
	uint64_t rx_trunc_frames;
	/* Total number of truncated frames received */
	uint64_t rx_good_frames;
	/* Total number of good frames (without errors) received */
	uint64_t rx_pfc_xon2xoff_frames_pri0;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 0
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri1;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 1
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri2;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 2
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri3;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 3
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri4;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 4
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri5;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 5
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri6;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 6
	 */
	uint64_t rx_pfc_xon2xoff_frames_pri7;
	/*
	 * Total number of received PFC frames with transition from XON to XOFF
	 * on Pri 7
	 */
	uint64_t rx_pfc_ena_frames_pri0;
	/* Total number of received PFC frames with PFC enabled bit for Pri 0 */
	uint64_t rx_pfc_ena_frames_pri1;
	/* Total number of received PFC frames with PFC enabled bit for Pri 1 */
	uint64_t rx_pfc_ena_frames_pri2;
	/* Total number of received PFC frames with PFC enabled bit for Pri 2 */
	uint64_t rx_pfc_ena_frames_pri3;
	/* Total number of received PFC frames with PFC enabled bit for Pri 3 */
	uint64_t rx_pfc_ena_frames_pri4;
	/* Total number of received PFC frames with PFC enabled bit for Pri 4 */
	uint64_t rx_pfc_ena_frames_pri5;
	/* Total number of received PFC frames with PFC enabled bit for Pri 5 */
	uint64_t rx_pfc_ena_frames_pri6;
	/* Total number of received PFC frames with PFC enabled bit for Pri 6 */
	uint64_t rx_pfc_ena_frames_pri7;
	/* Total number of received PFC frames with PFC enabled bit for Pri 7 */
	uint64_t rx_sch_crc_err_frames;
	/* Total Number of frames received with SCH CRC error */
	uint64_t rx_undrsz_frames;
	/* Total Number of under-sized frames received */
	uint64_t rx_frag_frames;
	/* Total Number of fragmented frames received */
	uint64_t rx_eee_lpi_events;
	/* Total number of RX EEE LPI Events */
	uint64_t rx_eee_lpi_duration;
	/* EEE LPI Duration Counter on RX */
	uint64_t rx_llfc_physical_msgs;
	/*
	 * Total number of physical type Link Level Flow Control (LLFC) messages
	 * received
	 */
	uint64_t rx_llfc_logical_msgs;
	/*
	 * Total number of logical type Link Level Flow Control (LLFC) messages
	 * received
	 */
	uint64_t rx_llfc_msgs_with_crc_err;
	/*
	 * Total number of logical type Link Level Flow Control (LLFC) messages
	 * received with CRC error
	 */
	uint64_t rx_hcfc_msgs;
	/* Total number of HCFC messages received */
	uint64_t rx_hcfc_msgs_with_crc_err;
	/* Total number of HCFC messages received with CRC error */
	uint64_t rx_bytes;
	/* Total number of received bytes */
	uint64_t rx_runt_bytes;
	/* Total number of bytes received in runt frames */
	uint64_t rx_runt_frames;
	/* Total number of runt frames received */
	uint64_t rx_stat_discard;
	/* Total Rx Discards per Port reported by STATS block */
	uint64_t rx_stat_err;
	/* Total Rx Error Drops per Port reported by STATS block */
} __attribute__((packed));

/* hwrm_ver_get */
/*
 * Description: This function is called by a driver to determine the HWRM
 * interface version supported by the HWRM firmware, the version of HWRM
 * firmware implementation, the name of HWRM firmware, the versions of other
 * embedded firmwares, and the names of other embedded firmwares, etc. Any
 * interface or firmware version with major = 0, minor = 0, and update = 0 shall
 * be considered an invalid version.
 */
/* Input (24 bytes) */

struct hwrm_ver_get_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t hwrm_intf_maj;
	/*
	 * This field represents the major version of HWRM interface
	 * specification supported by the driver HWRM implementation. The
	 * interface major version is intended to change only when non backward
	 * compatible changes are made to the HWRM interface specification.
	 */
	uint8_t hwrm_intf_min;
	/*
	 * This field represents the minor version of HWRM interface
	 * specification supported by the driver HWRM implementation. A change
	 * in interface minor version is used to reflect significant backward
	 * compatible modification to HWRM interface specification. This can be
	 * due to addition or removal of functionality. HWRM interface
	 * specifications with the same major version but different minor
	 * versions are compatible.
	 */
	uint8_t hwrm_intf_upd;
	/*
	 * This field represents the update version of HWRM interface
	 * specification supported by the driver HWRM implementation. The
	 * interface update version is used to reflect minor changes or bug
	 * fixes to a released HWRM interface specification.
	 */
	uint8_t unused_0[5];
} __attribute__((packed));

/* Output (128 bytes) */

struct hwrm_ver_get_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t hwrm_intf_maj;
	/*
	 * This field represents the major version of HWRM interface
	 * specification supported by the HWRM implementation. The interface
	 * major version is intended to change only when non backward compatible
	 * changes are made to the HWRM interface specification. A HWRM
	 * implementation that is compliant with this specification shall
	 * provide value of 1 in this field.
	 */
	uint8_t hwrm_intf_min;
	/*
	 * This field represents the minor version of HWRM interface
	 * specification supported by the HWRM implementation. A change in
	 * interface minor version is used to reflect significant backward
	 * compatible modification to HWRM interface specification. This can be
	 * due to addition or removal of functionality. HWRM interface
	 * specifications with the same major version but different minor
	 * versions are compatible. A HWRM implementation that is compliant with
	 * this specification shall provide value of 2 in this field.
	 */
	uint8_t hwrm_intf_upd;
	/*
	 * This field represents the update version of HWRM interface
	 * specification supported by the HWRM implementation. The interface
	 * update version is used to reflect minor changes or bug fixes to a
	 * released HWRM interface specification. A HWRM implementation that is
	 * compliant with this specification shall provide value of 2 in this
	 * field.
	 */
	uint8_t hwrm_intf_rsvd;
	uint8_t hwrm_fw_maj;
	/*
	 * This field represents the major version of HWRM firmware. A change in
	 * firmware major version represents a major firmware release.
	 */
	uint8_t hwrm_fw_min;
	/*
	 * This field represents the minor version of HWRM firmware. A change in
	 * firmware minor version represents significant firmware functionality
	 * changes.
	 */
	uint8_t hwrm_fw_bld;
	/*
	 * This field represents the build version of HWRM firmware. A change in
	 * firmware build version represents bug fixes to a released firmware.
	 */
	uint8_t hwrm_fw_rsvd;
	/*
	 * This field is a reserved field. This field can be used to represent
	 * firmware branches or customer specific releases tied to a specific
	 * (major,minor,update) version of the HWRM firmware.
	 */
	uint8_t mgmt_fw_maj;
	/*
	 * This field represents the major version of mgmt firmware. A change in
	 * major version represents a major release.
	 */
	uint8_t mgmt_fw_min;
	/*
	 * This field represents the minor version of mgmt firmware. A change in
	 * minor version represents significant functionality changes.
	 */
	uint8_t mgmt_fw_bld;
	/*
	 * This field represents the build version of mgmt firmware. A change in
	 * update version represents bug fixes.
	 */
	uint8_t mgmt_fw_rsvd;
	/*
	 * This field is a reserved field. This field can be used to represent
	 * firmware branches or customer specific releases tied to a specific
	 * (major,minor,update) version
	 */
	uint8_t netctrl_fw_maj;
	/*
	 * This field represents the major version of network control firmware.
	 * A change in major version represents a major release.
	 */
	uint8_t netctrl_fw_min;
	/*
	 * This field represents the minor version of network control firmware.
	 * A change in minor version represents significant functionality
	 * changes.
	 */
	uint8_t netctrl_fw_bld;
	/*
	 * This field represents the build version of network control firmware.
	 * A change in update version represents bug fixes.
	 */
	uint8_t netctrl_fw_rsvd;
	/*
	 * This field is a reserved field. This field can be used to represent
	 * firmware branches or customer specific releases tied to a specific
	 * (major,minor,update) version
	 */
	uint32_t reserved1;
	/*
	 * This field is reserved for future use. The responder should set it to
	 * 0. The requester should ignore this field.
	 */
	uint8_t roce_fw_maj;
	/*
	 * This field represents the major version of RoCE firmware. A change in
	 * major version represents a major release.
	 */
	uint8_t roce_fw_min;
	/*
	 * This field represents the minor version of RoCE firmware. A change in
	 * minor version represents significant functionality changes.
	 */
	uint8_t roce_fw_bld;
	/*
	 * This field represents the build version of RoCE firmware. A change in
	 * update version represents bug fixes.
	 */
	uint8_t roce_fw_rsvd;
	/*
	 * This field is a reserved field. This field can be used to represent
	 * firmware branches or customer specific releases tied to a specific
	 * (major,minor,update) version
	 */
	char hwrm_fw_name[16];
	/*
	 * This field represents the name of HWRM FW (ASCII chars with NULL at
	 * the end).
	 */
	char mgmt_fw_name[16];
	/*
	 * This field represents the name of mgmt FW (ASCII chars with NULL at
	 * the end).
	 */
	char netctrl_fw_name[16];
	/*
	 * This field represents the name of network control firmware (ASCII
	 * chars with NULL at the end).
	 */
	uint32_t reserved2[4];
	/*
	 * This field is reserved for future use. The responder should set it to
	 * 0. The requester should ignore this field.
	 */
	char roce_fw_name[16];
	/*
	 * This field represents the name of RoCE FW (ASCII chars with NULL at
	 * the end).
	 */
	uint16_t chip_num;
	/* This field returns the chip number. */
	uint8_t chip_rev;
	/* This field returns the revision of chip. */
	uint8_t chip_metal;
	/* This field returns the chip metal number. */
	uint8_t chip_bond_id;
	/* This field returns the bond id of the chip. */
	uint8_t chip_platform_type;
	/*
	 * This value indicates the type of platform used for chip
	 * implementation.
	 */
	/* ASIC */
	#define HWRM_VER_GET_OUTPUT_CHIP_PLATFORM_TYPE_ASIC	(UINT32_C(0x0) << 0)
	/* FPGA platform of the chip. */
	#define HWRM_VER_GET_OUTPUT_CHIP_PLATFORM_TYPE_FPGA	(UINT32_C(0x1) << 0)
	/* Palladium platform of the chip. */
	#define HWRM_VER_GET_OUTPUT_CHIP_PLATFORM_TYPE_PALLADIUM  (UINT32_C(0x2) << 0)
	uint16_t max_req_win_len;
	/*
	 * This field returns the maximum value of request window that is
	 * supported by the HWRM. The request window is mapped into device
	 * address space using MMIO.
	 */
	uint16_t max_resp_len;
	/* This field returns the maximum value of response buffer in bytes. */
	uint16_t def_req_timeout;
	/* This field returns the default request timeout value in milliseconds. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_reset */
/*
 * Description: This command resets a hardware function (PCIe function) and
 * frees any resources used by the function. This command shall be initiated by
 * the driver after an FLR has occurred to prepare the function for re-use. This
 * command may also be initiated by a driver prior to doing it's own
 * configuration. This command puts the function into the reset state. In the
 * reset state, global and port related features of the chip are not available.
 */
/*
 * Note: This command will reset a function that has already been disabled or
 * idled. The command returns all the resources owned by the function so a new
 * driver may allocate and configure resources normally.
 */
/* Input (24 bytes) */

struct hwrm_func_reset_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the vf_id_valid field to be configured. */
	#define HWRM_FUNC_RESET_INPUT_ENABLES_VF_ID_VALID	UINT32_C(0x1)
	uint16_t vf_id;
	/*
	 * The ID of the VF that this PF is trying to reset. Only the parent PF
	 * shall be allowed to reset a child VF. A parent PF driver shall use
	 * this field only when a specific child VF is requested to be reset.
	 */
	uint8_t func_reset_level;
	/* This value indicates the level of a function reset. */
	/*
	 * Reset the caller function and its children VFs (if any). If
	 * no children functions exist, then reset the caller function
	 * only.
	 */
	#define HWRM_FUNC_RESET_INPUT_FUNC_RESET_LEVEL_RESETALL   (UINT32_C(0x0) << 0)
	/* Reset the caller function only */
	#define HWRM_FUNC_RESET_INPUT_FUNC_RESET_LEVEL_RESETME	(UINT32_C(0x1) << 0)
	/*
	 * Reset all children VFs of the caller function driver if the
	 * caller is a PF driver. It is an error to specify this level
	 * by a VF driver. It is an error to specify this level by a PF
	 * driver with no children VFs.
	 */
	#define HWRM_FUNC_RESET_INPUT_FUNC_RESET_LEVEL_RESETCHILDREN (UINT32_C(0x2) << 0)
	/*
	 * Reset a specific VF of the caller function driver if the
	 * caller is the parent PF driver. It is an error to specify
	 * this level by a VF driver. It is an error to specify this
	 * level by a PF driver that is not the parent of the VF that is
	 * being requested to reset.
	 */
	#define HWRM_FUNC_RESET_INPUT_FUNC_RESET_LEVEL_RESETVF	(UINT32_C(0x3) << 0)
	uint8_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_reset_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_getfid */
/*
 * Description: This command returns the FID value for the function. If a valid
 * pci_id is provided, then this function returns fid for that PCI function.
 * Otherwise, it returns FID of the requesting function. This value is needed to
 * configure Rings and MSI-X vectors so their DMA operations appear correctly on
 * the PCI bus. For PF-HWRM commands, there is no need for FID. Similarly there
 * is no need for FID for VF-HWRM commands. In the PF-VF communication, only PF
 * needs to know FIDs.
 */
/* Input (24 bytes) */

struct hwrm_func_getfid_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the pci_id field to be configured. */
	#define HWRM_FUNC_GETFID_INPUT_ENABLES_PCI_ID		UINT32_C(0x1)
	uint16_t pci_id;
	/*
	 * This value is the PCI ID of the queried function. If ARI is enabled,
	 * then it is Bus Number (8b):Function Number(8b). Otherwise, it is Bus
	 * Number (8b):Device Number (4b):Function Number(4b).
	 */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_getfid_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t fid;
	/*
	 * FID value. This value is used to identify operations on the PCI bus
	 * as belonging to a particular PCI function.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_vf_alloc */
/*
 * Description: This command is used to allocate requested number of virtual
 * functions on a physical function. It will return the FID value of the first
 * virtual function. The FIDs of the remaining virtual functions can be derived
 * by sequentially incrementing the FID value of the first VF. This command
 * supports the following models for VF allocation: # Allocation of one or more
 * VFs from a PF without specifying the first VF ID # Allocation of multiple VFs
 * from a PF starting with a specific VF # Allocation of a specific VF from a PF
 * If this command is called on a virtual function or a physical function that
 * is not enabled for SR-IOV, the HWRM shall return an error. The VF IDs
 * returned by this function remain valid after the VF is disabled or reset.
 */
/* Input (24 bytes) */

struct hwrm_func_vf_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the first_vf_id field to be configured. */
	#define HWRM_FUNC_VF_ALLOC_INPUT_ENABLES_FIRST_VF_ID	UINT32_C(0x1)
	uint16_t first_vf_id;
	/*
	 * This value is used to identify a Virtual Function (VF). The scope of
	 * VF ID is local within a PF.
	 */
	uint16_t num_vfs;
	/* The number of virtual functions requested. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_vf_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t first_vf_id;
	/* The ID of the first VF allocated. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_vf_free */
/*
 * Description: This command will free allocated virtual functions on a physical
 * function. If this command is called on a virtual function or a physical
 * function that is not enabled for SR-IOV, the HWRM shall return an error.
 * After the successful completion of this command, the VF IDs of the VFs that
 * are freed on the PF are invalid.
 */
/* Input (24 bytes) */

struct hwrm_func_vf_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the first_vf_id field to be configured. */
	#define HWRM_FUNC_VF_FREE_INPUT_ENABLES_FIRST_VF_ID	UINT32_C(0x1)
	uint16_t first_vf_id;
	/*
	 * This value is used to identify a Virtual Function (VF). The scope of
	 * VF ID is local within a PF.
	 */
	uint16_t num_vfs;
	/*
	 * The number of virtual functions requested. 0xFFFF - Cleanup all
	 * children of this PF.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_vf_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_vf_cfg */
/*
 * Description: This command allows configuration of a VF by its driver. If this
 * function is called by a PF driver, then the HWRM shall fail this command. If
 * guest VLAN and/or MAC address are provided in this command, then the HWRM
 * shall set up appropriate MAC/VLAN filters for the VF that is being
 * configured.
 */
/* Input (32 bytes) */

struct hwrm_func_vf_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the mtu field to be configured. */
	#define HWRM_FUNC_VF_CFG_INPUT_ENABLES_MTU		UINT32_C(0x1)
	/* This bit must be '1' for the guest_vlan field to be configured. */
	#define HWRM_FUNC_VF_CFG_INPUT_ENABLES_GUEST_VLAN	UINT32_C(0x2)
	/* This bit must be '1' for the async_event_cr field to be configured. */
	#define HWRM_FUNC_VF_CFG_INPUT_ENABLES_ASYNC_EVENT_CR	UINT32_C(0x4)
	/* This bit must be '1' for the dflt_mac_addr field to be configured. */
	#define HWRM_FUNC_VF_CFG_INPUT_ENABLES_DFLT_MAC_ADDR	UINT32_C(0x8)
	uint16_t mtu;
	/*
	 * The maximum transmission unit requested on the function. The HWRM
	 * should make sure that the mtu of the function does not exceed the mtu
	 * of the physical port that this function is associated with. In
	 * addition to requesting mtu per function, it is possible to configure
	 * mtu per transmit ring. By default, the mtu of each transmit ring
	 * associated with a function is equal to the mtu of the function. The
	 * HWRM should make sure that the mtu of each transmit ring that is
	 * assigned to a function has a valid mtu.
	 */
	uint16_t guest_vlan;
	/*
	 * The guest VLAN for the function being configured. This VLAN is in
	 * 802.1Q tag format.
	 */
	uint16_t async_event_cr;
	/*
	 * ID of the target completion ring for receiving asynchronous event
	 * completions. If this field is not valid, then the HWRM shall use the
	 * default completion ring of the function that is being configured as
	 * the target completion ring for providing any asynchronous event
	 * completions for that function. If this field is valid, then the HWRM
	 * shall use the completion ring identified by this ID as the target
	 * completion ring for providing any asynchronous event completions for
	 * the function that is being configured.
	 */
	uint8_t dflt_mac_addr[6];
	/*
	 * This value is the current MAC address requested by the VF driver to
	 * be configured on this VF. A value of 00-00-00-00-00-00 indicates no
	 * MAC address configuration is requested by the VF driver. The parent
	 * PF driver may reject or overwrite this MAC address.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_vf_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_qcaps */
/*
 * Description: This command returns capabilities of a function. The input FID
 * value is used to indicate what function is being queried. This allows a
 * physical function driver to query virtual functions that are children of the
 * physical function. The output FID value is needed to configure Rings and
 * MSI-X vectors so their DMA operations appear correctly on the PCI bus.
 */
/* Input (24 bytes) */

struct hwrm_func_qcaps_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t fid;
	/*
	 * Function ID of the function that is being queried. 0xFF... (All Fs)
	 * if the query is for the requesting function.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (80 bytes) */

struct hwrm_func_qcaps_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t fid;
	/*
	 * FID value. This value is used to identify operations on the PCI bus
	 * as belonging to a particular PCI function.
	 */
	uint16_t port_id;
	/*
	 * Port ID of port that this function is associated with. Valid only for
	 * the PF. 0xFF... (All Fs) if this function is not associated with any
	 * port. 0xFF... (All Fs) if this function is called from a VF.
	 */
	uint32_t flags;
	/* If 1, then Push mode is supported on this function. */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_PUSH_MODE_SUPPORTED   UINT32_C(0x1)
	/* If 1, then the global MSI-X auto-masking is enabled for the device. */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_GLOBAL_MSIX_AUTOMASKING UINT32_C(0x2)
	/*
	 * If 1, then the Precision Time Protocol (PTP) processing is supported
	 * on this function. The HWRM should enable PTP on only a single
	 * Physical Function (PF) per port.
	 */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_PTP_SUPPORTED	UINT32_C(0x4)
	/*
	 * If 1, then RDMA over Converged Ethernet (RoCE) v1 is supported on
	 * this function.
	 */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_ROCE_V1_SUPPORTED	UINT32_C(0x8)
	/*
	 * If 1, then RDMA over Converged Ethernet (RoCE) v2 is supported on
	 * this function.
	 */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_ROCE_V2_SUPPORTED	UINT32_C(0x10)
	/*
	 * If 1, then control and configuration of WoL magic packet is supported
	 * on this function.
	 */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_WOL_MAGICPKT_SUPPORTED UINT32_C(0x20)
	/*
	 * If 1, then control and configuration of bitmap pattern packet is
	 * supported on this function.
	 */
	#define HWRM_FUNC_QCAPS_OUTPUT_FLAGS_WOL_BMP_SUPPORTED	UINT32_C(0x40)
	uint8_t mac_address[6];
	/*
	 * This value is current MAC address configured for this function. A
	 * value of 00-00-00-00-00-00 indicates no MAC address is currently
	 * configured.
	 */
	uint16_t max_rsscos_ctx;
	/*
	 * The maximum number of RSS/COS contexts that can be allocated to the
	 * function.
	 */
	uint16_t max_cmpl_rings;
	/*
	 * The maximum number of completion rings that can be allocated to the
	 * function.
	 */
	uint16_t max_tx_rings;
	/*
	 * The maximum number of transmit rings that can be allocated to the
	 * function.
	 */
	uint16_t max_rx_rings;
	/*
	 * The maximum number of receive rings that can be allocated to the
	 * function.
	 */
	uint16_t max_l2_ctxs;
	/*
	 * The maximum number of L2 contexts that can be allocated to the
	 * function.
	 */
	uint16_t max_vnics;
	/* The maximum number of VNICs that can be allocated to the function. */
	uint16_t first_vf_id;
	/*
	 * The identifier for the first VF enabled on a PF. This is valid only
	 * on the PF with SR-IOV enabled. 0xFF... (All Fs) if this command is
	 * called on a PF with SR-IOV disabled or on a VF.
	 */
	uint16_t max_vfs;
	/*
	 * The maximum number of VFs that can be allocated to the function. This
	 * is valid only on the PF with SR-IOV enabled. 0xFF... (All Fs) if this
	 * command is called on a PF with SR-IOV disabled or on a VF.
	 */
	uint16_t max_stat_ctx;
	/*
	 * The maximum number of statistic contexts that can be allocated to the
	 * function.
	 */
	uint32_t max_encap_records;
	/*
	 * The maximum number of Encapsulation records that can be offloaded by
	 * this function.
	 */
	uint32_t max_decap_records;
	/*
	 * The maximum number of decapsulation records that can be offloaded by
	 * this function.
	 */
	uint32_t max_tx_em_flows;
	/*
	 * The maximum number of Exact Match (EM) flows that can be offloaded by
	 * this function on the TX side.
	 */
	uint32_t max_tx_wm_flows;
	/*
	 * The maximum number of Wildcard Match (WM) flows that can be offloaded
	 * by this function on the TX side.
	 */
	uint32_t max_rx_em_flows;
	/*
	 * The maximum number of Exact Match (EM) flows that can be offloaded by
	 * this function on the RX side.
	 */
	uint32_t max_rx_wm_flows;
	/*
	 * The maximum number of Wildcard Match (WM) flows that can be offloaded
	 * by this function on the RX side.
	 */
	uint32_t max_mcast_filters;
	/*
	 * The maximum number of multicast filters that can be supported by this
	 * function on the RX side.
	 */
	uint32_t max_flow_id;
	/*
	 * The maximum value of flow_id that can be supported in completion
	 * records.
	 */
	uint32_t max_hw_ring_grps;
	/*
	 * The maximum number of HW ring groups that can be supported on this
	 * function.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_qcfg */
/*
 * Description: This command returns the current configuration of a function.
 * The input FID value is used to indicate what function is being queried. This
 * allows a physical function driver to query virtual functions that are
 * children of the physical function. The output FID value is needed to
 * configure Rings and MSI-X vectors so their DMA operations appear correctly on
 * the PCI bus.
 */
/* Input (24 bytes) */

struct hwrm_func_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t fid;
	/*
	 * Function ID of the function that is being queried. 0xFF... (All Fs)
	 * if the query is for the requesting function.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (72 bytes) */

struct hwrm_func_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t fid;
	/*
	 * FID value. This value is used to identify operations on the PCI bus
	 * as belonging to a particular PCI function.
	 */
	uint16_t port_id;
	/*
	 * Port ID of port that this function is associated with. 0xFF... (All
	 * Fs) if this function is not associated with any port.
	 */
	uint16_t vlan;
	/*
	 * This value is the current VLAN setting for this function. The value
	 * of 0 for this field indicates no priority tagging or VLAN is used.
	 * This VLAN is in 802.1Q tag format.
	 */
	uint16_t flags;
	/*
	 * If 1, then magic packet based Out-Of-Box WoL is enabled on the port
	 * associated with this function.
	 */
	#define HWRM_FUNC_QCFG_OUTPUT_FLAGS_OOB_WOL_MAGICPKT_ENABLED UINT32_C(0x1)
	/*
	 * If 1, then bitmap pattern based Out-Of-Box WoL packet is enabled on
	 * the port associated with this function.
	 */
	#define HWRM_FUNC_QCFG_OUTPUT_FLAGS_OOB_WOL_BMP_ENABLED	UINT32_C(0x2)
	uint8_t mac_address[6];
	/*
	 * This value is current MAC address configured for this function. A
	 * value of 00-00-00-00-00-00 indicates no MAC address is currently
	 * configured.
	 */
	uint16_t pci_id;
	/*
	 * This value is current PCI ID of this function. If ARI is enabled,
	 * then it is Bus Number (8b):Function Number(8b). Otherwise, it is Bus
	 * Number (8b):Device Number (4b):Function Number(4b).
	 */
	uint16_t alloc_rsscos_ctx;
	/* The number of RSS/COS contexts currently allocated to the function. */
	uint16_t alloc_cmpl_rings;
	/*
	 * The number of completion rings currently allocated to the function.
	 * This does not include the rings allocated to any children functions
	 * if any.
	 */
	uint16_t alloc_tx_rings;
	/*
	 * The number of transmit rings currently allocated to the function.
	 * This does not include the rings allocated to any children functions
	 * if any.
	 */
	uint16_t alloc_rx_rings;
	/*
	 * The number of receive rings currently allocated to the function. This
	 * does not include the rings allocated to any children functions if
	 * any.
	 */
	uint16_t alloc_l2_ctx;
	/* The allocated number of L2 contexts to the function. */
	uint16_t alloc_vnics;
	/* The allocated number of vnics to the function. */
	uint16_t mtu;
	/*
	 * The maximum transmission unit of the function. For rings allocated on
	 * this function, this default value is used if ring MTU is not
	 * specified.
	 */
	uint16_t mru;
	/*
	 * The maximum receive unit of the function. For vnics allocated on this
	 * function, this default value is used if vnic MRU is not specified.
	 */
	uint16_t stat_ctx_id;
	/* The statistics context assigned to a function. */
	uint8_t port_partition_type;
	/*
	 * The HWRM shall return Unknown value for this field when this command
	 * is used to query VF's configuration.
	 */
	/* Single physical function */
	#define HWRM_FUNC_QCFG_OUTPUT_PORT_PARTITION_TYPE_SPF	(UINT32_C(0x0) << 0)
	/* Multiple physical functions */
	#define HWRM_FUNC_QCFG_OUTPUT_PORT_PARTITION_TYPE_MPFS	(UINT32_C(0x1) << 0)
	/* Network Partitioning 1.0 */
	#define HWRM_FUNC_QCFG_OUTPUT_PORT_PARTITION_TYPE_NPAR1_0 (UINT32_C(0x2) << 0)
	/* Network Partitioning 1.5 */
	#define HWRM_FUNC_QCFG_OUTPUT_PORT_PARTITION_TYPE_NPAR1_5 (UINT32_C(0x3) << 0)
	/* Network Partitioning 2.0 */
	#define HWRM_FUNC_QCFG_OUTPUT_PORT_PARTITION_TYPE_NPAR2_0 (UINT32_C(0x4) << 0)
	/* Unknown */
	#define HWRM_FUNC_QCFG_OUTPUT_PORT_PARTITION_TYPE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t unused_0;
	uint16_t dflt_vnic_id;
	/* The default VNIC ID assigned to a function that is being queried. */
	uint8_t unused_1;
	uint8_t unused_2;
	uint32_t min_bw;
	/*
	 * Minimum BW allocated for this function in Mbps. The HWRM will
	 * translate this value into byte counter and time interval used for the
	 * scheduler inside the device. A value of 0 indicates the minimum
	 * bandwidth is not configured.
	 */
	uint32_t max_bw;
	/*
	 * Maximum BW allocated for this function in Mbps. The HWRM will
	 * translate this value into byte counter and time interval used for the
	 * scheduler inside the device. A value of 0 indicates that the maximum
	 * bandwidth is not configured.
	 */
	uint8_t evb_mode;
	/*
	 * This value indicates the Edge virtual bridge mode for the domain that
	 * this function belongs to.
	 */
	/* No Edge Virtual Bridging (EVB) */
	#define HWRM_FUNC_QCFG_OUTPUT_EVB_MODE_NO_EVB		(UINT32_C(0x0) << 0)
	/* Virtual Ethernet Bridge (VEB) */
	#define HWRM_FUNC_QCFG_OUTPUT_EVB_MODE_VEB		(UINT32_C(0x1) << 0)
	/* Virtual Ethernet Port Aggregator (VEPA) */
	#define HWRM_FUNC_QCFG_OUTPUT_EVB_MODE_VEPA		(UINT32_C(0x2) << 0)
	uint8_t unused_3;
	uint16_t unused_4;
	uint32_t alloc_mcast_filters;
	/*
	 * The number of allocated multicast filters for this function on the RX
	 * side.
	 */
	uint32_t alloc_hw_ring_grps;
	/* The number of allocated HW ring groups for this function. */
	uint8_t unused_5;
	uint8_t unused_6;
	uint8_t unused_7;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_cfg */
/*
 * Description: This command allows configuration of a PF by the corresponding
 * PF driver. This command also allows configuration of a child VF by its parent
 * PF driver. The input FID value is used to indicate what function is being
 * configured. This allows a PF driver to configure the PF owned by itself or a
 * virtual function that is a child of the PF. This command allows to reserve
 * resources for a VF by its parent PF. To reverse the process, the command
 * should be called with all enables flags cleared for resources. This will free
 * allocated resources for the VF and return them to the resource pool. If this
 * function is called by a VF driver to configure or reserve resources, then the
 * HWRM shall fail this command. If default MAC address and/or VLAN are provided
 * in this command, then the HWRM shall set up appropriate MAC/VLAN filters for
 * the function that is being configured. If source properties checks are
 * enabled and default MAC address and/or IP address are provided in this
 * command, then the HWRM shall set appropriate source property checks based on
 * provided MAC and/or IP addresses.
 */
/* Input (88 bytes) */

struct hwrm_func_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t fid;
	/*
	 * Function ID of the function that is being configured. If set to
	 * 0xFF... (All Fs), then the the configuration is for the requesting
	 * function.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint32_t flags;
	/*
	 * When this bit is '1', the function is requested to be put in the
	 * promiscuous mode.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_PROM_MODE		UINT32_C(0x1)
	/*
	 * When this bit is '1', the function is enabled with source MAC address
	 * check. This is an anti-spoofing check. If this flag is set, then the
	 * function shall be configured to allow transmission of frames with the
	 * source MAC address that is configured for this function.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_SRC_MAC_ADDR_CHECK	UINT32_C(0x2)
	/*
	 * When this bit is '1', the function is enabled with source IP address
	 * check. This is an anti-spoofing check. If this flag is set, then the
	 * function shall be configured to allow transmission of frames with the
	 * source IP address that is configured for this function.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_SRC_IP_ADDR_CHECK	UINT32_C(0x4)
	/*
	 * When this bit is set to '1', the function shall be configured with
	 * VLAN priority match. If the VLAN PRI of a packet originated from this
	 * function does not match, then the packet shall be discarded.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_VLAN_PRI_MATCH	UINT32_C(0x8)
	/*
	 * When this bit is set to '1', the function shall be configured to
	 * check for VLAN priority match. If the VLAN PRI of a packet originated
	 * from this function does not match, then the default VLAN PRI shall be
	 * used.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_DFLT_PRI_NOMATCH	UINT32_C(0x10)
	/*
	 * When this bit is set to '1', the function shall be configured to not
	 * allow the transmission of pause frames. PAUSE frames use 48-bit
	 * destination multicast MAC address 01-80-C2-00-00-01.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_DISABLE_PAUSE		UINT32_C(0x20)
	/*
	 * When this bit is set to '1', the function shall be configured to not
	 * allow the transmission of Spanning Tree Protocol (STP) frames. STP
	 * frames use Ethertype 0x0802 and 48-bit destination multicast MAC
	 * address 01-80-C2-00-00-00 and 01-80-C2-00-00-08 for 802.1D and
	 * 802.1ad respectively.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_DISABLE_STP		UINT32_C(0x40)
	/*
	 * When this bit is set to '1', the function shall be configured to not
	 * allow the transmission of Link Layer Discovery Protocol (LLDP)
	 * frames. LLDP frames use Ethertype 0x88CC and 48-bit destination
	 * multicast MAC address 01-80-C2-00-00-00 or 01-80-C2-00-00-03 or
	 * 01-80-C2-00-00-0E.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_DISABLE_LLDP		UINT32_C(0x80)
	/*
	 * When this bit is set to '1', the function shall be configured to not
	 * allow the transmission of Precision Time Protocol (PTP) v2 frames.
	 * PTP frames use Ethertype 0x88F7 and 48-bit destination multicast MAC
	 * address 01-80-C2-00-00-0E or 01-1B-19-00-00-00.
	 */
	#define HWRM_FUNC_CFG_INPUT_FLAGS_DISABLE_PTPV2		UINT32_C(0x100)
	uint32_t enables;
	/* This bit must be '1' for the mtu field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_MTU			UINT32_C(0x1)
	/* This bit must be '1' for the mru field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_MRU			UINT32_C(0x2)
	/* This bit must be '1' for the num_rsscos_ctxs field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_RSSCOS_CTXS	UINT32_C(0x4)
	/* This bit must be '1' for the num_cmpl_rings field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_CMPL_RINGS	UINT32_C(0x8)
	/* This bit must be '1' for the num_tx_rings field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_TX_RINGS	UINT32_C(0x10)
	/* This bit must be '1' for the num_rx_rings field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_RX_RINGS	UINT32_C(0x20)
	/* This bit must be '1' for the num_l2_ctxs field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_L2_CTXS		UINT32_C(0x40)
	/* This bit must be '1' for the num_vnics field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_VNICS		UINT32_C(0x80)
	/* This bit must be '1' for the num_stat_ctxs field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_STAT_CTXS	UINT32_C(0x100)
	/* This bit must be '1' for the dflt_mac_addr field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_DFLT_MAC_ADDR	UINT32_C(0x200)
	/* This bit must be '1' for the dflt_vlan field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_DFLT_VLAN		UINT32_C(0x400)
	/* This bit must be '1' for the dflt_ip_addr field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_DFLT_IP_ADDR	UINT32_C(0x800)
	/* This bit must be '1' for the min_bw field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_MIN_BW		UINT32_C(0x1000)
	/* This bit must be '1' for the max_bw field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_MAX_BW		UINT32_C(0x2000)
	/* This bit must be '1' for the async_event_cr field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_ASYNC_EVENT_CR	UINT32_C(0x4000)
	/*
	 * This bit must be '1' for the vlan_antispoof_mode field to be
	 * configured.
	 */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_VLAN_ANTISPOOF_MODE	UINT32_C(0x8000)
	/*
	 * This bit must be '1' for the allowed_vlan_pris field to be
	 * configured.
	 */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_ALLOWED_VLAN_PRIS	UINT32_C(0x10000)
	/* This bit must be '1' for the evb_mode field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_EVB_MODE		UINT32_C(0x20000)
	/*
	 * This bit must be '1' for the num_mcast_filters field to be
	 * configured.
	 */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_MCAST_FILTERS	UINT32_C(0x40000)
	/* This bit must be '1' for the num_hw_ring_grps field to be configured. */
	#define HWRM_FUNC_CFG_INPUT_ENABLES_NUM_HW_RING_GRPS	UINT32_C(0x80000)
	uint16_t mtu;
	/*
	 * The maximum transmission unit of the function. The HWRM should make
	 * sure that the mtu of the function does not exceed the mtu of the
	 * physical port that this function is associated with. In addition to
	 * configuring mtu per function, it is possible to configure mtu per
	 * transmit ring. By default, the mtu of each transmit ring associated
	 * with a function is equal to the mtu of the function. The HWRM should
	 * make sure that the mtu of each transmit ring that is assigned to a
	 * function has a valid mtu.
	 */
	uint16_t mru;
	/*
	 * The maximum receive unit of the function. The HWRM should make sure
	 * that the mru of the function does not exceed the mru of the physical
	 * port that this function is associated with. In addition to
	 * configuring mru per function, it is possible to configure mru per
	 * vnic. By default, the mru of each vnic associated with a function is
	 * equal to the mru of the function. The HWRM should make sure that the
	 * mru of each vnic that is assigned to a function has a valid mru.
	 */
	uint16_t num_rsscos_ctxs;
	/* The number of RSS/COS contexts requested for the function. */
	uint16_t num_cmpl_rings;
	/*
	 * The number of completion rings requested for the function. This does
	 * not include the rings allocated to any children functions if any.
	 */
	uint16_t num_tx_rings;
	/*
	 * The number of transmit rings requested for the function. This does
	 * not include the rings allocated to any children functions if any.
	 */
	uint16_t num_rx_rings;
	/*
	 * The number of receive rings requested for the function. This does not
	 * include the rings allocated to any children functions if any.
	 */
	uint16_t num_l2_ctxs;
	/* The requested number of L2 contexts for the function. */
	uint16_t num_vnics;
	/* The requested number of vnics for the function. */
	uint16_t num_stat_ctxs;
	/* The requested number of statistic contexts for the function. */
	uint16_t num_hw_ring_grps;
	/*
	 * The number of HW ring groups that should be reserved for this
	 * function.
	 */
	uint8_t dflt_mac_addr[6];
	/* The default MAC address for the function being configured. */
	uint16_t dflt_vlan;
	/*
	 * The default VLAN for the function being configured. This VLAN is in
	 * 802.1Q tag format.
	 */
	uint32_t dflt_ip_addr[4]; /* big endian */
	/*
	 * The default IP address for the function being configured. This
	 * address is only used in enabling source property check.
	 */
	uint32_t min_bw;
	/*
	 * Minimum BW allocated for this function in Mbps. The HWRM will
	 * translate this value into byte counter and time interval used for the
	 * scheduler inside the device.
	 */
	uint32_t max_bw;
	/*
	 * Maximum BW allocated for this function in Mbps. The HWRM will
	 * translate this value into byte counter and time interval used for the
	 * scheduler inside the device.
	 */
	uint16_t async_event_cr;
	/*
	 * ID of the target completion ring for receiving asynchronous event
	 * completions. If this field is not valid, then the HWRM shall use the
	 * default completion ring of the function that is being configured as
	 * the target completion ring for providing any asynchronous event
	 * completions for that function. If this field is valid, then the HWRM
	 * shall use the completion ring identified by this ID as the target
	 * completion ring for providing any asynchronous event completions for
	 * the function that is being configured.
	 */
	uint8_t vlan_antispoof_mode;
	/* VLAN Anti-spoofing mode. */
	/* No VLAN anti-spoofing checks are enabled */
	#define HWRM_FUNC_CFG_INPUT_VLAN_ANTISPOOF_MODE_NOCHECK   (UINT32_C(0x0) << 0)
	/* Validate VLAN against the configured VLAN(s) */
	#define HWRM_FUNC_CFG_INPUT_VLAN_ANTISPOOF_MODE_VALIDATE_VLAN (UINT32_C(0x1) << 0)
	/* Insert VLAN if it does not exist, otherwise discard */
	#define HWRM_FUNC_CFG_INPUT_VLAN_ANTISPOOF_MODE_INSERT_IF_VLANDNE (UINT32_C(0x2) << 0)
	/* Insert VLAN if it does not exist, override VLAN if it exists */
	#define HWRM_FUNC_CFG_INPUT_VLAN_ANTISPOOF_MODE_INSERT_OR_OVERRIDE_VLAN (UINT32_C(0x3) << 0)
	uint8_t allowed_vlan_pris;
	/*
	 * This bit field defines VLAN PRIs that are allowed on this function.
	 * If nth bit is set, then VLAN PRI n is allowed on this function.
	 */
	uint8_t evb_mode;
	/*
	 * The HWRM shall allow a PF driver to change EVB mode for the partition
	 * it belongs to. The HWRM shall not allow a VF driver to change the EVB
	 * mode. The HWRM shall take into account the switching of EVB mode from
	 * one to another and reconfigure hardware resources as appropriately.
	 * The switching from VEB to VEPA mode requires the disabling of the
	 * loopback traffic. Additionally, source knock outs are handled
	 * differently in VEB and VEPA modes.
	 */
	/* No Edge Virtual Bridging (EVB) */
	#define HWRM_FUNC_CFG_INPUT_EVB_MODE_NO_EVB		(UINT32_C(0x0) << 0)
	/* Virtual Ethernet Bridge (VEB) */
	#define HWRM_FUNC_CFG_INPUT_EVB_MODE_VEB		(UINT32_C(0x1) << 0)
	/* Virtual Ethernet Port Aggregator (VEPA) */
	#define HWRM_FUNC_CFG_INPUT_EVB_MODE_VEPA		(UINT32_C(0x2) << 0)
	uint8_t unused_2;
	uint16_t num_mcast_filters;
	/*
	 * The number of multicast filters that should be reserved for this
	 * function on the RX side.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_qstats */
/*
 * Description: This command returns statistics of a function. The input FID
 * value is used to indicate what function is being queried. This allows a
 * physical function driver to query virtual functions that are children of the
 * physical function. The HWRM shall return any unsupported counter with a value
 * of 0xFFFFFFFF for 32-bit counters and 0xFFFFFFFFFFFFFFFF for 64-bit counters.
 */
/* Input (24 bytes) */

struct hwrm_func_qstats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t fid;
	/*
	 * Function ID of the function that is being queried. 0xFF... (All Fs)
	 * if the query is for the requesting function.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (176 bytes) */

struct hwrm_func_qstats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t tx_ucast_pkts;
	/* Number of transmitted unicast packets on the function. */
	uint64_t tx_mcast_pkts;
	/* Number of transmitted multicast packets on the function. */
	uint64_t tx_bcast_pkts;
	/* Number of transmitted broadcast packets on the function. */
	uint64_t tx_err_pkts;
	/*
	 * Number of transmitted packets that were dropped due to internal NIC
	 * resource problems. For transmit, this can only happen if TMP is
	 * configured to allow dropping in HOL blocking conditions, which is not
	 * a normal configuration.
	 */
	uint64_t tx_drop_pkts;
	/*
	 * Number of dropped packets on transmit path on the function. These are
	 * packets that have been marked for drop by the TE CFA block or are
	 * packets that exceeded the transmit MTU limit for the function.
	 */
	uint64_t tx_ucast_bytes;
	/* Number of transmitted bytes for unicast traffic on the function. */
	uint64_t tx_mcast_bytes;
	/* Number of transmitted bytes for multicast traffic on the function. */
	uint64_t tx_bcast_bytes;
	/* Number of transmitted bytes for broadcast traffic on the function. */
	uint64_t rx_ucast_pkts;
	/* Number of received unicast packets on the function. */
	uint64_t rx_mcast_pkts;
	/* Number of received multicast packets on the function. */
	uint64_t rx_bcast_pkts;
	/* Number of received broadcast packets on the function. */
	uint64_t rx_err_pkts;
	/*
	 * Number of received packets that were dropped on the function due to
	 * resource limitations. This can happen for 3 reasons. # The BD used
	 * for the packet has a bad format. # There were no BDs available in the
	 * ring for the packet. # There were no BDs available on-chip for the
	 * packet.
	 */
	uint64_t rx_drop_pkts;
	/*
	 * Number of dropped packets on received path on the function. These are
	 * packets that have been marked for drop by the RE CFA.
	 */
	uint64_t rx_ucast_bytes;
	/* Number of received bytes for unicast traffic on the function. */
	uint64_t rx_mcast_bytes;
	/* Number of received bytes for multicast traffic on the function. */
	uint64_t rx_bcast_bytes;
	/* Number of received bytes for broadcast traffic on the function. */
	uint64_t rx_agg_pkts;
	/* Number of aggregated unicast packets on the function. */
	uint64_t rx_agg_bytes;
	/* Number of aggregated unicast bytes on the function. */
	uint64_t rx_agg_events;
	/* Number of aggregation events on the function. */
	uint64_t rx_agg_aborts;
	/* Number of aborted aggregations on the function. */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_clr_stats */
/*
 * Description: This command clears statistics of a function. The input FID
 * value is used to indicate what function's statistics is being cleared. This
 * allows a physical function driver to clear statistics of virtual functions
 * that are children of the physical function.
 */
/* Input (24 bytes) */

struct hwrm_func_clr_stats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t fid;
	/*
	 * Function ID of the function. 0xFF... (All Fs) if the query is for the
	 * requesting function.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_clr_stats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_vf_resc_free */
/* Description: This command frees resources of a vf. */
/* Input (24 bytes) */

struct hwrm_func_vf_resc_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t vf_id;
	/*
	 * This value is used to identify a Virtual Function (VF). The scope of
	 * VF ID is local within a PF.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_vf_resc_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_vf_vnic_ids_query */
/* Description: This command is used to query vf vnic ids. */
/* Input (32 bytes) */

struct hwrm_func_vf_vnic_ids_query_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t vf_id;
	/*
	 * This value is used to identify a Virtual Function (VF). The scope of
	 * VF ID is local within a PF.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint32_t max_vnic_id_cnt;
	/* Max number of vnic ids in vnic id table */
	uint64_t vnic_id_tbl_addr;
	/* This is the address for VF VNIC ID table */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_vf_vnic_ids_query_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t vnic_id_cnt;
	/* Actual number of vnic ids Each VNIC ID is written as a 32-bit number. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_drv_rgtr */
/*
 * Description: This command is used by the function driver to register its
 * information with the HWRM. A function driver shall implement this command. A
 * function driver shall use this command during the driver initialization right
 * after the HWRM version discovery and default ring resources allocation.
 */
/* Input (80 bytes) */

struct hwrm_func_drv_rgtr_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the function driver is requesting all requests
	 * from its children VF drivers to be forwarded to itself. This flag can
	 * only be set by the PF driver. If a VF driver sets this flag, it
	 * should be ignored by the HWRM.
	 */
	#define HWRM_FUNC_DRV_RGTR_INPUT_FLAGS_FWD_ALL_MODE	UINT32_C(0x1)
	/*
	 * When this bit is '1', the function is requesting none of the requests
	 * from its children VF drivers to be forwarded to itself. This flag can
	 * only be set by the PF driver. If a VF driver sets this flag, it
	 * should be ignored by the HWRM.
	 */
	#define HWRM_FUNC_DRV_RGTR_INPUT_FLAGS_FWD_NONE_MODE	UINT32_C(0x2)
	uint32_t enables;
	/* This bit must be '1' for the os_type field to be configured. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_OS_TYPE	UINT32_C(0x1)
	/* This bit must be '1' for the ver field to be configured. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_VER		UINT32_C(0x2)
	/* This bit must be '1' for the timestamp field to be configured. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_TIMESTAMP	UINT32_C(0x4)
	/* This bit must be '1' for the vf_req_fwd field to be configured. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_VF_REQ_FWD	UINT32_C(0x8)
	/* This bit must be '1' for the async_event_fwd field to be configured. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_ENABLES_ASYNC_EVENT_FWD   UINT32_C(0x10)
	uint16_t os_type;
	/* This value indicates the type of OS. */
	/* Unknown */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_UNKNOWN	(UINT32_C(0x0) << 0)
	/* Other OS not listed below. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_OTHER		(UINT32_C(0x1) << 0)
	/* MSDOS OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_MSDOS		(UINT32_C(0xe) << 0)
	/* Windows OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_WINDOWS	(UINT32_C(0x12) << 0)
	/* Solaris OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_SOLARIS	(UINT32_C(0x1d) << 0)
	/* Linux OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_LINUX		(UINT32_C(0x24) << 0)
	/* FreeBSD OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_FREEBSD	(UINT32_C(0x2a) << 0)
	/* VMware ESXi OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_ESXI		(UINT32_C(0x68) << 0)
	/* Microsoft Windows 8 64-bit OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_WIN864	(UINT32_C(0x73) << 0)
	/* Microsoft Windows Server 2012 R2 OS. */
	#define HWRM_FUNC_DRV_RGTR_INPUT_OS_TYPE_WIN2012R2	(UINT32_C(0x74) << 0)
	uint8_t ver_maj;
	/* This is the major version of the driver. */
	uint8_t ver_min;
	/* This is the minor version of the driver. */
	uint8_t ver_upd;
	/* This is the update version of the driver. */
	uint8_t unused_0;
	uint16_t unused_1;
	uint32_t timestamp;
	/*
	 * This is a 32-bit timestamp provided by the driver for keep alive. The
	 * timestamp is in multiples of 1ms.
	 */
	uint32_t unused_2;
	uint32_t vf_req_fwd[8];
	/*
	 * This is a 256-bit bit mask provided by the PF driver for letting the
	 * HWRM know what commands issued by the VF driver to the HWRM should be
	 * forwarded to the PF driver. Nth bit refers to the Nth req_type.
	 * Setting Nth bit to 1 indicates that requests from the VF driver with
	 * req_type equal to N shall be forwarded to the parent PF driver. This
	 * field is not valid for the VF driver.
	 */
	uint32_t async_event_fwd[8];
	/*
	 * This is a 256-bit bit mask provided by the function driver (PF or VF
	 * driver) to indicate the list of asynchronous event completions to be
	 * forwarded. Nth bit refers to the Nth event_id. Setting Nth bit to 1
	 * by the function driver shall result in the HWRM forwarding
	 * asynchronous event completion with event_id equal to N. If all bits
	 * are set to 0 (value of 0), then the HWRM shall not forward any
	 * asynchronous event completion to this function driver.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_drv_rgtr_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_drv_unrgtr */
/*
 * Description: This command is used by the function driver to un register with
 * the HWRM. A function driver shall implement this command. A function driver
 * shall use this command during the driver unloading.
 */
/* Input (24 bytes) */

struct hwrm_func_drv_unrgtr_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the function driver is notifying the HWRM to
	 * prepare for the shutdown.
	 */
	#define HWRM_FUNC_DRV_UNRGTR_INPUT_FLAGS_PREPARE_FOR_SHUTDOWN UINT32_C(0x1)
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_drv_unrgtr_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_buf_rgtr */
/*
 * Description: This command is used by the PF driver to register buffers used
 * in the PF-VF communication with the HWRM. The PF driver uses this command to
 * register buffers for each PF-VF channel. A parent PF may issue this command
 * per child VF. If VF ID is not valid, then this command is used to register
 * buffers for all children VFs of the PF.
 */
/* Input (128 bytes) */

struct hwrm_func_buf_rgtr_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the vf_id field to be configured. */
	#define HWRM_FUNC_BUF_RGTR_INPUT_ENABLES_VF_ID		UINT32_C(0x1)
	/* This bit must be '1' for the err_buf_addr field to be configured. */
	#define HWRM_FUNC_BUF_RGTR_INPUT_ENABLES_ERR_BUF_ADDR	UINT32_C(0x2)
	uint16_t vf_id;
	/*
	 * This value is used to identify a Virtual Function (VF). The scope of
	 * VF ID is local within a PF.
	 */
	uint16_t req_buf_num_pages;
	/* This field represents the number of pages used for request buffer(s). */
	uint16_t req_buf_page_size;
	/* This field represents the page size used for request buffer(s). */
	/* 16 bytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_16B	(UINT32_C(0x4) << 0)
	/* 4 Kbytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_4K	(UINT32_C(0xc) << 0)
	/* 8 Kbytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_8K	(UINT32_C(0xd) << 0)
	/* 64 Kbytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_64K	(UINT32_C(0x10) << 0)
	/* 2 Mbytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_2M	(UINT32_C(0x15) << 0)
	/* 4 Mbytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_4M	(UINT32_C(0x16) << 0)
	/* 1 Gbytes */
	#define HWRM_FUNC_BUF_RGTR_INPUT_REQ_BUF_PAGE_SIZE_1G	(UINT32_C(0x1e) << 0)
	uint16_t req_buf_len;
	/* The length of the request buffer per VF in bytes. */
	uint16_t resp_buf_len;
	/* The length of the response buffer in bytes. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint64_t req_buf_page_addr0;
	/* This field represents the page address of page #0. */
	uint64_t req_buf_page_addr1;
	/* This field represents the page address of page #1. */
	uint64_t req_buf_page_addr2;
	/* This field represents the page address of page #2. */
	uint64_t req_buf_page_addr3;
	/* This field represents the page address of page #3. */
	uint64_t req_buf_page_addr4;
	/* This field represents the page address of page #4. */
	uint64_t req_buf_page_addr5;
	/* This field represents the page address of page #5. */
	uint64_t req_buf_page_addr6;
	/* This field represents the page address of page #6. */
	uint64_t req_buf_page_addr7;
	/* This field represents the page address of page #7. */
	uint64_t req_buf_page_addr8;
	/* This field represents the page address of page #8. */
	uint64_t req_buf_page_addr9;
	/* This field represents the page address of page #9. */
	uint64_t error_buf_addr;
	/*
	 * This field is used to receive the error reporting from the chipset.
	 * Only applicable for PFs.
	 */
	uint64_t resp_buf_addr;
	/* This field is used to receive the response forwarded by the HWRM. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_buf_rgtr_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_buf_unrgtr */
/*
 * Description: This command is used by the PF driver to unregister buffers used
 * in the PF-VF communication with the HWRM. The PF driver uses this command to
 * unregister buffers for PF-VF communication. A parent PF may issue this
 * command to unregister buffers for communication between the PF and a specific
 * VF. If the VF ID is not valid, then this command is used to unregister
 * buffers used for communications with all children VFs of the PF.
 */
/* Input (24 bytes) */

struct hwrm_func_buf_unrgtr_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the vf_id field to be configured. */
	#define HWRM_FUNC_BUF_UNRGTR_INPUT_ENABLES_VF_ID	UINT32_C(0x1)
	uint16_t vf_id;
	/*
	 * This value is used to identify a Virtual Function (VF). The scope of
	 * VF ID is local within a PF.
	 */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_buf_unrgtr_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_func_drv_qver */
/*
 * Description: This command is used to query the version of the driver. Any
 * driver version with major = 0, minor = 0, and update = 0 shall be considered
 * an invalid or unknown version.
 */
/* Input (24 bytes) */

struct hwrm_func_drv_qver_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t reserved;
	/* Reserved for future use */
	uint16_t fid;
	/*
	 * Function ID of the function that is being queried. 0xFF... (All Fs)
	 * if the query is for the requesting function.
	 */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_func_drv_qver_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t os_type;
	/* This value indicates the type of OS. */
	/* Unknown */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_UNKNOWN	(UINT32_C(0x0) << 0)
	/* Other OS not listed below. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_OTHER	(UINT32_C(0x1) << 0)
	/* MSDOS OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_MSDOS	(UINT32_C(0xe) << 0)
	/* Windows OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_WINDOWS	(UINT32_C(0x12) << 0)
	/* Solaris OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_SOLARIS	(UINT32_C(0x1d) << 0)
	/* Linux OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_LINUX	(UINT32_C(0x24) << 0)
	/* FreeBSD OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_FREEBSD	(UINT32_C(0x2a) << 0)
	/* VMware ESXi OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_ESXI		(UINT32_C(0x68) << 0)
	/* Microsoft Windows 8 64-bit OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_WIN864	(UINT32_C(0x73) << 0)
	/* Microsoft Windows Server 2012 R2 OS. */
	#define HWRM_FUNC_DRV_QVER_OUTPUT_OS_TYPE_WIN2012R2	(UINT32_C(0x74) << 0)
	uint8_t ver_maj;
	/* This is the major version of the driver. */
	uint8_t ver_min;
	/* This is the minor version of the driver. */
	uint8_t ver_upd;
	/* This is the update version of the driver. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_phy_cfg */
/*
 * Description: This command configures the PHY device for the port. It allows
 * setting of the most generic settings for the PHY. The HWRM shall complete
 * this command as soon as PHY settings are configured. They may not be applied
 * when the command response is provided. A VF driver shall not be allowed to
 * configure PHY using this command. In a network partition mode, a PF driver
 * shall not be allowed to configure PHY using this command.
 */
/* Input (56 bytes) */

struct hwrm_port_phy_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is set to '1', the PHY for the port shall be reset. #
	 * If this bit is set to 1, then the HWRM shall reset the PHY after
	 * applying PHY configuration changes specified in this command. # In
	 * order to guarantee that PHY configuration changes specified in this
	 * command take effect, the HWRM client should set this flag to 1. # If
	 * this bit is not set to 1, then the HWRM may reset the PHY depending
	 * on the current PHY configuration and settings specified in this
	 * command.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_RESET_PHY		UINT32_C(0x1)
	/*
	 * When this bit is set to '1', the link shall be forced to be taken
	 * down. # When this bit is set to '1", all other command input settings
	 * related to the link speed shall be ignored. Once the link state is
	 * forced down, it can be explicitly cleared from that state by setting
	 * this flag to '0'. # If this flag is set to '0', then the link shall
	 * be cleared from forced down state if the link is in forced down
	 * state. There may be conditions (e.g. out-of-band or sideband
	 * configuration changes for the link) outside the scope of the HWRM
	 * implementation that may clear forced down link state.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FORCE_LINK_DOWN	UINT32_C(0x2)
	/*
	 * When this bit is set to '1', the link shall be forced to the
	 * force_link_speed value. When this bit is set to '1', the HWRM client
	 * should not enable any of the auto negotiation related fields
	 * represented by auto_XXX fields in this command. When this bit is set
	 * to '1' and the HWRM client has enabled a auto_XXX field in this
	 * command, then the HWRM shall ignore the enabled auto_XXX field. When
	 * this bit is set to zero, the link shall be allowed to autoneg.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FORCE		UINT32_C(0x4)
	/*
	 * When this bit is set to '1', the auto-negotiation process shall be
	 * restarted on the link.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_RESTART_AUTONEG	UINT32_C(0x8)
	/*
	 * When this bit is set to '1', Energy Efficient Ethernet (EEE) is
	 * requested to be enabled on this link. If EEE is not supported on this
	 * port, then this flag shall be ignored by the HWRM.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_EEE_ENABLE	UINT32_C(0x10)
	/*
	 * When this bit is set to '1', Energy Efficient Ethernet (EEE) is
	 * requested to be disabled on this link. If EEE is not supported on
	 * this port, then this flag shall be ignored by the HWRM.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_EEE_DISABLE	UINT32_C(0x20)
	/*
	 * When this bit is set to '1' and EEE is enabled on this link, then TX
	 * LPI is requested to be enabled on the link. If EEE is not supported
	 * on this port, then this flag shall be ignored by the HWRM. If EEE is
	 * disabled on this port, then this flag shall be ignored by the HWRM.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_EEE_TX_LPI_ENABLE	UINT32_C(0x40)
	/*
	 * When this bit is set to '1' and EEE is enabled on this link, then TX
	 * LPI is requested to be disabled on the link. If EEE is not supported
	 * on this port, then this flag shall be ignored by the HWRM. If EEE is
	 * disabled on this port, then this flag shall be ignored by the HWRM.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_EEE_TX_LPI_DISABLE   UINT32_C(0x80)
	/*
	 * When set to 1, then the HWRM shall enable FEC autonegotitation on
	 * this port if supported. When set to 0, then this flag shall be
	 * ignored. If FEC autonegotiation is not supported, then the HWRM shall
	 * ignore this flag.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FEC_AUTONEG_ENABLE   UINT32_C(0x100)
	/*
	 * When set to 1, then the HWRM shall disable FEC autonegotiation on
	 * this port if supported. When set to 0, then this flag shall be
	 * ignored. If FEC autonegotiation is not supported, then the HWRM shall
	 * ignore this flag.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FEC_AUTONEG_DISABLE  UINT32_C(0x200)
	/*
	 * When set to 1, then the HWRM shall enable FEC CLAUSE 74 (Fire Code)
	 * on this port if supported. When set to 0, then this flag shall be
	 * ignored. If FEC CLAUSE 74 is not supported, then the HWRM shall
	 * ignore this flag.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FEC_CLAUSE74_ENABLE  UINT32_C(0x400)
	/*
	 * When set to 1, then the HWRM shall disable FEC CLAUSE 74 (Fire Code)
	 * on this port if supported. When set to 0, then this flag shall be
	 * ignored. If FEC CLAUSE 74 is not supported, then the HWRM shall
	 * ignore this flag.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FEC_CLAUSE74_DISABLE UINT32_C(0x800)
	/*
	 * When set to 1, then the HWRM shall enable FEC CLAUSE 91 (Reed
	 * Solomon) on this port if supported. When set to 0, then this flag
	 * shall be ignored. If FEC CLAUSE 91 is not supported, then the HWRM
	 * shall ignore this flag.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FEC_CLAUSE91_ENABLE  UINT32_C(0x1000)
	/*
	 * When set to 1, then the HWRM shall disable FEC CLAUSE 91 (Reed
	 * Solomon) on this port if supported. When set to 0, then this flag
	 * shall be ignored. If FEC CLAUSE 91 is not supported, then the HWRM
	 * shall ignore this flag.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FLAGS_FEC_CLAUSE91_DISABLE UINT32_C(0x2000)
	uint32_t enables;
	/* This bit must be '1' for the auto_mode field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_AUTO_MODE	UINT32_C(0x1)
	/* This bit must be '1' for the auto_duplex field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_AUTO_DUPLEX	UINT32_C(0x2)
	/* This bit must be '1' for the auto_pause field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_AUTO_PAUSE	UINT32_C(0x4)
	/* This bit must be '1' for the auto_link_speed field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_AUTO_LINK_SPEED	UINT32_C(0x8)
	/*
	 * This bit must be '1' for the auto_link_speed_mask field to be
	 * configured.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_AUTO_LINK_SPEED_MASK UINT32_C(0x10)
	/* This bit must be '1' for the wirespeed field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_WIRESPEED	UINT32_C(0x20)
	/* This bit must be '1' for the lpbk field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_LPBK		UINT32_C(0x40)
	/* This bit must be '1' for the preemphasis field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_PREEMPHASIS	UINT32_C(0x80)
	/* This bit must be '1' for the force_pause field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_FORCE_PAUSE	UINT32_C(0x100)
	/*
	 * This bit must be '1' for the eee_link_speed_mask field to be
	 * configured.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_EEE_LINK_SPEED_MASK UINT32_C(0x200)
	/* This bit must be '1' for the tx_lpi_timer field to be configured. */
	#define HWRM_PORT_PHY_CFG_INPUT_ENABLES_TX_LPI_TIMER	UINT32_C(0x400)
	uint16_t port_id;
	/* Port ID of port that is to be configured. */
	uint16_t force_link_speed;
	/*
	 * This is the speed that will be used if the force bit is '1'. If
	 * unsupported speed is selected, an error will be generated.
	 */
	/* 100Mb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_100MB	(UINT32_C(0x1) << 0)
	/* 1Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_1GB	(UINT32_C(0xa) << 0)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_2GB	(UINT32_C(0x14) << 0)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_2_5GB	(UINT32_C(0x19) << 0)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_10GB	(UINT32_C(0x64) << 0)
	/* 20Mb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_20GB	(UINT32_C(0xc8) << 0)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_25GB	(UINT32_C(0xfa) << 0)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_40GB	(UINT32_C(0x190) << 0)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_50GB	(UINT32_C(0x1f4) << 0)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_100GB	(UINT32_C(0x3e8) << 0)
	/* 10Mb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_LINK_SPEED_10MB	(UINT32_C(0xffff) << 0)
	uint8_t auto_mode;
	/*
	 * This value is used to identify what autoneg mode is used when the
	 * link speed is not being forced.
	 */
	/* Disable autoneg or autoneg disabled. No speeds are selected. */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_MODE_NONE		(UINT32_C(0x0) << 0)
	/* Select all possible speeds for autoneg mode. */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_MODE_ALL_SPEEDS	(UINT32_C(0x1) << 0)
	/*
	 * Select only the auto_link_speed speed for autoneg mode. This
	 * mode has been DEPRECATED. An HWRM client should not use this
	 * mode.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_MODE_ONE_SPEED	(UINT32_C(0x2) << 0)
	/*
	 * Select the auto_link_speed or any speed below that speed for
	 * autoneg. This mode has been DEPRECATED. An HWRM client should
	 * not use this mode.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_MODE_ONE_OR_BELOW	(UINT32_C(0x3) << 0)
	/*
	 * Select the speeds based on the corresponding link speed mask
	 * value that is provided.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_MODE_SPEED_MASK	(UINT32_C(0x4) << 0)
	uint8_t auto_duplex;
	/*
	 * This is the duplex setting that will be used if the autoneg_mode is
	 * "one_speed" or "one_or_below".
	 */
	/* Half Duplex will be requested. */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_DUPLEX_HALF	(UINT32_C(0x0) << 0)
	/* Full duplex will be requested. */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_DUPLEX_FULL	(UINT32_C(0x1) << 0)
	/* Both Half and Full dupex will be requested. */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_DUPLEX_BOTH	(UINT32_C(0x2) << 0)
	uint8_t auto_pause;
	/*
	 * This value is used to configure the pause that will be used for
	 * autonegotiation. Add text on the usage of auto_pause and force_pause.
	 */
	/*
	 * When this bit is '1', Generation of tx pause messages has been
	 * requested. Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_PAUSE_TX		UINT32_C(0x1)
	/*
	 * When this bit is '1', Reception of rx pause messages has been
	 * requested. Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_PAUSE_RX		UINT32_C(0x2)
	/*
	 * When set to 1, the advertisement of pause is enabled. # When the
	 * auto_mode is not set to none and this flag is set to 1, then the
	 * auto_pause bits on this port are being advertised and autoneg pause
	 * results are being interpreted. # When the auto_mode is not set to
	 * none and this flag is set to 0, the pause is forced as indicated in
	 * force_pause, and also advertised as auto_pause bits, but the autoneg
	 * results are not interpreted since the pause configuration is being
	 * forced. # When the auto_mode is set to none and this flag is set to
	 * 1, auto_pause bits should be ignored and should be set to 0.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_PAUSE_AUTONEG_PAUSE   UINT32_C(0x4)
	uint8_t unused_0;
	uint16_t auto_link_speed;
	/*
	 * This is the speed that will be used if the autoneg_mode is
	 * "one_speed" or "one_or_below". If an unsupported speed is selected,
	 * an error will be generated.
	 */
	/* 100Mb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_100MB	(UINT32_C(0x1) << 0)
	/* 1Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_1GB	(UINT32_C(0xa) << 0)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_2GB	(UINT32_C(0x14) << 0)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_2_5GB	(UINT32_C(0x19) << 0)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_10GB	(UINT32_C(0x64) << 0)
	/* 20Mb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_20GB	(UINT32_C(0xc8) << 0)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_25GB	(UINT32_C(0xfa) << 0)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_40GB	(UINT32_C(0x190) << 0)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_50GB	(UINT32_C(0x1f4) << 0)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_100GB	(UINT32_C(0x3e8) << 0)
	/* 10Mb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_10MB	(UINT32_C(0xffff) << 0)
	uint16_t auto_link_speed_mask;
	/*
	 * This is a mask of link speeds that will be used if autoneg_mode is
	 * "mask". If unsupported speed is enabled an error will be generated.
	 */
	/* 100Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_100MBHD UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_100MB UINT32_C(0x2)
	/* 1Gb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_1GBHD UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_1GB   UINT32_C(0x8)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_2GB   UINT32_C(0x10)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_2_5GB UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_10GB  UINT32_C(0x40)
	/* 20Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_20GB  UINT32_C(0x80)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_25GB  UINT32_C(0x100)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_40GB  UINT32_C(0x200)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_50GB  UINT32_C(0x400)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_100GB UINT32_C(0x800)
	/* 10Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_10MBHD UINT32_C(0x1000)
	/* 10Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_AUTO_LINK_SPEED_MASK_10MB  UINT32_C(0x2000)
	uint8_t wirespeed;
	/* This value controls the wirespeed feature. */
	/* Wirespeed feature is disabled. */
	#define HWRM_PORT_PHY_CFG_INPUT_WIRESPEED_OFF		(UINT32_C(0x0) << 0)
	/* Wirespeed feature is enabled. */
	#define HWRM_PORT_PHY_CFG_INPUT_WIRESPEED_ON		(UINT32_C(0x1) << 0)
	uint8_t lpbk;
	/* This value controls the loopback setting for the PHY. */
	/* No loopback is selected. Normal operation. */
	#define HWRM_PORT_PHY_CFG_INPUT_LPBK_NONE		(UINT32_C(0x0) << 0)
	/*
	 * The HW will be configured with local loopback such that host
	 * data is sent back to the host without modification.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_LPBK_LOCAL		(UINT32_C(0x1) << 0)
	/*
	 * The HW will be configured with remote loopback such that port
	 * logic will send packets back out the transmitter that are
	 * received.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_LPBK_REMOTE		(UINT32_C(0x2) << 0)
	uint8_t force_pause;
	/*
	 * This value is used to configure the pause that will be used for force
	 * mode.
	 */
	/*
	 * When this bit is '1', Generation of tx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_PAUSE_TX		UINT32_C(0x1)
	/*
	 * When this bit is '1', Reception of rx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_FORCE_PAUSE_RX		UINT32_C(0x2)
	uint8_t unused_1;
	uint32_t preemphasis;
	/*
	 * This value controls the pre-emphasis to be used for the link. Driver
	 * should not set this value (use enable.preemphasis = 0) unless driver
	 * is sure of setting. Normally HWRM FW will determine proper pre-
	 * emphasis.
	 */
	uint16_t eee_link_speed_mask;
	/*
	 * Setting for link speed mask that is used to advertise speeds during
	 * autonegotiation when EEE is enabled. This field is valid only when
	 * EEE is enabled. The speeds specified in this field shall be a subset
	 * of speeds specified in auto_link_speed_mask. If EEE is enabled,then
	 * at least one speed shall be provided in this mask.
	 */
	/* Reserved */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_RSVD1  UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_100MB  UINT32_C(0x2)
	/* Reserved */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_RSVD2  UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_1GB	UINT32_C(0x8)
	/* Reserved */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_RSVD3  UINT32_C(0x10)
	/* Reserved */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_RSVD4  UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_CFG_INPUT_EEE_LINK_SPEED_MASK_10GB   UINT32_C(0x40)
	uint8_t unused_2;
	uint8_t unused_3;
	uint32_t tx_lpi_timer;
	uint32_t unused_4;
	/*
	 * Reuested setting of TX LPI timer in microseconds. This field is valid
	 * only when EEE is enabled and TX LPI is enabled.
	 */
	#define HWRM_PORT_PHY_CFG_INPUT_TX_LPI_TIMER_MASK	UINT32_C(0xffffff)
	#define HWRM_PORT_PHY_CFG_INPUT_TX_LPI_TIMER_SFT	0
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_port_phy_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_phy_qcfg */
/* Description: This command queries the PHY configuration for the port. */
/* Input (24 bytes) */

struct hwrm_port_phy_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/* Port ID of port that is to be queried. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (96 bytes) */

struct hwrm_port_phy_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t link;
	/* This value indicates the current link status. */
	/* There is no link or cable detected. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_NO_LINK		(UINT32_C(0x0) << 0)
	/* There is no link, but a cable has been detected. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SIGNAL		(UINT32_C(0x1) << 0)
	/* There is a link. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_LINK		(UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint16_t link_speed;
	/* This value indicates the current link speed of the connection. */
	/* 100Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_100MB	(UINT32_C(0x1) << 0)
	/* 1Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_1GB	(UINT32_C(0xa) << 0)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_2GB	(UINT32_C(0x14) << 0)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_2_5GB	(UINT32_C(0x19) << 0)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_10GB	(UINT32_C(0x64) << 0)
	/* 20Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_20GB	(UINT32_C(0xc8) << 0)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_25GB	(UINT32_C(0xfa) << 0)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_40GB	(UINT32_C(0x190) << 0)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_50GB	(UINT32_C(0x1f4) << 0)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_100GB	(UINT32_C(0x3e8) << 0)
	/* 10Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_SPEED_10MB	(UINT32_C(0xffff) << 0)
	uint8_t duplex;
	/* This value is indicates the duplex of the current connection. */
	/* Half Duplex connection. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_DUPLEX_HALF		(UINT32_C(0x0) << 0)
	/* Full duplex connection. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_DUPLEX_FULL		(UINT32_C(0x1) << 0)
	uint8_t pause;
	/*
	 * This value is used to indicate the current pause configuration. When
	 * autoneg is enabled, this value represents the autoneg results of
	 * pause configuration.
	 */
	/*
	 * When this bit is '1', Generation of tx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PAUSE_TX		UINT32_C(0x1)
	/*
	 * When this bit is '1', Reception of rx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PAUSE_RX		UINT32_C(0x2)
	uint16_t support_speeds;
	/*
	 * The supported speeds for the port. This is a bit mask. For each speed
	 * that is supported, the corrresponding bit will be set to '1'.
	 */
	/* 100Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MBHD   UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100MB	UINT32_C(0x2)
	/* 1Gb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GBHD	UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_1GB	UINT32_C(0x8)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_2GB	UINT32_C(0x10)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_2_5GB	UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10GB	UINT32_C(0x40)
	/* 20Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_20GB	UINT32_C(0x80)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_25GB	UINT32_C(0x100)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_40GB	UINT32_C(0x200)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_50GB	UINT32_C(0x400)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_100GB	UINT32_C(0x800)
	/* 10Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10MBHD	UINT32_C(0x1000)
	/* 10Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_SUPPORT_SPEEDS_10MB	UINT32_C(0x2000)
	uint16_t force_link_speed;
	/*
	 * Current setting of forced link speed. When the link speed is not
	 * being forced, this value shall be set to 0.
	 */
	/* 100Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_100MB  (UINT32_C(0x1) << 0)
	/* 1Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_1GB	(UINT32_C(0xa) << 0)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_2GB	(UINT32_C(0x14) << 0)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_2_5GB  (UINT32_C(0x19) << 0)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_10GB   (UINT32_C(0x64) << 0)
	/* 20Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_20GB   (UINT32_C(0xc8) << 0)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_25GB   (UINT32_C(0xfa) << 0)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_40GB   (UINT32_C(0x190) << 0)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_50GB   (UINT32_C(0x1f4) << 0)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_100GB  (UINT32_C(0x3e8) << 0)
	/* 10Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_LINK_SPEED_10MB   (UINT32_C(0xffff) << 0)
	uint8_t auto_mode;
	/* Current setting of auto negotiation mode. */
	/* Disable autoneg or autoneg disabled. No speeds are selected. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_NONE	(UINT32_C(0x0) << 0)
	/* Select all possible speeds for autoneg mode. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_ALL_SPEEDS	(UINT32_C(0x1) << 0)
	/*
	 * Select only the auto_link_speed speed for autoneg mode. This
	 * mode has been DEPRECATED. An HWRM client should not use this
	 * mode.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_ONE_SPEED	(UINT32_C(0x2) << 0)
	/*
	 * Select the auto_link_speed or any speed below that speed for
	 * autoneg. This mode has been DEPRECATED. An HWRM client should
	 * not use this mode.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_ONE_OR_BELOW  (UINT32_C(0x3) << 0)
	/*
	 * Select the speeds based on the corresponding link speed mask
	 * value that is provided.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_MODE_SPEED_MASK	(UINT32_C(0x4) << 0)
	uint8_t auto_pause;
	/*
	 * Current setting of pause autonegotiation. Move autoneg_pause flag
	 * here.
	 */
	/*
	 * When this bit is '1', Generation of tx pause messages has been
	 * requested. Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_PAUSE_TX		UINT32_C(0x1)
	/*
	 * When this bit is '1', Reception of rx pause messages has been
	 * requested. Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_PAUSE_RX		UINT32_C(0x2)
	/*
	 * When set to 1, the advertisement of pause is enabled. # When the
	 * auto_mode is not set to none and this flag is set to 1, then the
	 * auto_pause bits on this port are being advertised and autoneg pause
	 * results are being interpreted. # When the auto_mode is not set to
	 * none and this flag is set to 0, the pause is forced as indicated in
	 * force_pause, and also advertised as auto_pause bits, but the autoneg
	 * results are not interpreted since the pause configuration is being
	 * forced. # When the auto_mode is set to none and this flag is set to
	 * 1, auto_pause bits should be ignored and should be set to 0.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_PAUSE_AUTONEG_PAUSE UINT32_C(0x4)
	uint16_t auto_link_speed;
	/*
	 * Current setting for auto_link_speed. This field is only valid when
	 * auto_mode is set to "one_speed" or "one_or_below".
	 */
	/* 100Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_100MB   (UINT32_C(0x1) << 0)
	/* 1Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_1GB	(UINT32_C(0xa) << 0)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_2GB	(UINT32_C(0x14) << 0)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_2_5GB   (UINT32_C(0x19) << 0)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_10GB	(UINT32_C(0x64) << 0)
	/* 20Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_20GB	(UINT32_C(0xc8) << 0)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_25GB	(UINT32_C(0xfa) << 0)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_40GB	(UINT32_C(0x190) << 0)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_50GB	(UINT32_C(0x1f4) << 0)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_100GB   (UINT32_C(0x3e8) << 0)
	/* 10Mb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_10MB	(UINT32_C(0xffff) << 0)
	uint16_t auto_link_speed_mask;
	/*
	 * Current setting for auto_link_speed_mask that is used to advertise
	 * speeds during autonegotiation. This field is only valid when
	 * auto_mode is set to "mask". The speeds specified in this field shall
	 * be a subset of supported speeds on this port.
	 */
	/* 100Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_100MBHD UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_100MB UINT32_C(0x2)
	/* 1Gb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_1GBHD UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_1GB UINT32_C(0x8)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_2GB UINT32_C(0x10)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_2_5GB UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_10GB UINT32_C(0x40)
	/* 20Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_20GB UINT32_C(0x80)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_25GB UINT32_C(0x100)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_40GB UINT32_C(0x200)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_50GB UINT32_C(0x400)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_100GB UINT32_C(0x800)
	/* 10Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_10MBHD UINT32_C(0x1000)
	/* 10Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_AUTO_LINK_SPEED_MASK_10MB UINT32_C(0x2000)
	uint8_t wirespeed;
	/* Current setting for wirespeed. */
	/* Wirespeed feature is disabled. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_WIRESPEED_OFF	(UINT32_C(0x0) << 0)
	/* Wirespeed feature is enabled. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_WIRESPEED_ON		(UINT32_C(0x1) << 0)
	uint8_t lpbk;
	/* Current setting for loopback. */
	/* No loopback is selected. Normal operation. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LPBK_NONE		(UINT32_C(0x0) << 0)
	/*
	 * The HW will be configured with local loopback such that host
	 * data is sent back to the host without modification.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LPBK_LOCAL		(UINT32_C(0x1) << 0)
	/*
	 * The HW will be configured with remote loopback such that port
	 * logic will send packets back out the transmitter that are
	 * received.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LPBK_REMOTE		(UINT32_C(0x2) << 0)
	uint8_t force_pause;
	/*
	 * Current setting of forced pause. When the pause configuration is not
	 * being forced, then this value shall be set to 0.
	 */
	/*
	 * When this bit is '1', Generation of tx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_PAUSE_TX	UINT32_C(0x1)
	/*
	 * When this bit is '1', Reception of rx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FORCE_PAUSE_RX	UINT32_C(0x2)
	uint8_t module_status;
	/*
	 * This value indicates the current status of the optics module on this
	 * port.
	 */
	/* Module is inserted and accepted */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MODULE_STATUS_NONE	(UINT32_C(0x0) << 0)
	/* Module is rejected and transmit side Laser is disabled. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MODULE_STATUS_DISABLETX (UINT32_C(0x1) << 0)
	/* Module mismatch warning. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MODULE_STATUS_WARNINGMSG (UINT32_C(0x2) << 0)
	/* Module is rejected and powered down. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MODULE_STATUS_PWRDOWN   (UINT32_C(0x3) << 0)
	/* Module is not inserted. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MODULE_STATUS_NOTINSERTED (UINT32_C(0x4) << 0)
	/* Module status is not applicable. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MODULE_STATUS_NOTAPPLICABLE (UINT32_C(0xff) << 0)
	uint32_t preemphasis;
	/* Current setting for preemphasis. */
	uint8_t phy_maj;
	/* This field represents the major version of the PHY. */
	uint8_t phy_min;
	/* This field represents the minor version of the PHY. */
	uint8_t phy_bld;
	/* This field represents the build version of the PHY. */
	uint8_t phy_type;
	/* This value represents a PHY type. */
	/* Unknown */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_UNKNOWN	(UINT32_C(0x0) << 0)
	/* BASE-CR */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASECR	(UINT32_C(0x1) << 0)
	/* BASE-KR4 (Deprecated) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKR4	(UINT32_C(0x2) << 0)
	/* BASE-LR */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASELR	(UINT32_C(0x3) << 0)
	/* BASE-SR */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASESR	(UINT32_C(0x4) << 0)
	/* BASE-KR2 (Deprecated) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKR2	(UINT32_C(0x5) << 0)
	/* BASE-KX */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKX	(UINT32_C(0x6) << 0)
	/* BASE-KR */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASEKR	(UINT32_C(0x7) << 0)
	/* BASE-T */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASET	(UINT32_C(0x8) << 0)
	/* EEE capable BASE-T */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_BASETE	(UINT32_C(0x9) << 0)
	/* SGMII connected external PHY */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_TYPE_SGMIIEXTPHY	(UINT32_C(0xa) << 0)
	uint8_t media_type;
	/* This value represents a media type. */
	/* Unknown */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MEDIA_TYPE_UNKNOWN	(UINT32_C(0x0) << 0)
	/* Twisted Pair */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MEDIA_TYPE_TP	(UINT32_C(0x1) << 0)
	/* Direct Attached Copper */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MEDIA_TYPE_DAC	(UINT32_C(0x2) << 0)
	/* Fiber */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_MEDIA_TYPE_FIBRE	(UINT32_C(0x3) << 0)
	uint8_t xcvr_pkg_type;
	/* This value represents a transceiver type. */
	/* PHY and MAC are in the same package */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_PKG_TYPE_XCVR_INTERNAL (UINT32_C(0x1) << 0)
	/* PHY and MAC are in different packages */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_PKG_TYPE_XCVR_EXTERNAL (UINT32_C(0x2) << 0)
	uint8_t eee_config_phy_addr;
	/*
	 * This field represents flags related to EEE configuration. These EEE
	 * configuration flags are valid only when the auto_mode is not set to
	 * none (in other words autonegotiation is enabled).
	 */
	/* This field represents PHY address. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_ADDR_MASK		UINT32_C(0x1f)
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PHY_ADDR_SFT		0
	/*
	 * When set to 1, Energy Efficient Ethernet (EEE) mode is enabled.
	 * Speeds for autoneg with EEE mode enabled are based on
	 * eee_link_speed_mask.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_EEE_CONFIG_EEE_ENABLED   UINT32_C(0x20)
	/*
	 * This flag is valid only when eee_enabled is set to 1. # If
	 * eee_enabled is set to 0, then EEE mode is disabled and this flag
	 * shall be ignored. # If eee_enabled is set to 1 and this flag is set
	 * to 1, then Energy Efficient Ethernet (EEE) mode is enabled and in
	 * use. # If eee_enabled is set to 1 and this flag is set to 0, then
	 * Energy Efficient Ethernet (EEE) mode is enabled but is currently not
	 * in use.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_EEE_CONFIG_EEE_ACTIVE	UINT32_C(0x40)
	/*
	 * This flag is valid only when eee_enabled is set to 1. # If
	 * eee_enabled is set to 0, then EEE mode is disabled and this flag
	 * shall be ignored. # If eee_enabled is set to 1 and this flag is set
	 * to 1, then Energy Efficient Ethernet (EEE) mode is enabled and TX LPI
	 * is enabled. # If eee_enabled is set to 1 and this flag is set to 0,
	 * then Energy Efficient Ethernet (EEE) mode is enabled but TX LPI is
	 * disabled.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_EEE_CONFIG_EEE_TX_LPI	UINT32_C(0x80)
	/*
	 * This field represents flags related to EEE configuration. These EEE
	 * configuration flags are valid only when the auto_mode is not set to
	 * none (in other words autonegotiation is enabled).
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_EEE_CONFIG_MASK	UINT32_C(0xe0)
	#define HWRM_PORT_PHY_QCFG_OUTPUT_EEE_CONFIG_SFT	5
	uint8_t parallel_detect;
	/* Reserved field, set to 0 */
	/*
	 * When set to 1, the parallel detection is used to determine the speed
	 * of the link partner. Parallel detection is used when a
	 * autonegotiation capable device is connected to a link parter that is
	 * not capable of autonegotiation.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_PARALLEL_DETECT	UINT32_C(0x1)
	/* Reserved field, set to 0 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_RESERVED_MASK		UINT32_C(0xfe)
	#define HWRM_PORT_PHY_QCFG_OUTPUT_RESERVED_SFT		1
	uint16_t link_partner_adv_speeds;
	/*
	 * The advertised speeds for the port by the link partner. Each
	 * advertised speed will be set to '1'.
	 */
	/* 100Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_100MBHD UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_100MB UINT32_C(0x2)
	/* 1Gb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_1GBHD UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_1GB UINT32_C(0x8)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_2GB UINT32_C(0x10)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_2_5GB UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_10GB UINT32_C(0x40)
	/* 20Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_20GB UINT32_C(0x80)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_25GB UINT32_C(0x100)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_40GB UINT32_C(0x200)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_50GB UINT32_C(0x400)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_100GB UINT32_C(0x800)
	/* 10Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_10MBHD UINT32_C(0x1000)
	/* 10Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_SPEEDS_10MB UINT32_C(0x2000)
	uint8_t link_partner_adv_auto_mode;
	/*
	 * The advertised autoneg for the port by the link partner. This field
	 * is deprecated and should be set to 0.
	 */
	/* Disable autoneg or autoneg disabled. No speeds are selected. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_AUTO_MODE_NONE (UINT32_C(0x0) << 0)
	/* Select all possible speeds for autoneg mode. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_AUTO_MODE_ALL_SPEEDS (UINT32_C(0x1) << 0)
	/*
	 * Select only the auto_link_speed speed for autoneg mode. This
	 * mode has been DEPRECATED. An HWRM client should not use this
	 * mode.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_AUTO_MODE_ONE_SPEED (UINT32_C(0x2) << 0)
	/*
	 * Select the auto_link_speed or any speed below that speed for
	 * autoneg. This mode has been DEPRECATED. An HWRM client should
	 * not use this mode.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_AUTO_MODE_ONE_OR_BELOW (UINT32_C(0x3) << 0)
	/*
	 * Select the speeds based on the corresponding link speed mask
	 * value that is provided.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_AUTO_MODE_SPEED_MASK (UINT32_C(0x4) << 0)
	uint8_t link_partner_adv_pause;
	/* The advertised pause settings on the port by the link partner. */
	/*
	 * When this bit is '1', Generation of tx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_PAUSE_TX UINT32_C(0x1)
	/*
	 * When this bit is '1', Reception of rx pause messages is supported.
	 * Disabled otherwise.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_PAUSE_RX UINT32_C(0x2)
	uint16_t adv_eee_link_speed_mask;
	/*
	 * Current setting for link speed mask that is used to advertise speeds
	 * during autonegotiation when EEE is enabled. This field is valid only
	 * when eee_enabled flags is set to 1. The speeds specified in this
	 * field shall be a subset of speeds specified in auto_link_speed_mask.
	 */
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_RSVD1 UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_100MB UINT32_C(0x2)
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_RSVD2 UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_1GB UINT32_C(0x8)
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_RSVD3 UINT32_C(0x10)
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_RSVD4 UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_ADV_EEE_LINK_SPEED_MASK_10GB UINT32_C(0x40)
	uint16_t link_partner_adv_eee_link_speed_mask;
	/*
	 * Current setting for link speed mask that is advertised by the link
	 * partner when EEE is enabled. This field is valid only when
	 * eee_enabled flags is set to 1.
	 */
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_RSVD1 UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_100MB UINT32_C(0x2)
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_RSVD2 UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_1GB UINT32_C(0x8)
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_RSVD3 UINT32_C(0x10)
	/* Reserved */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_RSVD4 UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_LINK_PARTNER_ADV_EEE_LINK_SPEED_MASK_10GB UINT32_C(0x40)
	uint32_t xcvr_identifier_type_tx_lpi_timer;
	/* This value represents transceiver identifier type. */
	/*
	 * Current setting of TX LPI timer in microseconds. This field is valid
	 * only when_eee_enabled flag is set to 1 and tx_lpi_enabled is set to
	 * 1.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_TX_LPI_TIMER_MASK	UINT32_C(0xffffff)
	#define HWRM_PORT_PHY_QCFG_OUTPUT_TX_LPI_TIMER_SFT	0
	/* This value represents transceiver identifier type. */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_MASK UINT32_C(0xff000000)
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_SFT 24
	/* Unknown */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_UNKNOWN (UINT32_C(0x0) << 24)
	/* SFP/SFP+/SFP28 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_SFP (UINT32_C(0x3) << 24)
	/* QSFP */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_QSFP (UINT32_C(0xc) << 24)
	/* QSFP+ */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_QSFPPLUS (UINT32_C(0xd) << 24)
	/* QSFP28 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_XCVR_IDENTIFIER_TYPE_QSFP28 (UINT32_C(0x11) << 24)
	uint16_t fec_cfg;
	/*
	 * This value represents the current configuration of Forward Error
	 * Correction (FEC) on the port.
	 */
	/*
	 * When set to 1, then FEC is not supported on this port. If this flag
	 * is set to 1, then all other FEC configuration flags shall be ignored.
	 * When set to 0, then FEC is supported as indicated by other
	 * configuration flags. If no cable is attached and the HWRM does not
	 * yet know the FEC capability, then the HWRM shall set this flag to 1
	 * when reporting FEC capability.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_NONE_SUPPORTED UINT32_C(0x1)
	/*
	 * When set to 1, then FEC autonegotiation is supported on this port.
	 * When set to 0, then FEC autonegotiation is not supported on this
	 * port.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_AUTONEG_SUPPORTED UINT32_C(0x2)
	/*
	 * When set to 1, then FEC autonegotiation is enabled on this port. When
	 * set to 0, then FEC autonegotiation is disabled if supported. This
	 * flag should be ignored if FEC autonegotiation is not supported on
	 * this port.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_AUTONEG_ENABLED UINT32_C(0x4)
	/*
	 * When set to 1, then FEC CLAUSE 74 (Fire Code) is supported on this
	 * port. When set to 0, then FEC CLAUSE 74 (Fire Code) is not supported
	 * on this port.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_CLAUSE74_SUPPORTED UINT32_C(0x8)
	/*
	 * When set to 1, then FEC CLAUSE 74 (Fire Code) is enabled on this
	 * port. When set to 0, then FEC CLAUSE 74 (Fire Code) is disabled if
	 * supported. This flag should be ignored if FEC CLAUSE 74 is not
	 * supported on this port.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_CLAUSE74_ENABLED UINT32_C(0x10)
	/*
	 * When set to 1, then FEC CLAUSE 91 (Reed Solomon) is supported on this
	 * port. When set to 0, then FEC CLAUSE 91 (Reed Solomon) is not
	 * supported on this port.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_CLAUSE91_SUPPORTED UINT32_C(0x20)
	/*
	 * When set to 1, then FEC CLAUSE 91 (Reed Solomon) is enabled on this
	 * port. When set to 0, then FEC CLAUSE 91 (Reed Solomon) is disabled if
	 * supported. This flag should be ignored if FEC CLAUSE 91 is not
	 * supported on this port.
	 */
	#define HWRM_PORT_PHY_QCFG_OUTPUT_FEC_CFG_FEC_CLAUSE91_ENABLED UINT32_C(0x40)
	uint8_t unused_1;
	uint8_t unused_2;
	char phy_vendor_name[16];
	/*
	 * Up to 16 bytes of null padded ASCII string representing PHY vendor.
	 * If the string is set to null, then the vendor name is not available.
	 */
	char phy_vendor_partnumber[16];
	/*
	 * Up to 16 bytes of null padded ASCII string that identifies vendor
	 * specific part number of the PHY. If the string is set to null, then
	 * the vendor specific part number is not available.
	 */
	uint32_t unused_3;
	uint8_t unused_4;
	uint8_t unused_5;
	uint8_t unused_6;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_mac_cfg */
/* Description: This command configures the MAC block for the port. */
/* Input (40 bytes) */

struct hwrm_port_mac_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', this command will configure the MAC to match
	 * the current link state of the PHY. If the link is not established on
	 * the PHY, then this bit has no effect.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_MATCH_LINK	UINT32_C(0x1)
	/*
	 * When this bit is '1', the CoS assignment logic is enabled. When this
	 * logic is enabled, then inner VLAN PRI to CoS mapping is enabled. If
	 * this bit is '0', then the default CoS is used.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_COS_ASSIGNMENT_ENABLE UINT32_C(0x2)
	/*
	 * When this bit is '1', tunnel or outer VLAN PRI field to CoS mapping
	 * is enabled. If this bit is '0', then outer VLAN PRI bits are not used
	 * in determining CoS.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_TUNNEL_PRI2COS_ENABLE UINT32_C(0x4)
	/*
	 * When this bit is '1', the IP DSCP to CoS mapping is enabled. If this
	 * bit is '0', then IP DSCP bits are not used in determining CoS.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_IP_DSCP2COS_ENABLE   UINT32_C(0x8)
	/*
	 * When this bit is '1', the HWRM is requested to enable timestamp
	 * capture capability on the receive side of this port.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_PTP_RX_TS_CAPTURE_ENABLE UINT32_C(0x10)
	/*
	 * When this bit is '1', the HWRM is requested to disable timestamp
	 * capture capability on the receive side of this port.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_PTP_RX_TS_CAPTURE_DISABLE UINT32_C(0x20)
	/*
	 * When this bit is '1', the HWRM is requested to enable timestamp
	 * capture capability on the transmit side of this port.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_PTP_TX_TS_CAPTURE_ENABLE UINT32_C(0x40)
	/*
	 * When this bit is '1', the HWRM is requested to disable timestamp
	 * capture capability on the transmit side of this port.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_PTP_TX_TS_CAPTURE_DISABLE UINT32_C(0x80)
	/*
	 * When this bit is '1', the Out-Of-Box WoL is requested to be enabled
	 * on this port.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_OOB_WOL_ENABLE	UINT32_C(0x100)
	/*
	 * When this bit is '1', the the Out-Of-Box WoL is requested to be
	 * disabled on this port.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_FLAGS_OOB_WOL_DISABLE	UINT32_C(0x200)
	uint32_t enables;
	/* This bit must be '1' for the ipg field to be configured. */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_IPG		UINT32_C(0x1)
	/* This bit must be '1' for the lpbk field to be configured. */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_LPBK		UINT32_C(0x2)
	/*
	 * This bit must be '1' for the ivlan_pri2cos_map_pri field to be
	 * configured.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_IVLAN_PRI2COS_MAP_PRI UINT32_C(0x4)
	/* This bit must be '1' for the lcos_map_pri field to be configured. */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_LCOS_MAP_PRI	UINT32_C(0x8)
	/*
	 * This bit must be '1' for the tunnel_pri2cos_map_pri field to be
	 * configured.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_TUNNEL_PRI2COS_MAP_PRI UINT32_C(0x10)
	/* This bit must be '1' for the dscp2cos_map_pri field to be configured. */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_DSCP2COS_MAP_PRI   UINT32_C(0x20)
	/*
	 * This bit must be '1' for the rx_ts_capture_ptp_msg_type field to be
	 * configured.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_RX_TS_CAPTURE_PTP_MSG_TYPE UINT32_C(0x40)
	/*
	 * This bit must be '1' for the tx_ts_capture_ptp_msg_type field to be
	 * configured.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_ENABLES_TX_TS_CAPTURE_PTP_MSG_TYPE UINT32_C(0x80)
	uint16_t port_id;
	/* Port ID of port that is to be configured. */
	uint8_t ipg;
	/*
	 * This value is used to configure the minimum IPG that will be sent
	 * between packets by this port.
	 */
	uint8_t lpbk;
	/* This value controls the loopback setting for the MAC. */
	/* No loopback is selected. Normal operation. */
	#define HWRM_PORT_MAC_CFG_INPUT_LPBK_NONE		(UINT32_C(0x0) << 0)
	/*
	 * The HW will be configured with local loopback such that host
	 * data is sent back to the host without modification.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_LPBK_LOCAL		(UINT32_C(0x1) << 0)
	/*
	 * The HW will be configured with remote loopback such that port
	 * logic will send packets back out the transmitter that are
	 * received.
	 */
	#define HWRM_PORT_MAC_CFG_INPUT_LPBK_REMOTE		(UINT32_C(0x2) << 0)
	uint8_t ivlan_pri2cos_map_pri;
	/*
	 * This value controls the priority of mapping. Valid values: 1-4 Higher
	 * the number, higher the priority
	 */
	uint8_t lcos_map_pri;
	/*
	 * This value controls the priority of mapping. Valid values: 1-4 Higher
	 * the number, higher the priority
	 */
	uint8_t tunnel_pri2cos_map_pri;
	/*
	 * This value controls the priority of mapping. Valid values: 1-4 Higher
	 * the number, higher the priority
	 */
	uint8_t dscp2pri_map_pri;
	/*
	 * This value controls the priority of mapping. Valid values: 1-4 Higher
	 * the number, higher the priority
	 */
	uint16_t rx_ts_capture_ptp_msg_type;
	/*
	 * This is a 16-bit bit mask that is used to request a specific
	 * configuration of time stamp capture of PTP messages on the receive
	 * side of this port. This field shall be ignored if the
	 * ptp_rx_ts_capture_enable flag is not set in this command. Otherwise,
	 * if bit 'i' is set, then the HWRM is being requested to configure the
	 * receive side of the port to capture the time stamp of every received
	 * PTP message with messageType field value set to i.
	 */
	uint16_t tx_ts_capture_ptp_msg_type;
	/*
	 * This is a 16-bit bit mask that is used to request a specific
	 * configuration of time stamp capture of PTP messages on the transmit
	 * side of this port. This field shall be ignored if the
	 * ptp_tx_ts_capture_enable flag is not set in this command. Otherwise,
	 * if bit 'i' is set, then the HWRM is being requested to configure the
	 * transmit sied of the port to capture the time stamp of every
	 * transmitted PTP message with messageType field value set to i.
	 */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_port_mac_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t mru;
	/*
	 * This is the configured maximum length of Ethernet packet payload that
	 * is allowed to be received on the port. This value does not include
	 * the number of bytes used by Ethernet header and trailer (CRC).
	 */
	uint16_t mtu;
	/*
	 * This is the configured maximum length of Ethernet packet payload that
	 * is allowed to be transmitted on the port. This value does not include
	 * the number of bytes used by Ethernet header and trailer (CRC).
	 */
	uint8_t ipg;
	/* Current configuration of the IPG value. */
	uint8_t lpbk;
	/* Current value of the loopback value. */
	/* No loopback is selected. Normal operation. */
	#define HWRM_PORT_MAC_CFG_OUTPUT_LPBK_NONE		(UINT32_C(0x0) << 0)
	/*
	 * The HW will be configured with local loopback such that host
	 * data is sent back to the host without modification.
	 */
	#define HWRM_PORT_MAC_CFG_OUTPUT_LPBK_LOCAL		(UINT32_C(0x1) << 0)
	/*
	 * The HW will be configured with remote loopback such that port
	 * logic will send packets back out the transmitter that are
	 * received.
	 */
	#define HWRM_PORT_MAC_CFG_OUTPUT_LPBK_REMOTE		(UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_mac_qcfg */
/* Description: This command queries the MAC block for the port. */
/* Input (24 bytes) */

struct hwrm_port_mac_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/* Port ID of port that is to be configured. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_port_mac_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t mru;
	/*
	 * This is the configured maximum length of Ethernet packet payload that
	 * is allowed to be received on the port. This value does not include
	 * the number of bytes used by the Ethernet header and trailer (CRC).
	 */
	uint16_t mtu;
	/*
	 * This is the configured maximum length of Ethernet packet payload that
	 * is allowed to be transmitted on the port. This value does not include
	 * the number of bytes used by the Ethernet header and trailer (CRC).
	 */
	uint8_t ipg;
	/* The minimum IPG that will be sent between packets by this port. */
	uint8_t lpbk;
	/* The loopback setting for the MAC. */
	/* No loopback is selected. Normal operation. */
	#define HWRM_PORT_MAC_QCFG_OUTPUT_LPBK_NONE		(UINT32_C(0x0) << 0)
	/*
	 * The HW will be configured with local loopback such that host
	 * data is sent back to the host without modification.
	 */
	#define HWRM_PORT_MAC_QCFG_OUTPUT_LPBK_LOCAL		(UINT32_C(0x1) << 0)
	/*
	 * The HW will be configured with remote loopback such that port
	 * logic will send packets back out the transmitter that are
	 * received.
	 */
	#define HWRM_PORT_MAC_QCFG_OUTPUT_LPBK_REMOTE		(UINT32_C(0x2) << 0)
	uint8_t ivlan_pri2cos_map_pri;
	/*
	 * Priority of pri to CoS mapping. Valid values: 1-4 Higher the number,
	 * higher the priority Value 0 indicates that this mapping is not used.
	 */
	uint8_t lcos_map_pri;
	/*
	 * Priority of local CoS to PRI mapping. Valid values: 1-4 Higher the
	 * number, higher the priority Value 0 indicates that this mapping is
	 * not used.
	 */
	uint8_t tunnel_pri2cos_map_pri;
	/*
	 * Priority of tunnel PRI to CoS mapping. Valid values: 1-4 Higher the
	 * number, higher the priority Value 0 indicates that this mapping is
	 * not used.
	 */
	uint8_t dscp2pri_map_pri;
	/*
	 * Priority of DSCP to PRI mapping. Valid values: 1-4 Higher the number,
	 * higher the priority Value 0 indicates that this mapping is not used.
	 */
	uint16_t rx_ts_capture_ptp_msg_type;
	/*
	 * This is a 16-bit bit mask that represents the current configuration
	 * of time stamp capture of PTP messages on the receive side of this
	 * port. If bit 'i' is set, then the receive side of the port is
	 * configured to capture the time stamp of every received PTP message
	 * with messageType field value set to i. If all bits are set to 0 (i.e.
	 * field value set 0), then the receive side of the port is not
	 * configured to capture timestamp for PTP messages. If all bits are set
	 * to 1, then the receive side of the port is configured to capture
	 * timestamp for all PTP messages.
	 */
	uint16_t tx_ts_capture_ptp_msg_type;
	/*
	 * This is a 16-bit bit mask that represents the current configuration
	 * of time stamp capture of PTP messages on the transmit side of this
	 * port. If bit 'i' is set, then the transmit side of the port is
	 * configured to capture the time stamp of every received PTP message
	 * with messageType field value set to i. If all bits are set to 0 (i.e.
	 * field value set 0), then the transmit side of the port is not
	 * configured to capture timestamp for PTP messages. If all bits are set
	 * to 1, then the transmit side of the port is configured to capture
	 * timestamp for all PTP messages.
	 */
	uint8_t unused_0;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_qstats */
/* Description: This function returns per port Ethernet statistics. */
/* Input (40 bytes) */

struct hwrm_port_qstats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/* Port ID of port that is being queried. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2[3];
	uint8_t unused_3;
	uint64_t tx_stat_host_addr;
	/* This is the host address where Tx port statistics will be stored */
	uint64_t rx_stat_host_addr;
	/* This is the host address where Rx port statistics will be stored */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_port_qstats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t tx_stat_size;
	/* The size of TX port statistics block in bytes. */
	uint16_t rx_stat_size;
	/* The size of RX port statistics block in bytes. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_lpbk_qstats */
/* Description: This function returns loopback statistics. */
/* Input (16 bytes) */

struct hwrm_port_lpbk_qstats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (96 bytes) */

struct hwrm_port_lpbk_qstats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t lpbk_ucast_frames;
	/* Number of transmitted unicast frames */
	uint64_t lpbk_mcast_frames;
	/* Number of transmitted multicast frames */
	uint64_t lpbk_bcast_frames;
	/* Number of transmitted broadcast frames */
	uint64_t lpbk_ucast_bytes;
	/* Number of transmitted bytes for unicast traffic */
	uint64_t lpbk_mcast_bytes;
	/* Number of transmitted bytes for multicast traffic */
	uint64_t lpbk_bcast_bytes;
	/* Number of transmitted bytes for broadcast traffic */
	uint64_t tx_stat_discard;
	/* Total Tx Drops for loopback traffic reported by STATS block */
	uint64_t tx_stat_error;
	/* Total Tx Error Drops for loopback traffic reported by STATS block */
	uint64_t rx_stat_discard;
	/* Total Rx Drops for loopback traffic reported by STATS block */
	uint64_t rx_stat_error;
	/* Total Rx Error Drops for loopback traffic reported by STATS block */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_clr_stats */
/*
 * Description: This function clears per port statistics. The HWRM shall not
 * allow a VF driver to clear port statistics. The HWRM shall not allow a PF
 * driver to clear port statistics in a partitioning mode. The HWRM may allow a
 * PF driver to clear port statistics in the non-partitioning mode.
 */
/* Input (24 bytes) */

struct hwrm_port_clr_stats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/* Port ID of port that is being queried. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_port_clr_stats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_lpbk_clr_stats */
/*
 * Description: This function clears loopback statistics. The HWRM shall not
 * allow a VF driver to clear loopback statistics. The HWRM shall not allow a PF
 * driver to clear loopback statistics in a partitioning mode. The HWRM may
 * allow a PF driver to clear loopback statistics in the non-partitioning mode.
 */
/* Input (16 bytes) */

struct hwrm_port_lpbk_clr_stats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_port_lpbk_clr_stats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_blink_led */
/*
 * Description: This function blinks the port LED for the specified number of
 * times.
 */
/* Input (24 bytes) */

struct hwrm_port_blink_led_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t num_blinks;
	/* Number of blinks. */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_port_blink_led_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_ts_query */
/*
 * Description: This function is used to read timestamp information captured for
 * PTP messages on this port.
 */
/* Input (24 bytes) */

struct hwrm_port_ts_query_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_PORT_TS_QUERY_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_PORT_TS_QUERY_INPUT_FLAGS_PATH_TX		(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_PORT_TS_QUERY_INPUT_FLAGS_PATH_RX		(UINT32_C(0x1) << 0)
	#define HWRM_PORT_TS_QUERY_INPUT_FLAGS_PATH_LAST	HWRM_PORT_TS_QUERY_INPUT_FLAGS_PATH_RX
	uint16_t port_id;
	/* Port ID of port that is being queried. */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_port_ts_query_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t ptp_msg_ts;
	/* Timestamp value of PTP message captured. */
	uint16_t ptp_msg_seqid;
	/* Sequence ID of the PTP message captured. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_port_phy_qcaps */
/*
 * Description: This function is used to query the current capabilities of PHY
 * on this link.
 */
/* Input (24 bytes) */

struct hwrm_port_phy_qcaps_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/* Port ID of port that is being queried. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_port_phy_qcaps_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t eee_supported;
	/*
	 * Reserved field. The HWRM shall set this field to 0. An HWRM client
	 * shall ignore this field.
	 */
	/*
	 * If set to 1, then this field indicates that the link is capable of
	 * supporting EEE.
	 */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_EEE_SUPPORTED	UINT32_C(0x1)
	/*
	 * Reserved field. The HWRM shall set this field to 0. An HWRM client
	 * shall ignore this field.
	 */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_RSVD1_MASK		UINT32_C(0xfe)
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_RSVD1_SFT		1
	uint8_t unused_0;
	uint16_t supported_speeds_force_mode;
	/*
	 * This is a bit mask to indicate what speeds are supported as forced
	 * speeds on this link. For each speed that can be forced on this link,
	 * the corresponding mask bit shall be set to '1'.
	 */
	/* 100Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_100MBHD UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_100MB UINT32_C(0x2)
	/* 1Gb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_1GBHD UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_1GB UINT32_C(0x8)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_2GB UINT32_C(0x10)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_2_5GB UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_10GB UINT32_C(0x40)
	/* 20Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_20GB UINT32_C(0x80)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_25GB UINT32_C(0x100)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_40GB UINT32_C(0x200)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_50GB UINT32_C(0x400)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_100GB UINT32_C(0x800)
	/* 10Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_10MBHD UINT32_C(0x1000)
	/* 10Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_FORCE_MODE_10MB UINT32_C(0x2000)
	uint16_t supported_speeds_auto_mode;
	/*
	 * This is a bit mask to indicate what speeds are supported for
	 * autonegotiation on this link. For each speed that can be
	 * autonegotiated on this link, the corresponding mask bit shall be set
	 * to '1'.
	 */
	/* 100Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_100MBHD UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_100MB UINT32_C(0x2)
	/* 1Gb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_1GBHD UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_1GB UINT32_C(0x8)
	/* 2Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_2GB UINT32_C(0x10)
	/* 2.5Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_2_5GB UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_10GB UINT32_C(0x40)
	/* 20Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_20GB UINT32_C(0x80)
	/* 25Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_25GB UINT32_C(0x100)
	/* 40Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_40GB UINT32_C(0x200)
	/* 50Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_50GB UINT32_C(0x400)
	/* 100Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_100GB UINT32_C(0x800)
	/* 10Mb link speed (Half-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_10MBHD UINT32_C(0x1000)
	/* 10Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_AUTO_MODE_10MB UINT32_C(0x2000)
	uint16_t supported_speeds_eee_mode;
	/*
	 * This is a bit mask to indicate what speeds are supported for EEE on
	 * this link. For each speed that can be autonegotiated when EEE is
	 * enabled on this link, the corresponding mask bit shall be set to '1'.
	 * This field is only valid when the eee_suppotred is set to '1'.
	 */
	/* Reserved */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_RSVD1 UINT32_C(0x1)
	/* 100Mb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_100MB UINT32_C(0x2)
	/* Reserved */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_RSVD2 UINT32_C(0x4)
	/* 1Gb link speed (Full-duplex) */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_1GB UINT32_C(0x8)
	/* Reserved */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_RSVD3 UINT32_C(0x10)
	/* Reserved */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_RSVD4 UINT32_C(0x20)
	/* 10Gb link speed */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_SUPPORTED_SPEEDS_EEE_MODE_10GB UINT32_C(0x40)
	uint32_t tx_lpi_timer_low;
	/*
	 * Reserved field. The HWRM shall set this field to 0. An HWRM client
	 * shall ignore this field.
	 */
	/*
	 * The lowest value of TX LPI timer that can be set on this link when
	 * EEE is enabled. This value is in microseconds. This field is valid
	 * only when_eee_supported is set to '1'.
	 */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_TX_LPI_TIMER_LOW_MASK   UINT32_C(0xffffff)
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_TX_LPI_TIMER_LOW_SFT	0
	/*
	 * Reserved field. The HWRM shall set this field to 0. An HWRM client
	 * shall ignore this field.
	 */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_RSVD2_MASK		UINT32_C(0xff000000)
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_RSVD2_SFT		24
	uint32_t valid_tx_lpi_timer_high;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
	/*
	 * The highest value of TX LPI timer that can be set on this link when
	 * EEE is enabled. This value is in microseconds. This field is valid
	 * only when_eee_supported is set to '1'.
	 */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_TX_LPI_TIMER_HIGH_MASK  UINT32_C(0xffffff)
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_TX_LPI_TIMER_HIGH_SFT   0
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_VALID_MASK		UINT32_C(0xff000000)
	#define HWRM_PORT_PHY_QCAPS_OUTPUT_VALID_SFT		24
} __attribute__((packed));

/* hwrm_queue_qportcfg */
/*
 * Description: This function is called by a driver to query queue configuration
 * of a port. # The HWRM shall at least advertise one queue with lossy service
 * profile. # The driver shall use this command to query queue ids before
 * configuring or using any queues. # If a service profile is not set for a
 * queue, then the driver shall not use that queue without configuring a service
 * profile for it. # If the driver is not allowed to configure service profiles,
 * then the driver shall only use queues for which service profiles are pre-
 * configured.
 */
/* Input (24 bytes) */

struct hwrm_queue_qportcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_QPORTCFG_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_QPORTCFG_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_QPORTCFG_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_QPORTCFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_QPORTCFG_INPUT_FLAGS_PATH_RX
	uint16_t port_id;
	/*
	 * Port ID of port for which the queue configuration is being queried.
	 * This field is only required when sent by IPC.
	 */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_queue_qportcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t max_configurable_queues;
	/* The maximum number of queues that can be configured. */
	uint8_t max_configurable_lossless_queues;
	/* The maximum number of lossless queues that can be configured. */
	uint8_t queue_cfg_allowed;
	/*
	 * 0 - Not allowed. Non-zero - Allowed. If this value is non-zero, then
	 * the HWRM shall allow the host SW driver to configure queues using
	 * hwrm_queue_cfg.
	 */
	uint8_t queue_buffers_cfg_allowed;
	/*
	 * 0 - Not allowed. Non-zero - Allowed If this value is non-zero, then
	 * the HWRM shall allow the host SW driver to configure queue buffers
	 * using hwrm_queue_buffers_cfg.
	 */
	uint8_t queue_pfcenable_cfg_allowed;
	/*
	 * 0 - Not allowed. Non-zero - Allowed If this value is non-zero, then
	 * the HWRM shall allow the host SW driver to configure PFC using
	 * hwrm_queue_pfcenable_cfg.
	 */
	uint8_t queue_pri2cos_cfg_allowed;
	/*
	 * 0 - Not allowed. Non-zero - Allowed If this value is non-zero, then
	 * the HWRM shall allow the host SW driver to configure Priority to CoS
	 * mapping using hwrm_queue_pri2cos_cfg.
	 */
	uint8_t queue_cos2bw_cfg_allowed;
	/*
	 * 0 - Not allowed. Non-zero - Allowed If this value is non-zero, then
	 * the HWRM shall allow the host SW driver to configure CoS Bandwidth
	 * configuration using hwrm_queue_cos2bw_cfg.
	 */
	uint8_t queue_id0;
	/* ID of CoS Queue 0. FF - Invalid id */
	uint8_t queue_id0_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID0_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID0_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID0_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id1;
	/* ID of CoS Queue 1. FF - Invalid id */
	uint8_t queue_id1_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID1_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID1_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID1_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id2;
	/* ID of CoS Queue 2. FF - Invalid id */
	uint8_t queue_id2_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID2_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID2_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID2_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id3;
	/* ID of CoS Queue 3. FF - Invalid id */
	uint8_t queue_id3_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID3_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID3_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID3_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id4;
	/* ID of CoS Queue 4. FF - Invalid id */
	uint8_t queue_id4_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID4_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID4_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID4_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id5;
	/* ID of CoS Queue 5. FF - Invalid id */
	uint8_t queue_id5_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID5_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID5_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID5_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id6;
	/* ID of CoS Queue 6. FF - Invalid id */
	uint8_t queue_id6_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID6_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID6_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID6_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t queue_id7;
	/* ID of CoS Queue 7. FF - Invalid id */
	uint8_t queue_id7_service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID7_SERVICE_PROFILE_LOSSY (UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID7_SERVICE_PROFILE_LOSSLESS (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QPORTCFG_OUTPUT_QUEUE_ID7_SERVICE_PROFILE_UNKNOWN (UINT32_C(0xff) << 0)
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_qcfg */
/*
 * Description: This function is called by a driver to query a queue
 * configuration.
 */
/* Input (24 bytes) */

struct hwrm_queue_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_QCFG_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_QCFG_INPUT_FLAGS_PATH_TX		(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_QCFG_INPUT_FLAGS_PATH_RX		(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_QCFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_QCFG_INPUT_FLAGS_PATH_RX
	uint32_t queue_id;
	/* Queue ID of the queue. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t queue_len;
	/* This value is a the estimate packet length used in the TX arbiter. */
	uint8_t service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_QCFG_OUTPUT_SERVICE_PROFILE_LOSSY	(UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_QCFG_OUTPUT_SERVICE_PROFILE_LOSSLESS   (UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_QCFG_OUTPUT_SERVICE_PROFILE_UNKNOWN	(UINT32_C(0xff) << 0)
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_cfg */
/* Description: This function is called by a driver to configure a queue. */
/* Input (40 bytes) */

struct hwrm_queue_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_CFG_INPUT_FLAGS_PATH			UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_CFG_INPUT_FLAGS_PATH_TX		(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_CFG_INPUT_FLAGS_PATH_RX		(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_CFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_CFG_INPUT_FLAGS_PATH_RX
	uint32_t enables;
	/* This bit must be '1' for the dflt_len field to be configured. */
	#define HWRM_QUEUE_CFG_INPUT_ENABLES_DFLT_LEN		UINT32_C(0x1)
	/* This bit must be '1' for the service_profile field to be configured. */
	#define HWRM_QUEUE_CFG_INPUT_ENABLES_SERVICE_PROFILE	UINT32_C(0x2)
	uint32_t queue_id;
	/* Queue ID of queue that is to be configured by this function. */
	uint32_t dflt_len;
	/*
	 * This value is a the estimate packet length used in the TX arbiter.
	 * Set to 0xFF... (All Fs) to not adjust this value.
	 */
	uint8_t service_profile;
	/* This value is applicable to CoS queues only. */
	/* Lossy (best-effort) */
	#define HWRM_QUEUE_CFG_INPUT_SERVICE_PROFILE_LOSSY	(UINT32_C(0x0) << 0)
	/* Lossless */
	#define HWRM_QUEUE_CFG_INPUT_SERVICE_PROFILE_LOSSLESS	(UINT32_C(0x1) << 0)
	/* Set to 0xFF... (All Fs) if there is no service profile specified */
	#define HWRM_QUEUE_CFG_INPUT_SERVICE_PROFILE_UNKNOWN	(UINT32_C(0xff) << 0)
	uint8_t unused_0[7];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_buffers_qcfg */
/*
 * Description: This function is called by a driver to query configuration of
 * the buffers assigned to a queue.
 */
/* Input (24 bytes) */

struct hwrm_queue_buffers_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_BUFFERS_QCFG_INPUT_FLAGS_PATH	UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_BUFFERS_QCFG_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_BUFFERS_QCFG_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_BUFFERS_QCFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_BUFFERS_QCFG_INPUT_FLAGS_PATH_RX
	uint32_t queue_id;
	/* Queue ID of queue that is to be configured by this function. */
} __attribute__((packed));

/* Output (40 bytes) */

struct hwrm_queue_buffers_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t reserved;
	/* Number of bytes allocated as reserved space for this queue. */
	uint32_t shared;
	/*
	 * Number of bytes of shared buffer space for this queue. The changing
	 * of shared buffer size for one CoS may create an adverse effect on
	 * other CoSs sharing the same buffer. It is recommended that the driver
	 * does not modify the shared mbuf size without understanding the
	 * consequence of it.
	 */
	uint32_t xoff;
	/*
	 * XOFF threshold of the queue. This is a high threshold value used to
	 * trigger XOFF.
	 */
	uint32_t xon;
	/*
	 * XON threshold of the queue. This is the low threshold value used to
	 * trigger XON.
	 */
	uint32_t full;
	/*
	 * FULL threshold of the queue. At this threshold, buffers allocated for
	 * this queue are full. Once this condition is asserted, packets on that
	 * queue are dropped.
	 */
	uint32_t notfull;
	/*
	 * NOTFULL threshold of the queue. This threshold is used for the de-
	 * assertion of buffers full condition.
	 */
	uint32_t max;
	/*
	 * The maximum number of bytes that will be allowed to be consumed by
	 * the queue. This value is the sum of both the number of bytes reserved
	 * for this queue and the maximum number of bytes of shared buffers
	 * allowed to be consumed by this queue.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_buffers_cfg */
/*
 * Description: This function is called by a driver to configure the buffering
 * for a queue.
 */
/* Input (56 bytes) */

struct hwrm_queue_buffers_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_BUFFERS_CFG_INPUT_FLAGS_PATH_RX
	uint32_t enables;
	/* This bit must be '1' for the reserved field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_RESERVED	UINT32_C(0x1)
	/* This bit must be '1' for the shared field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_SHARED	UINT32_C(0x2)
	/* This bit must be '1' for the xoff field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_XOFF	UINT32_C(0x4)
	/* This bit must be '1' for the xon field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_XON	UINT32_C(0x8)
	/* This bit must be '1' for the full field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_FULL	UINT32_C(0x10)
	/* This bit must be '1' for the notfull field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_NOTFULL	UINT32_C(0x20)
	/* This bit must be '1' for the max field to be configured. */
	#define HWRM_QUEUE_BUFFERS_CFG_INPUT_ENABLES_MAX	UINT32_C(0x40)
	uint32_t queue_id;
	/* Queue ID of queue that is to be configured by this function. */
	uint32_t reserved;
	/* Number of bytes to be allocated as reserved space for this queue. */
	uint32_t shared;
	/*
	 * Number of bytes of shared buffer space for this queue. The changing
	 * of shared buffer size for one CoS may create an adverse effect on
	 * other CoSs sharing the same buffer. It is recommended that the driver
	 * does not modify the shared mbuf size without understanding the
	 * consequence of it.
	 */
	uint32_t xoff;
	/*
	 * XOFF threshold of the queue. This is a high threshold value used to
	 * trigger XOFF.
	 */
	uint32_t xon;
	/*
	 * XON threshold of the queue. This is the low threshold value used to
	 * trigger XON.
	 */
	uint32_t full;
	/*
	 * FULL threshold of the queue. At this threshold, buffers allocated for
	 * this queue are full. Once this condition is asserted, packets on that
	 * queue are dropped.
	 */
	uint32_t notfull;
	/*
	 * NOTFULL threshold of the queue. This threshold is used for the de-
	 * assertion of buffers full condition.
	 */
	uint32_t max;
	/*
	 * The maximum number of bytes that will be allowed to be consumed by
	 * the queue. This value is the sum of both the number of bytes reserved
	 * for this queue and the maximum number of bytes of shared buffers
	 * allowed to be consumed by this queue.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_buffers_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_pfcenable_qcfg */
/*
 * Description: This function is called by a driver to query PFC configuration
 * for different priorities on that port. This mapping can be different on
 * different ports.
 */
/* Input (24 bytes) */

struct hwrm_queue_pfcenable_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/*
	 * Port ID of port for which the table is being configured. The HWRM
	 * needs to check whether this function is allowed to configure pri2cos
	 * mapping on this port.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_pfcenable_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t flags;
	/* If set to 1, then PFC is enabled on PRI 0. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI0_PFC_ENABLED UINT32_C(0x1)
	/* If set to 1, then PFC is enabled on PRI 1. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI1_PFC_ENABLED UINT32_C(0x2)
	/* If set to 1, then PFC is enabled on PRI 2. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI2_PFC_ENABLED UINT32_C(0x4)
	/* If set to 1, then PFC is enabled on PRI 3. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI3_PFC_ENABLED UINT32_C(0x8)
	/* If set to 1, then PFC is enabled on PRI 4. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI4_PFC_ENABLED UINT32_C(0x10)
	/* If set to 1, then PFC is enabled on PRI 5. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI5_PFC_ENABLED UINT32_C(0x20)
	/* If set to 1, then PFC is enabled on PRI 6. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI6_PFC_ENABLED UINT32_C(0x40)
	/* If set to 1, then PFC is enabled on PRI 7. */
	#define HWRM_QUEUE_PFCENABLE_QCFG_OUTPUT_FLAGS_PRI7_PFC_ENABLED UINT32_C(0x80)
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_pfcenable_cfg */
/*
 * Description: This function is called by a driver to configure the PFC enabled
 * for different priorities on that port. This mapping can be different on
 * different ports.
 */
/* Input (24 bytes) */

struct hwrm_queue_pfcenable_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/* If set to 1, then PFC is requested to be enabled on PRI 0. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI0_PFC_ENABLED UINT32_C(0x1)
	/* If set to 1, then PFC is requested to be enabled on PRI 1. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI1_PFC_ENABLED UINT32_C(0x2)
	/* If set to 1, then PFC is requested to be enabled on PRI 2. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI2_PFC_ENABLED UINT32_C(0x4)
	/* If set to 1, then PFC is requested to be enabled on PRI 3. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI3_PFC_ENABLED UINT32_C(0x8)
	/* If set to 1, then PFC is requested to be enabled on PRI 4. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI4_PFC_ENABLED UINT32_C(0x10)
	/* If set to 1, then PFC is requested to be enabled on PRI 5. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI5_PFC_ENABLED UINT32_C(0x20)
	/* If set to 1, then PFC is requested to be enabled on PRI 6. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI6_PFC_ENABLED UINT32_C(0x40)
	/* If set to 1, then PFC is requested to be enabled on PRI 7. */
	#define HWRM_QUEUE_PFCENABLE_CFG_INPUT_FLAGS_PRI7_PFC_ENABLED UINT32_C(0x80)
	uint16_t port_id;
	/*
	 * Port ID of port for which the table is being configured. The HWRM
	 * needs to check whether this function is allowed to configure pri2cos
	 * mapping on this port.
	 */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_pfcenable_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_pri2cos_qcfg */
/*
 * Description: This function is called by a driver to query configuration of
 * the priority to CoS queue mapping on the transmit side and receive side. This
 * mapping can be different in each direction (TX or RX). This mapping can be
 * different on different ports. Each CoS queue represents a Traffic Class (TC)
 * on that port.
 */
/* Input (24 bytes) */

struct hwrm_queue_pri2cos_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_PRI2COS_QCFG_INPUT_FLAGS_PATH	UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_PRI2COS_QCFG_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_PRI2COS_QCFG_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_PRI2COS_QCFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_PRI2COS_QCFG_INPUT_FLAGS_PATH_RX
	/*
	 * When this bit is set to '1', the mapping is requested for inner VLAN
	 * PRI.
	 */
	#define HWRM_QUEUE_PRI2COS_QCFG_INPUT_FLAGS_IVLAN	UINT32_C(0x2)
	uint8_t port_id;
	/*
	 * Port ID of port for which the table is being configured. The HWRM
	 * needs to check whether this function is allowed to configure pri2cos
	 * mapping on this port.
	 */
	uint8_t unused_0[3];
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_queue_pri2cos_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t pri0_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 0. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri1_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 1. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri2_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 2 This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri3_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 3. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri4_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 4. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri5_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 5. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri6_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 6. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri7_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 7. This value can only be changed
	 * before traffic has started.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_pri2cos_cfg */
/*
 * Description: This function is called by a driver to configure the priority to
 * CoS queue mapping on the transmit side and receive side. This mapping can be
 * different in each direction (TX or RX). This mapping can be different on
 * different ports. Each CoS queue represents a Traffic Class (TC) on that port.
 * This command configures the VLAN PRI-to-TC mapping for a specific port in
 * specific direction.
 */
/* Input (40 bytes) */

struct hwrm_queue_pri2cos_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_QUEUE_PRI2COS_CFG_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_QUEUE_PRI2COS_CFG_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_QUEUE_PRI2COS_CFG_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_QUEUE_PRI2COS_CFG_INPUT_FLAGS_PATH_LAST	HWRM_QUEUE_PRI2COS_CFG_INPUT_FLAGS_PATH_RX
	/* When this bit is '1', the mapping is for inner VLAN PRI. */
	#define HWRM_QUEUE_PRI2COS_CFG_INPUT_FLAGS_IVLAN	UINT32_C(0x2)
	uint32_t enables;
	uint8_t port_id;
	/*
	 * Port ID of port for which the table is being configured. The HWRM
	 * needs to check whether this function is allowed to configure pri2cos
	 * mapping on this port.
	 */
	uint8_t pri0_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 0. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri1_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 1. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri2_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 2 This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri3_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 3. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri4_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 4. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri5_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 5. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri6_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 6. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t pri7_cos_queue_id;
	/*
	 * CoS Queue assigned to priority 7. This value can only be changed
	 * before traffic has started.
	 */
	uint8_t unused_0[7];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_pri2cos_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_cos2bw_qcfg */
/*
 * Description: This function is called by a driver to query the BW to CoS queue
 * mapping on the transmit side of a specific port. This mapping can be
 * different on different ports. Each CoS queue represents a Traffic Class (TC)
 * on that port. Each traffic class can be assigned a valid combination of the
 * following: - Minimum bandwidth - Maximum bandwidth - Transmission selection
 * algorithm (TSA) - Priority Level (only applies to strict priority COS) -
 * Bandwidth weight # A CoS can be SP or non-SP: A SP CoS always gets the strict
 * priority. Is an COS min BW is set to 0x0 then it is considered to be non-SP;
 * this is a valid configuration. Note: SP provides lower latency in addition to
 * reserved bandwidth # For both SP CoS and non-SP CoS, min BW can be specified
 * to reserve specific amount of the port BW. # The min BW specified for a CoS
 * shall not exceed max port bandwidth. # The total of min BWs specified for all
 * CoS shall not exceed max port bandwidth. # For any non-SP CoS, the minimum
 * bandwidth guarantees are subject to round-robin scheduling. This allows BW
 * reservation with anti-starvation; one CoS will not block another CoS using
 * RR. Note: The bandwidth guarantees for any non-SP CoS are met after servicing
 * all SP CoS. # An SP CoS can potentially starve other lower priority SP CoS
 * and non-SP CoS queues. This can occur to the extent the SP min exceeds the
 * available port BW. # For any CoS, max BW can be specified to limit the BW
 * consumed by the CoS. # The max BW specified for a CoS shall not exceed the
 * max port bandwidth. # The WFQ provides a mechanism for sharing available
 * bandwidth beyond the reserved minimums configured for each CoS. The WFQ
 * scheduler is used to provide the percentages of remaining bandwidth after: *
 * first servicing the reserved bandwidth for all SP CoS, * followed by the
 * reserved bandwidth for all non-SP CoS * All CoS may participate in the WFQ #
 * If a CoS does not have a configured max BW it may use all available bandwidth
 * up to the max port bandwidth Minimum Bandwidth (min BW): # This is the
 * guaranteed bandwidth for the COS. # A value of 0x0 is valid and it means that
 * this COS is not guaranteed any bandwidth. A value of 0xFF.. (all Fs) means
 * min BW is not specified. When the min BW is not specified, the HWRM can set
 * it to any value it considers appropriate. Note: For a non-SP COS, the HWRM
 * should set min BW to 0 when the min BW is not specified. For an SP COS, the
 * HWRM should set min BW to some small value when the min BW is not specified.
 * Maximum Bandwidth: # This is the bandwidth limit of the COS. # Values 0x0 and
 * 0xFF.. (all Fs) are considered unspecified and the HWRM will set the maximum
 * bandwidth to maximum port bandwidth. Priority Level: # It applies only to SP.
 * # This parameter is ignored for non-SP. # 0-7 are valid values (higher value
 * means higher priority) # A priority level can be assigned to at most one SP.
 * # Invalid priority levels assignment for SPs shall result in failure.
 * Additional notes: # The HWRM may have to use min and (max - min) to set
 * appropriate counters of hardware rate limiters. # The bandwidth percentage as
 * specified in the DCB TC BW assignment should be used by the driver to specify
 * maximum bandwidth and bandwidth weight for a COS. For example, the driver
 * should set max BW to 20 Gbps and weight to 50 for two COSs when these two
 * COSs are assigned 50% share of 40 Gbps max port bandwidth.
 */
/* Input (24 bytes) */

struct hwrm_queue_cos2bw_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/*
	 * Port ID of port for which the table is being configured. The HWRM
	 * needs to check whether this function is allowed to configure TC BW
	 * assignment on this port.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (112 bytes) */

struct hwrm_queue_cos2bw_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t queue_id0;
	/* ID of CoS Queue 0. */
	uint8_t unused_0;
	uint16_t unused_1;
	uint32_t queue_id0_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id0_max_bw;
	/*
	 * Maximum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id0_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID0_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID0_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID0_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID0_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id0_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id0_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id1;
	/* ID of CoS Queue 1. */
	uint32_t queue_id1_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id1_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id1_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID1_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID1_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID1_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID1_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id1_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id1_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id2;
	/* ID of CoS Queue 2. */
	uint32_t queue_id2_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id2_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id2_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID2_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID2_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID2_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID2_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id2_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id2_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id3;
	/* ID of CoS Queue 3. */
	uint32_t queue_id3_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id3_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id3_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID3_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID3_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID3_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID3_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id3_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id3_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id4;
	/* ID of CoS Queue 4. */
	uint32_t queue_id4_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id4_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id4_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID4_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID4_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID4_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID4_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id4_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id4_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id5;
	/* ID of CoS Queue 5. */
	uint32_t queue_id5_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id5_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id5_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID5_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID5_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID5_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID5_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id5_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id5_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id6;
	/* ID of CoS Queue 6. */
	uint32_t queue_id6_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id6_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id6_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID6_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID6_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID6_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID6_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id6_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id6_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id7;
	/* ID of CoS Queue 7. */
	uint32_t queue_id7_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id7_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id7_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID7_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID7_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID7_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_QCFG_OUTPUT_QUEUE_ID7_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id7_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id7_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t unused_5;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_queue_cos2bw_cfg */
/*
 * Description: This function is called by a driver to configure the BW to CoS
 * queue mapping on the transmit side of a specific port. This mapping can be
 * different on different ports. Each CoS queue represents a Traffic Class (TC)
 * on that port. Each traffic class can be assigned a valid combination of the
 * following: - Minimum bandwidth - Maximum bandwidth - Transmission selection
 * algorithm (TSA) - Priority Level (only applies to strict priority COS) -
 * Bandwidth weight # A CoS can be SP or non-SP: A SP CoS always gets the strict
 * priority. Is an COS min BW is set to 0x0 then it is considered to be non-SP;
 * this is a valid configuration. Note: SP provides lower latency in addition to
 * reserved bandwidth # For both SP CoS and non-SP CoS, min BW can be specified
 * to reserve specific amount of the port BW. # The min BW specified for a CoS
 * shall not exceed max port bandwidth. # The total of min BWs specified for all
 * CoS shall not exceed max port bandwidth. # For any non-SP CoS, the minimum
 * bandwidth guarantees are subject to round-robin scheduling. This allows BW
 * reservation with anti-starvation; one CoS will not block another CoS using
 * RR. Note: The bandwidth guarantees for any non-SP CoS are met after servicing
 * all SP CoS. # An SP CoS can potentially starve other lower priority SP CoS
 * and non-SP CoS queues. This can occur to the extent the SP min exceeds the
 * available port BW. # For any CoS, max BW can be specified to limit the BW
 * consumed by the CoS. # The max BW specified for a CoS shall not exceed the
 * max port bandwidth. # The WFQ provides a mechanism for sharing available
 * bandwidth beyond the reserved minimums configured for each CoS. The WFQ
 * scheduler is used to provide the percentages of remaining bandwidth after: -
 * first servicing the reserved bandwidth for all SP CoS, - followed by the
 * reserved bandwidth for all non-SP CoS - All CoS may participate in the WFQ #
 * If a CoS does not have a configured max BW it may use all available bandwidth
 * up to the max port bandwidth Minimum Bandwidth (min BW): # This is the
 * guaranteed bandwidth for the COS. # A value of 0x0 is valid and it means that
 * this COS is not guaranteed any bandwidth. A value of 0xFF.. (all Fs) means
 * min BW is not specified. When the min BW is not specified, the HWRM can set
 * it to any value it considers appropriate. Note: For a non-SP COS, the HWRM
 * should set min BW to 0 when the min BW is not specified. For an SP COS, the
 * HWRM should set min BW to some small value when the min BW is not specified.
 * Maximum Bandwidth: # This is the bandwidth limit of the COS. # Values 0x0 and
 * 0xFF.. (all Fs) are considered unspecified and the HWRM will set the maximum
 * bandwidth to maximum port bandwidth. Priority Level: # It applies only to SP.
 * # This parameter is ignored for non-SP. # 0-7 are valid values (higher value
 * means higher priority) # A priority level can be assigned to at most one SP.
 * # Invalid priority levels assignment for SPs shall result in failure.
 * Additional notes: # The HWRM may have to use min and (max - min) to set
 * appropriate counters of hardware rate limiters. # The bandwidth percentage as
 * specified in the DCB TC BW assignment should be used by the driver to specify
 * maximum bandwidth and bandwidth weight for a COS. For example, the driver
 * should set max BW to 20 Gbps and weight to 50 for two COSs when these two
 * COSs are assigned 50% share of 40 Gbps max port bandwidth.
 */
/* Input (128 bytes) */

struct hwrm_queue_cos2bw_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	uint32_t enables;
	/*
	 * This bit must be '1' for the cos_queue_id0_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID0_VALID UINT32_C(0x1)
	/*
	 * This bit must be '1' for the cos_queue_id1_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID1_VALID UINT32_C(0x2)
	/*
	 * This bit must be '1' for the cos_queue_id2_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID2_VALID UINT32_C(0x4)
	/*
	 * This bit must be '1' for the cos_queue_id3_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID3_VALID UINT32_C(0x8)
	/*
	 * This bit must be '1' for the cos_queue_id4_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID4_VALID UINT32_C(0x10)
	/*
	 * This bit must be '1' for the cos_queue_id5_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID5_VALID UINT32_C(0x20)
	/*
	 * This bit must be '1' for the cos_queue_id6_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID6_VALID UINT32_C(0x40)
	/*
	 * This bit must be '1' for the cos_queue_id7_valid field to be
	 * configured.
	 */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_ENABLES_COS_QUEUE_ID7_VALID UINT32_C(0x80)
	uint16_t port_id;
	/*
	 * Port ID of port for which the table is being configured. The HWRM
	 * needs to check whether this function is allowed to configure TC BW
	 * assignment on this port.
	 */
	uint8_t queue_id0;
	/* ID of CoS Queue 0. */
	uint8_t unused_0;
	uint32_t queue_id0_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id0_max_bw;
	/*
	 * Maximum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id0_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID0_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID0_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID0_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID0_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id0_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id0_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id1;
	/* ID of CoS Queue 1. */
	uint32_t queue_id1_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id1_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id1_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID1_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID1_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID1_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID1_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id1_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id1_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id2;
	/* ID of CoS Queue 2. */
	uint32_t queue_id2_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id2_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id2_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID2_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID2_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID2_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID2_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id2_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id2_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id3;
	/* ID of CoS Queue 3. */
	uint32_t queue_id3_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id3_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id3_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID3_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID3_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID3_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID3_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id3_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id3_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id4;
	/* ID of CoS Queue 4. */
	uint32_t queue_id4_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id4_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id4_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID4_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID4_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID4_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID4_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id4_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id4_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id5;
	/* ID of CoS Queue 5. */
	uint32_t queue_id5_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id5_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id5_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID5_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID5_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID5_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID5_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id5_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id5_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id6;
	/* ID of CoS Queue 6. */
	uint32_t queue_id6_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id6_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id6_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID6_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID6_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID6_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID6_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id6_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id6_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t queue_id7;
	/* ID of CoS Queue 7. */
	uint32_t queue_id7_min_bw;
	/*
	 * Minimum BW allocated to CoS Queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint32_t queue_id7_max_bw;
	/*
	 * Maximum BW allocated to CoS queue in Mbps. The HWRM will translate
	 * this value into byte counter and time interval used for this COS
	 * inside the device.
	 */
	uint8_t queue_id7_tsa_assign;
	/* Transmission Selection Algorithm (TSA) for CoS Queue. */
	/* Strict Priority */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID7_TSA_ASSIGN_SP (UINT32_C(0x0) << 0)
	/* Enhanced Transmission Selection */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID7_TSA_ASSIGN_ETS (UINT32_C(0x1) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID7_TSA_ASSIGN_RESERVED_FIRST (UINT32_C(0x2) << 0)
	/* reserved */
	#define HWRM_QUEUE_COS2BW_CFG_INPUT_QUEUE_ID7_TSA_ASSIGN_RESERVED_LAST (UINT32_C(0xff) << 0)
	uint8_t queue_id7_pri_lvl;
	/*
	 * Priority level for strict priority. Valid only when the tsa_assign is
	 * 0 - Strict Priority (SP) 0..7 - Valid values. 8..255 - Reserved.
	 */
	uint8_t queue_id7_bw_weight;
	/*
	 * Weight used to allocate remaining BW for this COS after servicing
	 * guaranteed bandwidths for all COS.
	 */
	uint8_t unused_1[5];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_queue_cos2bw_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_alloc */
/*
 * Description: This VNIC is a resource in the RX side of the chip that is used
 * to represent a virtual host "interface". # At the time of VNIC allocation or
 * configuration, the function can specify whether it wants the requested VNIC
 * to be the default VNIC for the function or not. # If a function requests
 * allocation of a VNIC for the first time and a VNIC is successfully allocated
 * by the HWRM, then the HWRM shall make the allocated VNIC as the default VNIC
 * for that function. # The default VNIC shall be used for the default action
 * for a partition or function. # For each VNIC allocated on a function, a
 * mapping on the RX side to map the allocated VNIC to source virtual interface
 * shall be performed by the HWRM. This should be hidden to the function driver
 * requesting the VNIC allocation. This enables broadcast/multicast replication
 * with source knockout. # If multicast replication with source knockout is
 * enabled, then the internal VNIC to SVIF mapping data structures shall be
 * programmed at the time of VNIC allocation.
 */
/* Input (24 bytes) */

struct hwrm_vnic_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', this VNIC is requested to be the default VNIC
	 * for this function.
	 */
	#define HWRM_VNIC_ALLOC_INPUT_FLAGS_DEFAULT		UINT32_C(0x1)
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t vnic_id;
	/* Logical vnic ID */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_free */
/*
 * Description: Free a VNIC resource. Idle any resources associated with the
 * VNIC as well as the VNIC. Reset and release all resources associated with the
 * VNIC.
 */
/* Input (24 bytes) */

struct hwrm_vnic_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t vnic_id;
	/* Logical vnic ID */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_cfg */
/* Description: Configure the RX VNIC structure. */
/* Input (40 bytes) */

struct hwrm_vnic_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the VNIC is requested to be the default VNIC
	 * for the function.
	 */
	#define HWRM_VNIC_CFG_INPUT_FLAGS_DEFAULT		UINT32_C(0x1)
	/*
	 * When this bit is '1', the VNIC is being configured to strip VLAN in
	 * the RX path. If set to '0', then VLAN stripping is disabled on this
	 * VNIC.
	 */
	#define HWRM_VNIC_CFG_INPUT_FLAGS_VLAN_STRIP_MODE	UINT32_C(0x2)
	/*
	 * When this bit is '1', the VNIC is being configured to buffer receive
	 * packets in the hardware until the host posts new receive buffers. If
	 * set to '0', then bd_stall is being configured to be disabled on this
	 * VNIC.
	 */
	#define HWRM_VNIC_CFG_INPUT_FLAGS_BD_STALL_MODE		UINT32_C(0x4)
	/*
	 * When this bit is '1', the VNIC is being configured to receive both
	 * RoCE and non-RoCE traffic. If set to '0', then this VNIC is not
	 * configured to be operating in dual VNIC mode.
	 */
	#define HWRM_VNIC_CFG_INPUT_FLAGS_ROCE_DUAL_VNIC_MODE	UINT32_C(0x8)
	/*
	 * When this flag is set to '1', the VNIC is requested to be configured
	 * to receive only RoCE traffic. If this flag is set to '0', then this
	 * flag shall be ignored by the HWRM. If roce_dual_vnic_mode flag is set
	 * to '1', then the HWRM client shall not set this flag to '1'.
	 */
	#define HWRM_VNIC_CFG_INPUT_FLAGS_ROCE_ONLY_VNIC_MODE	UINT32_C(0x10)
	uint32_t enables;
	/* This bit must be '1' for the dflt_ring_grp field to be configured. */
	#define HWRM_VNIC_CFG_INPUT_ENABLES_DFLT_RING_GRP	UINT32_C(0x1)
	/* This bit must be '1' for the rss_rule field to be configured. */
	#define HWRM_VNIC_CFG_INPUT_ENABLES_RSS_RULE		UINT32_C(0x2)
	/* This bit must be '1' for the cos_rule field to be configured. */
	#define HWRM_VNIC_CFG_INPUT_ENABLES_COS_RULE		UINT32_C(0x4)
	/* This bit must be '1' for the lb_rule field to be configured. */
	#define HWRM_VNIC_CFG_INPUT_ENABLES_LB_RULE		UINT32_C(0x8)
	/* This bit must be '1' for the mru field to be configured. */
	#define HWRM_VNIC_CFG_INPUT_ENABLES_MRU			UINT32_C(0x10)
	uint16_t vnic_id;
	/* Logical vnic ID */
	uint16_t dflt_ring_grp;
	/*
	 * Default Completion ring for the VNIC. This ring will be chosen if
	 * packet does not match any RSS rules and if there is no COS rule.
	 */
	uint16_t rss_rule;
	/*
	 * RSS ID for RSS rule/table structure. 0xFF... (All Fs) if there is no
	 * RSS rule.
	 */
	uint16_t cos_rule;
	/*
	 * RSS ID for COS rule/table structure. 0xFF... (All Fs) if there is no
	 * COS rule.
	 */
	uint16_t lb_rule;
	/*
	 * RSS ID for load balancing rule/table structure. 0xFF... (All Fs) if
	 * there is no LB rule.
	 */
	uint16_t mru;
	/*
	 * The maximum receive unit of the vnic. Each vnic is associated with a
	 * function. The vnic mru value overwrites the mru setting of the
	 * associated function. The HWRM shall make sure that vnic mru does not
	 * exceed the mru of the port the function is associated with.
	 */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_qcfg */
/*
 * Description: Query the RX VNIC structure. This function can be used by a PF
 * driver to query its own VNIC resource or VNIC resource of its child VF. This
 * function can also be used by a VF driver to query its own VNIC resource.
 */
/* Input (32 bytes) */

struct hwrm_vnic_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the vf_id_valid field to be configured. */
	#define HWRM_VNIC_QCFG_INPUT_ENABLES_VF_ID_VALID	UINT32_C(0x1)
	uint32_t vnic_id;
	/* Logical vnic ID */
	uint16_t vf_id;
	/* ID of Virtual Function whose VNIC resource is being queried. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_vnic_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t dflt_ring_grp;
	/* Default Completion ring for the VNIC. */
	uint16_t rss_rule;
	/*
	 * RSS ID for RSS rule/table structure. 0xFF... (All Fs) if there is no
	 * RSS rule.
	 */
	uint16_t cos_rule;
	/*
	 * RSS ID for COS rule/table structure. 0xFF... (All Fs) if there is no
	 * COS rule.
	 */
	uint16_t lb_rule;
	/*
	 * RSS ID for load balancing rule/table structure. 0xFF... (All Fs) if
	 * there is no LB rule.
	 */
	uint16_t mru;
	/* The maximum receive unit of the vnic. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint32_t flags;
	/* When this bit is '1', the VNIC is the default VNIC for the function. */
	#define HWRM_VNIC_QCFG_OUTPUT_FLAGS_DEFAULT		UINT32_C(0x1)
	/*
	 * When this bit is '1', the VNIC is configured to strip VLAN in the RX
	 * path. If set to '0', then VLAN stripping is disabled on this VNIC.
	 */
	#define HWRM_VNIC_QCFG_OUTPUT_FLAGS_VLAN_STRIP_MODE	UINT32_C(0x2)
	/*
	 * When this bit is '1', the VNIC is configured to buffer receive
	 * packets in the hardware until the host posts new receive buffers. If
	 * set to '0', then bd_stall is disabled on this VNIC.
	 */
	#define HWRM_VNIC_QCFG_OUTPUT_FLAGS_BD_STALL_MODE	UINT32_C(0x4)
	/*
	 * When this bit is '1', the VNIC is configured to receive both RoCE and
	 * non-RoCE traffic. If set to '0', then this VNIC is not configured to
	 * operate in dual VNIC mode.
	 */
	#define HWRM_VNIC_QCFG_OUTPUT_FLAGS_ROCE_DUAL_VNIC_MODE	UINT32_C(0x8)
	/*
	 * When this flag is set to '1', the VNIC is configured to receive only
	 * RoCE traffic. When this flag is set to '0', the VNIC is not
	 * configured to receive only RoCE traffic. If roce_dual_vnic_mode flag
	 * and this flag both are set to '1', then it is an invalid
	 * configuration of the VNIC. The HWRM should not allow that type of
	 * mis-configuration by HWRM clients.
	 */
	#define HWRM_VNIC_QCFG_OUTPUT_FLAGS_ROCE_ONLY_VNIC_MODE	UINT32_C(0x10)
	uint32_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t unused_5;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_tpa_cfg */
/* Description: This function is used to enable/configure TPA on the VNIC. */
/* Input (40 bytes) */

struct hwrm_vnic_tpa_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the VNIC shall be configured to perform
	 * transparent packet aggregation (TPA) of non-tunneled TCP packets.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_TPA		UINT32_C(0x1)
	/*
	 * When this bit is '1', the VNIC shall be configured to perform
	 * transparent packet aggregation (TPA) of tunneled TCP packets.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_ENCAP_TPA		UINT32_C(0x2)
	/*
	 * When this bit is '1', the VNIC shall be configured to perform
	 * transparent packet aggregation (TPA) according to Windows Receive
	 * Segment Coalescing (RSC) rules.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_RSC_WND_UPDATE	UINT32_C(0x4)
	/*
	 * When this bit is '1', the VNIC shall be configured to perform
	 * transparent packet aggregation (TPA) according to Linux Generic
	 * Receive Offload (GRO) rules.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_GRO		UINT32_C(0x8)
	/*
	 * When this bit is '1', the VNIC shall be configured to perform
	 * transparent packet aggregation (TPA) for TCP packets with IP ECN set
	 * to non-zero.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_AGG_WITH_ECN	UINT32_C(0x10)
	/*
	 * When this bit is '1', the VNIC shall be configured to perform
	 * transparent packet aggregation (TPA) for GRE tunneled TCP packets
	 * only if all packets have the same GRE sequence.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_AGG_WITH_SAME_GRE_SEQ UINT32_C(0x20)
	/*
	 * When this bit is '1' and the GRO mode is enabled, the VNIC shall be
	 * configured to perform transparent packet aggregation (TPA) for
	 * TCP/IPv4 packets with consecutively increasing IPIDs. In other words,
	 * the last packet that is being aggregated to an already existing
	 * aggregation context shall have IPID 1 more than the IPID of the last
	 * packet that was aggregated in that aggregation context.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_GRO_IPID_CHECK	UINT32_C(0x40)
	/*
	 * When this bit is '1' and the GRO mode is enabled, the VNIC shall be
	 * configured to perform transparent packet aggregation (TPA) for TCP
	 * packets with the same TTL (IPv4) or Hop limit (IPv6) value.
	 */
	#define HWRM_VNIC_TPA_CFG_INPUT_FLAGS_GRO_TTL_CHECK	UINT32_C(0x80)
	uint32_t enables;
	/* This bit must be '1' for the max_agg_segs field to be configured. */
	#define HWRM_VNIC_TPA_CFG_INPUT_ENABLES_MAX_AGG_SEGS	UINT32_C(0x1)
	/* This bit must be '1' for the max_aggs field to be configured. */
	#define HWRM_VNIC_TPA_CFG_INPUT_ENABLES_MAX_AGGS	UINT32_C(0x2)
	/* This bit must be '1' for the max_agg_timer field to be configured. */
	#define HWRM_VNIC_TPA_CFG_INPUT_ENABLES_MAX_AGG_TIMER	UINT32_C(0x4)
	/* This bit must be '1' for the min_agg_len field to be configured. */
	#define HWRM_VNIC_TPA_CFG_INPUT_ENABLES_MIN_AGG_LEN	UINT32_C(0x8)
	uint16_t vnic_id;
	/* Logical vnic ID */
	uint16_t max_agg_segs;
	/*
	 * This is the maximum number of TCP segments that can be aggregated
	 * (unit is Log2). Max value is 31.
	 */
	/* 1 segment */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGG_SEGS_1		(UINT32_C(0x0) << 0)
	/* 2 segments */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGG_SEGS_2		(UINT32_C(0x1) << 0)
	/* 4 segments */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGG_SEGS_4		(UINT32_C(0x2) << 0)
	/* 8 segments */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGG_SEGS_8		(UINT32_C(0x3) << 0)
	/* Any segment size larger than this is not valid */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGG_SEGS_MAX	(UINT32_C(0x1f) << 0)
	uint16_t max_aggs;
	/*
	 * This is the maximum number of aggregations this VNIC is allowed (unit
	 * is Log2). Max value is 7
	 */
	/* 1 aggregation */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGGS_1		(UINT32_C(0x0) << 0)
	/* 2 aggregations */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGGS_2		(UINT32_C(0x1) << 0)
	/* 4 aggregations */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGGS_4		(UINT32_C(0x2) << 0)
	/* 8 aggregations */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGGS_8		(UINT32_C(0x3) << 0)
	/* 16 aggregations */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGGS_16		(UINT32_C(0x4) << 0)
	/* Any aggregation size larger than this is not valid */
	#define HWRM_VNIC_TPA_CFG_INPUT_MAX_AGGS_MAX		(UINT32_C(0x7) << 0)
	uint8_t unused_0;
	uint8_t unused_1;
	uint32_t max_agg_timer;
	/*
	 * This is the maximum amount of time allowed for an aggregation context
	 * to complete after it was initiated.
	 */
	uint32_t min_agg_len;
	/*
	 * This is the minimum amount of payload length required to start an
	 * aggregation context.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_tpa_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_tpa_qcfg */
/*
 * Description: This function can be used to query TPA configuration on the
 * VNIC.
 */
/* Input (24 bytes) */

struct hwrm_vnic_tpa_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t vnic_id;
	/* Logical vnic ID */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_vnic_tpa_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the VNIC is configured to perform transparent
	 * packet aggregation (TPA) of non-tunneled TCP packets.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_TPA		UINT32_C(0x1)
	/*
	 * When this bit is '1', the VNIC is configured to perform transparent
	 * packet aggregation (TPA) of tunneled TCP packets.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_ENCAP_TPA	UINT32_C(0x2)
	/*
	 * When this bit is '1', the VNIC is configured to perform transparent
	 * packet aggregation (TPA) according to Windows Receive Segment
	 * Coalescing (RSC) rules.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_RSC_WND_UPDATE	UINT32_C(0x4)
	/*
	 * When this bit is '1', the VNIC is configured to perform transparent
	 * packet aggregation (TPA) according to Linux Generic Receive Offload
	 * (GRO) rules.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_GRO		UINT32_C(0x8)
	/*
	 * When this bit is '1', the VNIC is configured to perform transparent
	 * packet aggregation (TPA) for TCP packets with IP ECN set to non-zero.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_AGG_WITH_ECN	UINT32_C(0x10)
	/*
	 * When this bit is '1', the VNIC is configured to perform transparent
	 * packet aggregation (TPA) for GRE tunneled TCP packets only if all
	 * packets have the same GRE sequence.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_AGG_WITH_SAME_GRE_SEQ UINT32_C(0x20)
	/*
	 * When this bit is '1' and the GRO mode is enabled, the VNIC is
	 * configured to perform transparent packet aggregation (TPA) for
	 * TCP/IPv4 packets with consecutively increasing IPIDs. In other words,
	 * the last packet that is being aggregated to an already existing
	 * aggregation context shall have IPID 1 more than the IPID of the last
	 * packet that was aggregated in that aggregation context.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_GRO_IPID_CHECK	UINT32_C(0x40)
	/*
	 * When this bit is '1' and the GRO mode is enabled, the VNIC is
	 * configured to perform transparent packet aggregation (TPA) for TCP
	 * packets with the same TTL (IPv4) or Hop limit (IPv6) value.
	 */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_FLAGS_GRO_TTL_CHECK	UINT32_C(0x80)
	uint16_t max_agg_segs;
	/*
	 * This is the maximum number of TCP segments that can be aggregated
	 * (unit is Log2). Max value is 31.
	 */
	/* 1 segment */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGG_SEGS_1	(UINT32_C(0x0) << 0)
	/* 2 segments */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGG_SEGS_2	(UINT32_C(0x1) << 0)
	/* 4 segments */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGG_SEGS_4	(UINT32_C(0x2) << 0)
	/* 8 segments */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGG_SEGS_8	(UINT32_C(0x3) << 0)
	/* Any segment size larger than this is not valid */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGG_SEGS_MAX	(UINT32_C(0x1f) << 0)
	uint16_t max_aggs;
	/*
	 * This is the maximum number of aggregations this VNIC is allowed (unit
	 * is Log2). Max value is 7
	 */
	/* 1 aggregation */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGGS_1		(UINT32_C(0x0) << 0)
	/* 2 aggregations */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGGS_2		(UINT32_C(0x1) << 0)
	/* 4 aggregations */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGGS_4		(UINT32_C(0x2) << 0)
	/* 8 aggregations */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGGS_8		(UINT32_C(0x3) << 0)
	/* 16 aggregations */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGGS_16		(UINT32_C(0x4) << 0)
	/* Any aggregation size larger than this is not valid */
	#define HWRM_VNIC_TPA_QCFG_OUTPUT_MAX_AGGS_MAX		(UINT32_C(0x7) << 0)
	uint32_t max_agg_timer;
	/*
	 * This is the maximum amount of time allowed for an aggregation context
	 * to complete after it was initiated.
	 */
	uint32_t min_agg_len;
	/*
	 * This is the minimum amount of payload length required to start an
	 * aggregation context.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_rss_cfg */
/* Description: This function is used to enable RSS configuration. */
/* Input (48 bytes) */

struct hwrm_vnic_rss_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t hash_type;
	/*
	 * When this bit is '1', the RSS hash shall be computed over source and
	 * destination IPv4 addresses of IPv4 packets.
	 */
	#define HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_IPV4		UINT32_C(0x1)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv4 addresses and source/destination ports of
	 * TCP/IPv4 packets.
	 */
	#define HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_TCP_IPV4	UINT32_C(0x2)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv4 addresses and source/destination ports of
	 * UDP/IPv4 packets.
	 */
	#define HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_UDP_IPV4	UINT32_C(0x4)
	/*
	 * When this bit is '1', the RSS hash shall be computed over source and
	 * destination IPv4 addresses of IPv6 packets.
	 */
	#define HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_IPV6		UINT32_C(0x8)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv6 addresses and source/destination ports of
	 * TCP/IPv6 packets.
	 */
	#define HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_TCP_IPV6	UINT32_C(0x10)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv6 addresses and source/destination ports of
	 * UDP/IPv6 packets.
	 */
	#define HWRM_VNIC_RSS_CFG_INPUT_HASH_TYPE_UDP_IPV6	UINT32_C(0x20)
	uint32_t unused_0;
	uint64_t ring_grp_tbl_addr;
	/* This is the address for rss ring group table */
	uint64_t hash_key_tbl_addr;
	/* This is the address for rss hash key table */
	uint16_t rss_ctx_idx;
	/* Index to the rss indirection table. */
	uint16_t unused_1[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_rss_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_rss_qcfg */
/* Description: This function is used to query RSS context configuration. */
/* Input (24 bytes) */

struct hwrm_vnic_rss_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t rss_ctx_idx;
	/* Index to the rss indirection table. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (64 bytes) */

struct hwrm_vnic_rss_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t hash_type;
	/*
	 * When this bit is '1', the RSS hash shall be computed over source and
	 * destination IPv4 addresses of IPv4 packets.
	 */
	#define HWRM_VNIC_RSS_QCFG_OUTPUT_HASH_TYPE_IPV4	UINT32_C(0x1)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv4 addresses and source/destination ports of
	 * TCP/IPv4 packets.
	 */
	#define HWRM_VNIC_RSS_QCFG_OUTPUT_HASH_TYPE_TCP_IPV4	UINT32_C(0x2)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv4 addresses and source/destination ports of
	 * UDP/IPv4 packets.
	 */
	#define HWRM_VNIC_RSS_QCFG_OUTPUT_HASH_TYPE_UDP_IPV4	UINT32_C(0x4)
	/*
	 * When this bit is '1', the RSS hash shall be computed over source and
	 * destination IPv4 addresses of IPv6 packets.
	 */
	#define HWRM_VNIC_RSS_QCFG_OUTPUT_HASH_TYPE_IPV6	UINT32_C(0x8)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv6 addresses and source/destination ports of
	 * TCP/IPv6 packets.
	 */
	#define HWRM_VNIC_RSS_QCFG_OUTPUT_HASH_TYPE_TCP_IPV6	UINT32_C(0x10)
	/*
	 * When this bit is '1', the RSS hash shall be computed over
	 * source/destination IPv6 addresses and source/destination ports of
	 * UDP/IPv6 packets.
	 */
	#define HWRM_VNIC_RSS_QCFG_OUTPUT_HASH_TYPE_UDP_IPV6	UINT32_C(0x20)
	uint32_t unused_0;
	uint32_t hash_key[10];
	/* This is the value of rss hash key */
	uint32_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_plcmodes_cfg */
/*
 * Description: This function can be used to set placement mode configuration of
 * the VNIC.
 */
/* Input (40 bytes) */

struct hwrm_vnic_plcmodes_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the VNIC shall be configured to use regular
	 * placement algorithm. By default, the regular placement algorithm
	 * shall be enabled on the VNIC.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_FLAGS_REGULAR_PLACEMENT UINT32_C(0x1)
	/*
	 * When this bit is '1', the VNIC shall be configured use the jumbo
	 * placement algorithm.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_FLAGS_JUMBO_PLACEMENT UINT32_C(0x2)
	/*
	 * When this bit is '1', the VNIC shall be configured to enable Header-
	 * Data split for IPv4 packets according to the following rules: # If
	 * the packet is identified as TCP/IPv4, then the packet is split at the
	 * beginning of the TCP payload. # If the packet is identified as
	 * UDP/IPv4, then the packet is split at the beginning of UDP payload. #
	 * If the packet is identified as non-TCP and non-UDP IPv4 packet, then
	 * the packet is split at the beginning of the upper layer protocol
	 * header carried in the IPv4 packet.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_FLAGS_HDS_IPV4	UINT32_C(0x4)
	/*
	 * When this bit is '1', the VNIC shall be configured to enable Header-
	 * Data split for IPv6 packets according to the following rules: # If
	 * the packet is identified as TCP/IPv6, then the packet is split at the
	 * beginning of the TCP payload. # If the packet is identified as
	 * UDP/IPv6, then the packet is split at the beginning of UDP payload. #
	 * If the packet is identified as non-TCP and non-UDP IPv6 packet, then
	 * the packet is split at the beginning of the upper layer protocol
	 * header carried in the IPv6 packet.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_FLAGS_HDS_IPV6	UINT32_C(0x8)
	/*
	 * When this bit is '1', the VNIC shall be configured to enable Header-
	 * Data split for FCoE packets at the beginning of FC payload.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_FLAGS_HDS_FCOE	UINT32_C(0x10)
	/*
	 * When this bit is '1', the VNIC shall be configured to enable Header-
	 * Data split for RoCE packets at the beginning of RoCE payload (after
	 * BTH/GRH headers).
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_FLAGS_HDS_ROCE	UINT32_C(0x20)
	uint32_t enables;
	/*
	 * This bit must be '1' for the jumbo_thresh_valid field to be
	 * configured.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_ENABLES_JUMBO_THRESH_VALID UINT32_C(0x1)
	/* This bit must be '1' for the hds_offset_valid field to be configured. */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_ENABLES_HDS_OFFSET_VALID UINT32_C(0x2)
	/*
	 * This bit must be '1' for the hds_threshold_valid field to be
	 * configured.
	 */
	#define HWRM_VNIC_PLCMODES_CFG_INPUT_ENABLES_HDS_THRESHOLD_VALID UINT32_C(0x4)
	uint32_t vnic_id;
	/* Logical vnic ID */
	uint16_t jumbo_thresh;
	/*
	 * When jumbo placement algorithm is enabled, this value is used to
	 * determine the threshold for jumbo placement. Packets with length
	 * larger than this value will be placed according to the jumbo
	 * placement algorithm.
	 */
	uint16_t hds_offset;
	/*
	 * This value is used to determine the offset into packet buffer where
	 * the split data (payload) will be placed according to one of of HDS
	 * placement algorithm. The lengths of packet buffers provided for split
	 * data shall be larger than this value.
	 */
	uint16_t hds_threshold;
	/*
	 * When one of the HDS placement algorithm is enabled, this value is
	 * used to determine the threshold for HDS placement. Packets with
	 * length larger than this value will be placed according to the HDS
	 * placement algorithm. This value shall be in multiple of 4 bytes.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_plcmodes_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_plcmodes_qcfg */
/*
 * Description: This function can be used to query placement mode configuration
 * of the VNIC.
 */
/* Input (24 bytes) */

struct hwrm_vnic_plcmodes_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t vnic_id;
	/* Logical vnic ID */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_vnic_plcmodes_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t flags;
	/*
	 * When this bit is '1', the VNIC is configured to use regular placement
	 * algorithm.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_REGULAR_PLACEMENT UINT32_C(0x1)
	/*
	 * When this bit is '1', the VNIC is configured to use the jumbo
	 * placement algorithm.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_JUMBO_PLACEMENT UINT32_C(0x2)
	/*
	 * When this bit is '1', the VNIC is configured to enable Header-Data
	 * split for IPv4 packets.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_HDS_IPV4	UINT32_C(0x4)
	/*
	 * When this bit is '1', the VNIC is configured to enable Header-Data
	 * split for IPv6 packets.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_HDS_IPV6	UINT32_C(0x8)
	/*
	 * When this bit is '1', the VNIC is configured to enable Header-Data
	 * split for FCoE packets.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_HDS_FCOE	UINT32_C(0x10)
	/*
	 * When this bit is '1', the VNIC is configured to enable Header-Data
	 * split for RoCE packets.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_HDS_ROCE	UINT32_C(0x20)
	/*
	 * When this bit is '1', the VNIC is configured to be the default VNIC
	 * of the requesting function.
	 */
	#define HWRM_VNIC_PLCMODES_QCFG_OUTPUT_FLAGS_DFLT_VNIC	UINT32_C(0x40)
	uint16_t jumbo_thresh;
	/*
	 * When jumbo placement algorithm is enabled, this value is used to
	 * determine the threshold for jumbo placement. Packets with length
	 * larger than this value will be placed according to the jumbo
	 * placement algorithm.
	 */
	uint16_t hds_offset;
	/*
	 * This value is used to determine the offset into packet buffer where
	 * the split data (payload) will be placed according to one of of HDS
	 * placement algorithm. The lengths of packet buffers provided for split
	 * data shall be larger than this value.
	 */
	uint16_t hds_threshold;
	/*
	 * When one of the HDS placement algorithm is enabled, this value is
	 * used to determine the threshold for HDS placement. Packets with
	 * length larger than this value will be placed according to the HDS
	 * placement algorithm. This value shall be in multiple of 4 bytes.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_rss_cos_lb_ctx_alloc */
/* Description: This function is used to allocate COS/Load Balance context. */
/* Input (16 bytes) */

struct hwrm_vnic_rss_cos_lb_ctx_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_rss_cos_lb_ctx_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t rss_cos_lb_ctx_id;
	/* rss_cos_lb_ctx_id is 16 b */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_vnic_rss_cos_lb_ctx_free */
/* Description: This function can be used to free COS/Load Balance context. */
/* Input (24 bytes) */

struct hwrm_vnic_rss_cos_lb_ctx_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t rss_cos_lb_ctx_id;
	/* rss_cos_lb_ctx_id is 16 b */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_vnic_rss_cos_lb_ctx_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_alloc */
/*
 * Description: This command allocates and does basic preparation for a ring.
 */
/* Input (80 bytes) */

struct hwrm_ring_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the Reserved1 field to be configured. */
	#define HWRM_RING_ALLOC_INPUT_ENABLES_RESERVED1		UINT32_C(0x1)
	/* This bit must be '1' for the Reserved2 field to be configured. */
	#define HWRM_RING_ALLOC_INPUT_ENABLES_RESERVED2		UINT32_C(0x2)
	/* This bit must be '1' for the Reserved3 field to be configured. */
	#define HWRM_RING_ALLOC_INPUT_ENABLES_RESERVED3		UINT32_C(0x4)
	/*
	 * This bit must be '1' for the stat_ctx_id_valid field to be
	 * configured.
	 */
	#define HWRM_RING_ALLOC_INPUT_ENABLES_STAT_CTX_ID_VALID	UINT32_C(0x8)
	/* This bit must be '1' for the Reserved4 field to be configured. */
	#define HWRM_RING_ALLOC_INPUT_ENABLES_RESERVED4		UINT32_C(0x10)
	/* This bit must be '1' for the max_bw_valid field to be configured. */
	#define HWRM_RING_ALLOC_INPUT_ENABLES_MAX_BW_VALID	UINT32_C(0x20)
	uint8_t ring_type;
	/* Ring Type. */
	/* Completion Ring (CR) */
	#define HWRM_RING_ALLOC_INPUT_RING_TYPE_CMPL		(UINT32_C(0x0) << 0)
	/* TX Ring (TR) */
	#define HWRM_RING_ALLOC_INPUT_RING_TYPE_TX		(UINT32_C(0x1) << 0)
	/* RX Ring (RR) */
	#define HWRM_RING_ALLOC_INPUT_RING_TYPE_RX		(UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint16_t unused_1;
	uint64_t page_tbl_addr;
	/* This value is a pointer to the page table for the Ring. */
	uint32_t fbo;
	/* First Byte Offset of the first entry in the first page. */
	uint8_t page_size;
	/*
	 * Actual page size in 2^page_size. The supported range is increments in
	 * powers of 2 from 16 bytes to 1GB. - 4 = 16 B Page size is 16 B. - 12
	 * = 4 KB Page size is 4 KB. - 13 = 8 KB Page size is 8 KB. - 16 = 64 KB
	 * Page size is 64 KB. - 21 = 2 MB Page size is 2 MB. - 22 = 4 MB Page
	 * size is 4 MB. - 30 = 1 GB Page size is 1 GB.
	 */
	uint8_t page_tbl_depth;
	/*
	 * This value indicates the depth of page table. For this version of the
	 * specification, value other than 0 or 1 shall be considered as an
	 * invalid value. When the page_tbl_depth = 0, then it is treated as a
	 * special case with the following. 1. FBO and page size fields are not
	 * valid. 2. page_tbl_addr is the physical address of the first element
	 * of the ring.
	 */
	uint8_t unused_2;
	uint8_t unused_3;
	uint32_t length;
	/*
	 * Number of 16B units in the ring. Minimum size for a ring is 16 16B
	 * entries.
	 */
	uint16_t logical_id;
	/*
	 * Logical ring number for the ring to be allocated. This value
	 * determines the position in the doorbell area where the update to the
	 * ring will be made. For completion rings, this value is also the MSI-X
	 * vector number for the function the completion ring is associated
	 * with.
	 */
	uint16_t cmpl_ring_id;
	/*
	 * This field is used only when ring_type is a TX ring. This value
	 * indicates what completion ring the TX ring is associated with.
	 */
	uint16_t queue_id;
	/*
	 * This field is used only when ring_type is a TX ring. This value
	 * indicates what CoS queue the TX ring is associated with.
	 */
	uint8_t unused_4;
	uint8_t unused_5;
	uint32_t reserved1;
	/* This field is reserved for the future use. It shall be set to 0. */
	uint16_t reserved2;
	/* This field is reserved for the future use. It shall be set to 0. */
	uint8_t unused_6;
	uint8_t unused_7;
	uint32_t reserved3;
	/* This field is reserved for the future use. It shall be set to 0. */
	uint32_t stat_ctx_id;
	/*
	 * This field is used only when ring_type is a TX ring. This input
	 * indicates what statistics context this ring should be associated
	 * with.
	 */
	uint32_t reserved4;
	/* This field is reserved for the future use. It shall be set to 0. */
	uint32_t max_bw;
	/*
	 * This field is used only when ring_type is a TX ring. Maximum BW
	 * allocated to this TX ring in Mbps. The HWRM will translate this value
	 * into byte counter and time interval used for this ring inside the
	 * device.
	 */
	uint8_t int_mode;
	/*
	 * This field is used only when ring_type is a Completion ring. This
	 * value indicates what interrupt mode should be used on this completion
	 * ring. Note: In the legacy interrupt mode, no more than 16 completion
	 * rings are allowed.
	 */
	/* Legacy INTA */
	#define HWRM_RING_ALLOC_INPUT_INT_MODE_LEGACY		(UINT32_C(0x0) << 0)
	/* Reserved */
	#define HWRM_RING_ALLOC_INPUT_INT_MODE_RSVD		(UINT32_C(0x1) << 0)
	/* MSI-X */
	#define HWRM_RING_ALLOC_INPUT_INT_MODE_MSIX		(UINT32_C(0x2) << 0)
	/* No Interrupt - Polled mode */
	#define HWRM_RING_ALLOC_INPUT_INT_MODE_POLL		(UINT32_C(0x3) << 0)
	uint8_t unused_8[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_ring_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t ring_id;
	/* Physical number of ring allocated. */
	uint16_t logical_ring_id;
	/* Logical number of ring allocated. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_free */
/*
 * Description: This command is used to free a ring and associated resources.
 */
/* Input (24 bytes) */

struct hwrm_ring_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t ring_type;
	/* Ring Type. */
	/* Completion Ring (CR) */
	#define HWRM_RING_FREE_INPUT_RING_TYPE_CMPL		(UINT32_C(0x0) << 0)
	/* TX Ring (TR) */
	#define HWRM_RING_FREE_INPUT_RING_TYPE_TX		(UINT32_C(0x1) << 0)
	/* RX Ring (RR) */
	#define HWRM_RING_FREE_INPUT_RING_TYPE_RX		(UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint16_t ring_id;
	/* Physical number of ring allocated. */
	uint32_t unused_1;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_ring_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_cmpl_ring_qaggint_params */
/*
 * Description: This command is used to query aggregation and interrupt related
 * parameters specified on a given completion ring.
 */
/* Input (24 bytes) */

struct hwrm_ring_cmpl_ring_qaggint_params_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t ring_id;
	/* Physical number of completion ring. */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_ring_cmpl_ring_qaggint_params_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t flags;
	/*
	 * When this bit is set to '1', interrupt max timer is reset whenever a
	 * completion is received.
	 */
	#define HWRM_RING_CMPL_RING_QAGGINT_PARAMS_OUTPUT_FLAGS_TIMER_RESET UINT32_C(0x1)
	/*
	 * When this bit is set to '1', ring idle mode aggregation will be
	 * enabled.
	 */
	#define HWRM_RING_CMPL_RING_QAGGINT_PARAMS_OUTPUT_FLAGS_RING_IDLE UINT32_C(0x2)
	uint16_t num_cmpl_dma_aggr;
	/* Number of completions to aggregate before DMA during the normal mode. */
	uint16_t num_cmpl_dma_aggr_during_int;
	/*
	 * Number of completions to aggregate before DMA during the interrupt
	 * mode.
	 */
	uint16_t cmpl_aggr_dma_tmr;
	/*
	 * Timer in unit of 80-nsec used to aggregate completions before DMA
	 * during the normal mode (not in interrupt mode).
	 */
	uint16_t cmpl_aggr_dma_tmr_during_int;
	/*
	 * Timer in unit of 80-nsec used to aggregate completions before DMA
	 * during the interrupt mode.
	 */
	uint16_t int_lat_tmr_min;
	/* Minimum time (in unit of 80-nsec) between two interrupts. */
	uint16_t int_lat_tmr_max;
	/*
	 * Maximum wait time (in unit of 80-nsec) spent aggregating completions
	 * before signaling the interrupt after the interrupt is enabled.
	 */
	uint16_t num_cmpl_aggr_int;
	/*
	 * Minimum number of completions aggregated before signaling an
	 * interrupt.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_cmpl_ring_cfg_aggint_params */
/*
 * Description: This command is used to configure aggregation and interrupt
 * related parameters specified on a given completion ring.
 */
/* Input (40 bytes) */

struct hwrm_ring_cmpl_ring_cfg_aggint_params_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t ring_id;
	/* Physical number of completion ring. */
	uint16_t flags;
	/*
	 * When this bit is set to '1', interrupt latency max timer is reset
	 * whenever a completion is received.
	 */
	#define HWRM_RING_CMPL_RING_CFG_AGGINT_PARAMS_INPUT_FLAGS_TIMER_RESET UINT32_C(0x1)
	/*
	 * When this bit is set to '1', ring idle mode aggregation will be
	 * enabled.
	 */
	#define HWRM_RING_CMPL_RING_CFG_AGGINT_PARAMS_INPUT_FLAGS_RING_IDLE UINT32_C(0x2)
	uint16_t num_cmpl_dma_aggr;
	/* Number of completions to aggregate before DMA during the normal mode. */
	uint16_t num_cmpl_dma_aggr_during_int;
	/*
	 * Number of completions to aggregate before DMA during the interrupt
	 * mode.
	 */
	uint16_t cmpl_aggr_dma_tmr;
	/*
	 * Timer in unit of 80-nsec used to aggregate completions before DMA
	 * during the normal mode (not in interrupt mode).
	 */
	uint16_t cmpl_aggr_dma_tmr_during_int;
	/*
	 * Timer in unit of 80-nsec used to aggregate completions before DMA
	 * during the interrupt mode.
	 */
	uint16_t int_lat_tmr_min;
	/* Minimum time (in unit of 80-nsec) between two interrupts. */
	uint16_t int_lat_tmr_max;
	/*
	 * Maximum wait time (in unit of 80-nsec) spent aggregating cmpls before
	 * signaling the interrupt after the interrupt is enabled.
	 */
	uint16_t num_cmpl_aggr_int;
	/*
	 * Minimum number of completions aggregated before signaling an
	 * interrupt.
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_ring_cmpl_ring_cfg_aggint_params_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_reset */
/*
 * Description: This command is used to reset a given ring. When an RX ring is
 * being reset, the HWRM shall perform TPA flush on all VNICs associated with
 * the RX ring that is being reset.
 */
/* Input (24 bytes) */

struct hwrm_ring_reset_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t ring_type;
	/* Ring Type. */
	/* Completion Ring (CR) */
	#define HWRM_RING_RESET_INPUT_RING_TYPE_CMPL		(UINT32_C(0x0) << 0)
	/* TX Ring (TR) */
	#define HWRM_RING_RESET_INPUT_RING_TYPE_TX		(UINT32_C(0x1) << 0)
	/* RX Ring (RR) */
	#define HWRM_RING_RESET_INPUT_RING_TYPE_RX		(UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint16_t ring_id;
	/* Physical number of the ring. */
	uint32_t unused_1;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_ring_reset_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_grp_alloc */
/*
 * Description: This API allocates and does basic preparation for a ring group.
 */
/* Input (24 bytes) */

struct hwrm_ring_grp_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t cr;
	/* This value identifies the CR associated with the ring group. */
	uint16_t rr;
	/* This value identifies the main RR associated with the ring group. */
	uint16_t ar;
	/*
	 * This value identifies the aggregation RR associated with the ring
	 * group. If this value is 0xFF... (All Fs), then no Aggregation ring
	 * will be set.
	 */
	uint16_t sc;
	/*
	 * This value identifies the statistics context associated with the ring
	 * group.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_ring_grp_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t ring_group_id;
	/*
	 * This is the ring group ID value. Use this value to program the
	 * default ring group for the VNIC or as table entries in an RSS/COS
	 * context.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_ring_grp_free */
/*
 * Description: This API frees a ring group and associated resources. # If a
 * ring in the ring group is reset or free, then the associated rings in the
 * ring group shall also be reset/free using hwrm_ring_free. # A function driver
 * shall always use hwrm_ring_grp_free after freeing all rings in a group. # As
 * a part of executing this command, the HWRM shall reset all associated ring
 * group resources.
 */
/* Input (24 bytes) */

struct hwrm_ring_grp_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t ring_group_id;
	/* This is the ring group ID value. */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_ring_grp_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_l2_filter_alloc */
/*
 * Description: An L2 filter is a filter resource that is used to identify a
 * vnic or ring for a packet based on layer 2 fields. Layer 2 fields for
 * encapsulated packets include both outer L2 header and/or inner l2 header of
 * encapsulated packet. The L2 filter resource covers the following OS specific
 * L2 filters. Linux/FreeBSD (per function): # Broadcast enable/disable # List
 * of individual multicast filters # All multicast enable/disable filter #
 * Unicast filters # Promiscuous mode VMware: # Broadcast enable/disable (per
 * physical function) # All multicast enable/disable (per function) # Unicast
 * filters per ring or vnic # Promiscuous mode per PF Windows: # Broadcast
 * enable/disable (per physical function) # List of individual multicast filters
 * (Driver needs to advertise the maximum number of filters supported) # All
 * multicast enable/disable per physical function # Unicast filters per vnic #
 * Promiscuous mode per PF Implementation notes on the use of VNIC in this
 * command: # By default, these filters belong to default vnic for the function.
 * # Once these filters are set up, only destination VNIC can be modified. # If
 * the destination VNIC is not specified in this command, then the HWRM shall
 * only create an l2 context id. HWRM Implementation notes for multicast
 * filters: # The hwrm_filter_alloc command can be used to set up multicast
 * filters (perfect match or partial match). Each individual function driver can
 * set up multicast filters independently. # The HWRM needs to keep track of
 * multicast filters set up by function drivers and maintain multicast group
 * replication records to enable a subset of functions to receive traffic for a
 * specific multicast address. # When a specific multicast filter cannot be set,
 * the HWRM shall return an error. In this error case, the driver should fall
 * back to using one general filter (rather than specific) for all multicast
 * traffic. # When the SR-IOV is enabled, the HWRM needs to additionally track
 * source knockout per multicast group record. Examples of setting unicast
 * filters: For a unicast MAC based filter, one can use a combination of the
 * fields and masks provided in this command to set up the filter. Below are
 * some examples: # MAC + no VLAN filter: This filter is used to identify
 * traffic that does not contain any VLAN tags and matches destination (or
 * source) MAC address. This filter can be set up by setting only l2_addr field
 * to be a valid field. All other fields are not valid. The following value is
 * set for l2_addr. l2_addr = MAC # MAC + Any VLAN filter: This filter is used
 * to identify traffic that carries single VLAN tag and matches (destination or
 * source) MAC address. This filter can be set up by setting only l2_addr and
 * l2_ovlan_mask fields to be valid fields. All other fields are not valid. The
 * following values are set for those two valid fields. l2_addr = MAC,
 * l2_ovlan_mask = 0xFFFF # MAC + no VLAN or VLAN ID=0: This filter is used to
 * identify untagged traffic that does not contain any VLAN tags or a VLAN tag
 * with VLAN ID = 0 and matches destination (or source) MAC address. This filter
 * can be set up by setting only l2_addr and l2_ovlan fields to be valid fields.
 * All other fields are not valid. The following value are set for l2_addr and
 * l2_ovlan. l2_addr = MAC, l2_ovlan = 0x0 # MAC + no VLAN or any VLAN: This
 * filter is used to identify traffic that contains zero or 1 VLAN tag and
 * matches destination (or source) MAC address. This filter can be set up by
 * setting only l2_addr, l2_ovlan, and l2_mask fields to be valid fields. All
 * other fields are not valid. The following value are set for l2_addr,
 * l2_ovlan, and l2_mask fields. l2_addr = MAC, l2_ovlan = 0x0, l2_ovlan_mask =
 * 0xFFFF # MAC + VLAN ID filter: This filter can be set up by setting only
 * l2_addr, l2_ovlan, and l2_ovlan_mask fields to be valid fields. All other
 * fields are not valid. The following values are set for those three valid
 * fields. l2_addr = MAC, l2_ovlan = VLAN ID, l2_ovlan_mask = 0xF000
 */
/* Input (96 bytes) */

struct hwrm_cfa_l2_filter_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_PATH	UINT32_C(0x1)
	/* tx path */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_PATH_LAST	HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_PATH_RX
	/*
	 * Setting of this flag indicates the applicability to the loopback
	 * path.
	 */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_LOOPBACK	UINT32_C(0x2)
	/*
	 * Setting of this flag indicates drop action. If this flag is not set,
	 * then it should be considered accept action.
	 */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_DROP	UINT32_C(0x4)
	/*
	 * If this flag is set, all t_l2_* fields are invalid and they should
	 * not be specified. If this flag is set, then l2_* fields refer to
	 * fields of outermost L2 header.
	 */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_FLAGS_OUTERMOST	UINT32_C(0x8)
	uint32_t enables;
	/* This bit must be '1' for the l2_addr field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_ADDR	UINT32_C(0x1)
	/* This bit must be '1' for the l2_addr_mask field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_ADDR_MASK UINT32_C(0x2)
	/* This bit must be '1' for the l2_ovlan field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_OVLAN	UINT32_C(0x4)
	/* This bit must be '1' for the l2_ovlan_mask field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_OVLAN_MASK UINT32_C(0x8)
	/* This bit must be '1' for the l2_ivlan field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_IVLAN	UINT32_C(0x10)
	/* This bit must be '1' for the l2_ivlan_mask field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_L2_IVLAN_MASK UINT32_C(0x20)
	/* This bit must be '1' for the t_l2_addr field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_T_L2_ADDR   UINT32_C(0x40)
	/* This bit must be '1' for the t_l2_addr_mask field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_T_L2_ADDR_MASK UINT32_C(0x80)
	/* This bit must be '1' for the t_l2_ovlan field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_T_L2_OVLAN  UINT32_C(0x100)
	/* This bit must be '1' for the t_l2_ovlan_mask field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_T_L2_OVLAN_MASK UINT32_C(0x200)
	/* This bit must be '1' for the t_l2_ivlan field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_T_L2_IVLAN  UINT32_C(0x400)
	/* This bit must be '1' for the t_l2_ivlan_mask field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_T_L2_IVLAN_MASK UINT32_C(0x800)
	/* This bit must be '1' for the src_type field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_SRC_TYPE	UINT32_C(0x1000)
	/* This bit must be '1' for the src_id field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_SRC_ID	UINT32_C(0x2000)
	/* This bit must be '1' for the tunnel_type field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_TUNNEL_TYPE UINT32_C(0x4000)
	/* This bit must be '1' for the dst_id field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_DST_ID	UINT32_C(0x8000)
	/* This bit must be '1' for the mirror_vnic_id field to be configured. */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_ENABLES_MIRROR_VNIC_ID UINT32_C(0x10000)
	uint8_t l2_addr[6];
	/*
	 * This value sets the match value for the L2 MAC address. Destination
	 * MAC address for RX path. Source MAC address for TX path.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t l2_addr_mask[6];
	/*
	 * This value sets the mask value for the L2 address. A value of 0 will
	 * mask the corresponding bit from compare.
	 */
	uint16_t l2_ovlan;
	/* This value sets VLAN ID value for outer VLAN. */
	uint16_t l2_ovlan_mask;
	/*
	 * This value sets the mask value for the ovlan id. A value of 0 will
	 * mask the corresponding bit from compare.
	 */
	uint16_t l2_ivlan;
	/* This value sets VLAN ID value for inner VLAN. */
	uint16_t l2_ivlan_mask;
	/*
	 * This value sets the mask value for the ivlan id. A value of 0 will
	 * mask the corresponding bit from compare.
	 */
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t t_l2_addr[6];
	/*
	 * This value sets the match value for the tunnel L2 MAC address.
	 * Destination MAC address for RX path. Source MAC address for TX path.
	 */
	uint8_t unused_4;
	uint8_t unused_5;
	uint8_t t_l2_addr_mask[6];
	/*
	 * This value sets the mask value for the tunnel L2 address. A value of
	 * 0 will mask the corresponding bit from compare.
	 */
	uint16_t t_l2_ovlan;
	/* This value sets VLAN ID value for tunnel outer VLAN. */
	uint16_t t_l2_ovlan_mask;
	/*
	 * This value sets the mask value for the tunnel ovlan id. A value of 0
	 * will mask the corresponding bit from compare.
	 */
	uint16_t t_l2_ivlan;
	/* This value sets VLAN ID value for tunnel inner VLAN. */
	uint16_t t_l2_ivlan_mask;
	/*
	 * This value sets the mask value for the tunnel ivlan id. A value of 0
	 * will mask the corresponding bit from compare.
	 */
	uint8_t src_type;
	/* This value identifies the type of source of the packet. */
	/* Network port */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_NPORT	(UINT32_C(0x0) << 0)
	/* Physical function */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_PF	(UINT32_C(0x1) << 0)
	/* Virtual function */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_VF	(UINT32_C(0x2) << 0)
	/* Virtual NIC of a function */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_VNIC	(UINT32_C(0x3) << 0)
	/* Embedded processor for CFA management */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_KONG	(UINT32_C(0x4) << 0)
	/* Embedded processor for OOB management */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_APE	(UINT32_C(0x5) << 0)
	/* Embedded processor for RoCE */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_BONO	(UINT32_C(0x6) << 0)
	/* Embedded processor for network proxy functions */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_SRC_TYPE_TANG	(UINT32_C(0x7) << 0)
	uint8_t unused_6;
	uint32_t src_id;
	/*
	 * This value is the id of the source. For a network port, it represents
	 * port_id. For a physical function, it represents fid. For a virtual
	 * function, it represents vf_id. For a vnic, it represents vnic_id. For
	 * embedded processors, this id is not valid. Notes: 1. The function ID
	 * is implied if it src_id is not provided for a src_type that is either
	 */
	uint8_t tunnel_type;
	/* Tunnel Type. */
	/* Non-tunnel */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_NONTUNNEL (UINT32_C(0x0) << 0)
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_VXLAN  (UINT32_C(0x1) << 0)
	/* Network Virtualization Generic Routing Encapsulation (NVGRE) */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_NVGRE  (UINT32_C(0x2) << 0)
	/* Generic Routing Encapsulation (GRE) inside Ethernet payload */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_L2GRE  (UINT32_C(0x3) << 0)
	/* IP in IP */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_IPIP   (UINT32_C(0x4) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_GENEVE (UINT32_C(0x5) << 0)
	/* Multi-Protocol Lable Switching (MPLS) */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_MPLS   (UINT32_C(0x6) << 0)
	/* Stateless Transport Tunnel (STT) */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_STT	(UINT32_C(0x7) << 0)
	/* Generic Routing Encapsulation (GRE) inside IP datagram payload */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_IPGRE  (UINT32_C(0x8) << 0)
	/* Any tunneled traffic */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_TUNNEL_TYPE_ANYTUNNEL (UINT32_C(0xff) << 0)
	uint8_t unused_7;
	uint16_t dst_id;
	/*
	 * If set, this value shall represent the Logical VNIC ID of the
	 * destination VNIC for the RX path and network port id of the
	 * destination port for the TX path.
	 */
	uint16_t mirror_vnic_id;
	/* Logical VNIC ID of the VNIC where traffic is mirrored. */
	uint8_t pri_hint;
	/*
	 * This hint is provided to help in placing the filter in the filter
	 * table.
	 */
	/* No preference */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_PRI_HINT_NO_PREFER (UINT32_C(0x0) << 0)
	/* Above the given filter */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_PRI_HINT_ABOVE_FILTER (UINT32_C(0x1) << 0)
	/* Below the given filter */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_PRI_HINT_BELOW_FILTER (UINT32_C(0x2) << 0)
	/* As high as possible */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_PRI_HINT_MAX	(UINT32_C(0x3) << 0)
	/* As low as possible */
	#define HWRM_CFA_L2_FILTER_ALLOC_INPUT_PRI_HINT_MIN	(UINT32_C(0x4) << 0)
	uint8_t unused_8;
	uint32_t unused_9;
	uint64_t l2_filter_id_hint;
	/*
	 * This is the ID of the filter that goes along with the pri_hint. This
	 * field is valid only for the following values. 1 - Above the given
	 * filter 2 - Below the given filter
	 */
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_cfa_l2_filter_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t l2_filter_id;
	/*
	 * This value identifies a set of CFA data structures used for an L2
	 * context.
	 */
	uint32_t flow_id;
	/*
	 * This is the ID of the flow associated with this filter. This value
	 * shall be used to match and associate the flow identifier returned in
	 * completion records. A value of 0xFFFFFFFF shall indicate no flow id.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_l2_filter_free */
/*
 * Description: Free a L2 filter. The HWRM shall free all associated filter
 * resources with the L2 filter.
 */
/* Input (24 bytes) */

struct hwrm_cfa_l2_filter_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t l2_filter_id;
	/*
	 * This value identifies a set of CFA data structures used for an L2
	 * context.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_l2_filter_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_l2_filter_cfg */
/* Description: Change the configuration of an existing L2 filter */
/* Input (40 bytes) */

struct hwrm_cfa_l2_filter_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_FLAGS_PATH_LAST	HWRM_CFA_L2_FILTER_CFG_INPUT_FLAGS_PATH_RX
	/*
	 * Setting of this flag indicates drop action. If this flag is not set,
	 * then it should be considered accept action.
	 */
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_FLAGS_DROP		UINT32_C(0x2)
	uint32_t enables;
	/* This bit must be '1' for the dst_id field to be configured. */
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_ENABLES_DST_ID	UINT32_C(0x1)
	/*
	 * This bit must be '1' for the new_mirror_vnic_id field to be
	 * configured.
	 */
	#define HWRM_CFA_L2_FILTER_CFG_INPUT_ENABLES_NEW_MIRROR_VNIC_ID UINT32_C(0x2)
	uint64_t l2_filter_id;
	/*
	 * This value identifies a set of CFA data structures used for an L2
	 * context.
	 */
	uint32_t dst_id;
	/*
	 * If set, this value shall represent the Logical VNIC ID of the
	 * destination VNIC for the RX path and network port id of the
	 * destination port for the TX path.
	 */
	uint32_t new_mirror_vnic_id;
	/* New Logical VNIC ID of the VNIC where traffic is mirrored. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_l2_filter_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_l2_set_rx_mask */
/* Description: This command will set rx mask of the function. */
/* Input (56 bytes) */

struct hwrm_cfa_l2_set_rx_mask_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t vnic_id;
	/* VNIC ID */
	uint32_t mask;
	/* Reserved for future use. */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_RESERVED	UINT32_C(0x1)
	/*
	 * When this bit is '1', the function is requested to accept multi-cast
	 * packets specified by the multicast addr table.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_MCAST	UINT32_C(0x2)
	/*
	 * When this bit is '1', the function is requested to accept all multi-
	 * cast packets.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ALL_MCAST	UINT32_C(0x4)
	/*
	 * When this bit is '1', the function is requested to accept broadcast
	 * packets.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_BCAST	UINT32_C(0x8)
	/*
	 * When this bit is '1', the function is requested to be put in the
	 * promiscuous mode. The HWRM should accept any function to set up
	 * promiscuous mode. The HWRM shall follow the semantics below for the
	 * promiscuous mode support. # When partitioning is not enabled on a
	 * port (i.e. single PF on the port), then the PF shall be allowed to be
	 * in the promiscuous mode. When the PF is in the promiscuous mode, then
	 * it shall receive all host bound traffic on that port. # When
	 * partitioning is enabled on a port (i.e. multiple PFs per port) and a
	 * PF on that port is in the promiscuous mode, then the PF receives all
	 * traffic within that partition as identified by a unique identifier
	 * for the PF (e.g. S-Tag). If a unique outer VLAN for the PF is
	 * specified, then the setting of promiscuous mode on that PF shall
	 * result in the PF receiving all host bound traffic with matching outer
	 * VLAN. # A VF shall can be set in the promiscuous mode. In the
	 * promiscuous mode, the VF does not receive any traffic unless a unique
	 * outer VLAN for the VF is specified. If a unique outer VLAN for the VF
	 * is specified, then the setting of promiscuous mode on that VF shall
	 * result in the VF receiving all host bound traffic with the matching
	 * outer VLAN. # The HWRM shall allow the setting of promiscuous mode on
	 * a function independently from the promiscuous mode settings on other
	 * functions.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_PROMISCUOUS	UINT32_C(0x10)
	/*
	 * If this flag is set, the corresponding RX filters shall be set up to
	 * cover multicast/broadcast filters for the outermost Layer 2
	 * destination MAC address field.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_OUTERMOST	UINT32_C(0x20)
	/*
	 * If this flag is set, the corresponding RX filters shall be set up to
	 * cover multicast/broadcast filters for the VLAN-tagged packets that
	 * match the TPID and VID fields of VLAN tags in the VLAN tag table
	 * specified in this command.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_VLANONLY	UINT32_C(0x40)
	/*
	 * If this flag is set, the corresponding RX filters shall be set up to
	 * cover multicast/broadcast filters for non-VLAN tagged packets and
	 * VLAN-tagged packets that match the TPID and VID fields of VLAN tags
	 * in the VLAN tag table specified in this command.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_VLAN_NONVLAN	UINT32_C(0x80)
	/*
	 * If this flag is set, the corresponding RX filters shall be set up to
	 * cover multicast/broadcast filters for non-VLAN tagged packets and
	 * VLAN-tagged packets matching any VLAN tag. If this flag is set, then
	 * the HWRM shall ignore VLAN tags specified in vlan_tag_tbl. If none of
	 * vlanonly, vlan_nonvlan, and anyvlan_nonvlan flags is set, then the
	 * HWRM shall ignore VLAN tags specified in vlan_tag_tbl. The HWRM
	 * client shall set at most one flag out of vlanonly, vlan_nonvlan, and
	 * anyvlan_nonvlan.
	 */
	#define HWRM_CFA_L2_SET_RX_MASK_INPUT_MASK_ANYVLAN_NONVLAN UINT32_C(0x100)
	uint64_t mc_tbl_addr;
	/* This is the address for mcast address tbl. */
	uint32_t num_mc_entries;
	/*
	 * This value indicates how many entries in mc_tbl are valid. Each entry
	 * is 6 bytes.
	 */
	uint32_t unused_0;
	uint64_t vlan_tag_tbl_addr;
	/*
	 * This is the address for VLAN tag table. Each VLAN entry in the table
	 * is 4 bytes of a VLAN tag including TPID, PCP, DEI, and VID fields in
	 * network byte order.
	 */
	uint32_t num_vlan_tags;
	/*
	 * This value indicates how many entries in vlan_tag_tbl are valid. Each
	 * entry is 4 bytes.
	 */
	uint32_t unused_1;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_l2_set_rx_mask_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_tunnel_filter_alloc */
/*
 * Description: This is a tunnel filter that uses fields from tunnel header in
 * addition to l2 context. The tunnel filter applies to receive side only. The
 * l2_* fields in this command represent fields of inner L2 header. They are
 * optional to be specified. It allows l2_filter_id to be created with outer L2
 * header fields that can be shared with multiple tunnel filters specified as
 * combinations of inner L2 header fields, tunnel type, and VNI.
 */
/* Input (88 bytes) */

struct hwrm_cfa_tunnel_filter_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Setting of this flag indicates the applicability to the loopback
	 * path.
	 */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_FLAGS_LOOPBACK  UINT32_C(0x1)
	uint32_t enables;
	/* This bit must be '1' for the l2_filter_id field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_L2_FILTER_ID UINT32_C(0x1)
	/* This bit must be '1' for the l2_addr field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_L2_ADDR UINT32_C(0x2)
	/* This bit must be '1' for the l2_ivlan field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_L2_IVLAN UINT32_C(0x4)
	/* This bit must be '1' for the l3_addr field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_L3_ADDR UINT32_C(0x8)
	/* This bit must be '1' for the l3_addr_type field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_L3_ADDR_TYPE UINT32_C(0x10)
	/* This bit must be '1' for the t_l3_addr_type field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_T_L3_ADDR_TYPE UINT32_C(0x20)
	/* This bit must be '1' for the t_l3_addr field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_T_L3_ADDR UINT32_C(0x40)
	/* This bit must be '1' for the tunnel_type field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_TUNNEL_TYPE UINT32_C(0x80)
	/* This bit must be '1' for the vni field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_VNI	UINT32_C(0x100)
	/* This bit must be '1' for the dst_vnic_id field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_DST_VNIC_ID UINT32_C(0x200)
	/* This bit must be '1' for the mirror_vnic_id field to be configured. */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_ENABLES_MIRROR_VNIC_ID UINT32_C(0x400)
	uint64_t l2_filter_id;
	/*
	 * This value identifies a set of CFA data structures used for an L2
	 * context.
	 */
	uint8_t l2_addr[6];
	/*
	 * This value sets the match value for the inner L2 MAC address.
	 * Destination MAC address for RX path. Source MAC address for TX path.
	 */
	uint16_t l2_ivlan;
	/*
	 * This value sets VLAN ID value for inner VLAN. Only 12-bits of VLAN ID
	 * are used in setting the filter.
	 */
	uint32_t l3_addr[4];
	/*
	 * The value of inner destination IP address to be used in filtering.
	 * For IPv4, first four bytes represent the IP address.
	 */
	uint32_t t_l3_addr[4];
	/*
	 * The value of tunnel destination IP address to be used in filtering.
	 * For IPv4, first four bytes represent the IP address.
	 */
	uint8_t l3_addr_type;
	/*
	 * This value indicates the type of inner IP address. 4 - IPv4 6 - IPv6
	 * All others are invalid.
	 */
	uint8_t t_l3_addr_type;
	/*
	 * This value indicates the type of tunnel IP address. 4 - IPv4 6 - IPv6
	 * All others are invalid.
	 */
	uint8_t tunnel_type;
	/* Tunnel Type. */
	/* Non-tunnel */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_NONTUNNEL (UINT32_C(0x0) << 0)
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_VXLAN (UINT32_C(0x1) << 0)
	/* Network Virtualization Generic Routing Encapsulation (NVGRE) */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_NVGRE (UINT32_C(0x2) << 0)
	/* Generic Routing Encapsulation (GRE) inside Ethernet payload */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_L2GRE (UINT32_C(0x3) << 0)
	/* IP in IP */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_IPIP (UINT32_C(0x4) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_GENEVE (UINT32_C(0x5) << 0)
	/* Multi-Protocol Lable Switching (MPLS) */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_MPLS (UINT32_C(0x6) << 0)
	/* Stateless Transport Tunnel (STT) */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_STT (UINT32_C(0x7) << 0)
	/* Generic Routing Encapsulation (GRE) inside IP datagram payload */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_IPGRE (UINT32_C(0x8) << 0)
	/* Any tunneled traffic */
	#define HWRM_CFA_TUNNEL_FILTER_ALLOC_INPUT_TUNNEL_TYPE_ANYTUNNEL (UINT32_C(0xff) << 0)
	uint8_t unused_0;
	uint32_t vni;
	/*
	 * Virtual Network Identifier (VNI). Only valid with tunnel_types VXLAN,
	 * NVGRE, and Geneve. Only lower 24-bits of VNI field are used in
	 * setting up the filter.
	 */
	uint32_t dst_vnic_id;
	/* Logical VNIC ID of the destination VNIC. */
	uint32_t mirror_vnic_id;
	/* Logical VNIC ID of the VNIC where traffic is mirrored. */
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_cfa_tunnel_filter_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t tunnel_filter_id;
	/* This value is an opaque id into CFA data structures. */
	uint32_t flow_id;
	/*
	 * This is the ID of the flow associated with this filter. This value
	 * shall be used to match and associate the flow identifier returned in
	 * completion records. A value of 0xFFFFFFFF shall indicate no flow id.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_tunnel_filter_free */
/* Description: Free a tunnel filter */
/* Input (24 bytes) */

struct hwrm_cfa_tunnel_filter_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t tunnel_filter_id;
	/* This value is an opaque id into CFA data structures. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_tunnel_filter_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_encap_record_alloc */
/*
 * Description: This command is used to create an encapsulation record. The
 * source MAC address and source IP address specified for the source property
 * checks shall be used in the encapsulation where applicable.
 */
/* Input (32 bytes) */

struct hwrm_cfa_encap_record_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Setting of this flag indicates the applicability to the loopback
	 * path.
	 */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_FLAGS_LOOPBACK   UINT32_C(0x1)
	uint8_t encap_type;
	/* Encapsulation Type. */
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_VXLAN (UINT32_C(0x1) << 0)
	/* Network Virtualization Generic Routing Encapsulation (NVGRE) */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_NVGRE (UINT32_C(0x2) << 0)
	/* Generic Routing Encapsulation (GRE) after inside Ethernet payload */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_L2GRE (UINT32_C(0x3) << 0)
	/* IP in IP */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_IPIP (UINT32_C(0x4) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_GENEVE (UINT32_C(0x5) << 0)
	/* Multi-Protocol Lable Switching (MPLS) */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_MPLS (UINT32_C(0x6) << 0)
	/* VLAN */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_VLAN (UINT32_C(0x7) << 0)
	/* Generic Routing Encapsulation (GRE) inside IP datagram payload */
	#define HWRM_CFA_ENCAP_RECORD_ALLOC_INPUT_ENCAP_TYPE_IPGRE (UINT32_C(0x8) << 0)
	uint8_t unused_0;
	uint16_t unused_1;
	uint32_t encap_data[16];
	/* This value is encap data used for the given encap type. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_encap_record_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t encap_record_id;
	/* This value is an opaque id into CFA data structures. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_encap_record_free */
/* Description: Free an encap record */
/* Input (24 bytes) */

struct hwrm_cfa_encap_record_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t encap_record_id;
	/* This value is an opaque id into CFA data structures. */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_encap_record_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_ntuple_filter_alloc */
/*
 * Description: This is a ntuple filter that uses fields from L4/L3 header and
 * optionally fields from L2. The ntuple filters apply to receive traffic only.
 * All L2/L3/L4 header fields are specified in network byte order. These filters
 * can be used for Receive Flow Steering (RFS). # For ethertype value, only
 * 0x0800 (IPv4) and 0x86dd (IPv6) shall be supported for ntuple filters. # If a
 * field specified in this command is not enabled as a valid field, then that
 * field shall not be used in matching packet header fields against this filter.
 */
/* Input (128 bytes) */

struct hwrm_cfa_ntuple_filter_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Setting of this flag indicates the applicability to the loopback
	 * path.
	 */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_FLAGS_LOOPBACK  UINT32_C(0x1)
	/*
	 * Setting of this flag indicates drop action. If this flag is not set,
	 * then it should be considered accept action.
	 */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_FLAGS_DROP	UINT32_C(0x2)
	uint32_t enables;
	/* This bit must be '1' for the l2_filter_id field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_L2_FILTER_ID UINT32_C(0x1)
	/* This bit must be '1' for the ethertype field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_ETHERTYPE UINT32_C(0x2)
	/* This bit must be '1' for the tunnel_type field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_TUNNEL_TYPE UINT32_C(0x4)
	/* This bit must be '1' for the src_macaddr field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_SRC_MACADDR UINT32_C(0x8)
	/* This bit must be '1' for the ipaddr_type field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_IPADDR_TYPE UINT32_C(0x10)
	/* This bit must be '1' for the src_ipaddr field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_SRC_IPADDR UINT32_C(0x20)
	/* This bit must be '1' for the src_ipaddr_mask field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_SRC_IPADDR_MASK UINT32_C(0x40)
	/* This bit must be '1' for the dst_ipaddr field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_DST_IPADDR UINT32_C(0x80)
	/* This bit must be '1' for the dst_ipaddr_mask field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_DST_IPADDR_MASK UINT32_C(0x100)
	/* This bit must be '1' for the ip_protocol field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_IP_PROTOCOL UINT32_C(0x200)
	/* This bit must be '1' for the src_port field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_SRC_PORT UINT32_C(0x400)
	/* This bit must be '1' for the src_port_mask field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_SRC_PORT_MASK UINT32_C(0x800)
	/* This bit must be '1' for the dst_port field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_DST_PORT UINT32_C(0x1000)
	/* This bit must be '1' for the dst_port_mask field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_DST_PORT_MASK UINT32_C(0x2000)
	/* This bit must be '1' for the pri_hint field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_PRI_HINT UINT32_C(0x4000)
	/* This bit must be '1' for the ntuple_filter_id field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_NTUPLE_FILTER_ID UINT32_C(0x8000)
	/* This bit must be '1' for the dst_id field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_DST_ID  UINT32_C(0x10000)
	/* This bit must be '1' for the mirror_vnic_id field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_MIRROR_VNIC_ID UINT32_C(0x20000)
	/* This bit must be '1' for the dst_macaddr field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_ENABLES_DST_MACADDR UINT32_C(0x40000)
	uint64_t l2_filter_id;
	/*
	 * This value identifies a set of CFA data structures used for an L2
	 * context.
	 */
	uint8_t src_macaddr[6];
	/* This value indicates the source MAC address in the Ethernet header. */
	uint16_t ethertype; /* big endian */
	/* This value indicates the ethertype in the Ethernet header. */
	uint8_t ip_addr_type;
	/*
	 * This value indicates the type of IP address. 4 - IPv4 6 - IPv6 All
	 * others are invalid.
	 */
	/* invalid */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_IP_ADDR_TYPE_UNKNOWN (UINT32_C(0x0) << 0)
	/* IPv4 */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_IP_ADDR_TYPE_IPV4 (UINT32_C(0x4) << 0)
	/* IPv6 */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_IP_ADDR_TYPE_IPV6 (UINT32_C(0x6) << 0)
	uint8_t ip_protocol;
	/*
	 * The value of protocol filed in IP header. Applies to UDP and TCP
	 * traffic. 6 - UDP 17 - TCP
	 */
	/* invalid */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_IP_PROTOCOL_UNKNOWN (UINT32_C(0x0) << 0)
	/* UDP */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_IP_PROTOCOL_UDP (UINT32_C(0x6) << 0)
	/* TCP */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_IP_PROTOCOL_TCP (UINT32_C(0x11) << 0)
	uint16_t dst_id;
	/*
	 * If set, this value shall represent the Logical VNIC ID of the
	 * destination VNIC for the RX path and network port id of the
	 * destination port for the TX path.
	 */
	uint16_t mirror_vnic_id;
	/* Logical VNIC ID of the VNIC where traffic is mirrored. */
	uint8_t tunnel_type;
	/*
	 * This value indicates the tunnel type for this filter. If this field
	 * is not specified, then the filter shall apply to both non-tunneled
	 * and tunneled packets. If this field conflicts with the tunnel_type
	 * specified in the l2_filter_id, then the HWRM shall return an error
	 * for this command.
	 */
	/* Non-tunnel */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_NONTUNNEL (UINT32_C(0x0) << 0)
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_VXLAN (UINT32_C(0x1) << 0)
	/* Network Virtualization Generic Routing Encapsulation (NVGRE) */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_NVGRE (UINT32_C(0x2) << 0)
	/* Generic Routing Encapsulation (GRE) inside Ethernet payload */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_L2GRE (UINT32_C(0x3) << 0)
	/* IP in IP */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_IPIP (UINT32_C(0x4) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_GENEVE (UINT32_C(0x5) << 0)
	/* Multi-Protocol Lable Switching (MPLS) */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_MPLS (UINT32_C(0x6) << 0)
	/* Stateless Transport Tunnel (STT) */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_STT (UINT32_C(0x7) << 0)
	/* Generic Routing Encapsulation (GRE) inside IP datagram payload */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_IPGRE (UINT32_C(0x8) << 0)
	/* Any tunneled traffic */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_TUNNEL_TYPE_ANYTUNNEL (UINT32_C(0xff) << 0)
	uint8_t pri_hint;
	/*
	 * This hint is provided to help in placing the filter in the filter
	 * table.
	 */
	/* No preference */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_PRI_HINT_NO_PREFER (UINT32_C(0x0) << 0)
	/* Above the given filter */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_PRI_HINT_ABOVE (UINT32_C(0x1) << 0)
	/* Below the given filter */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_PRI_HINT_BELOW (UINT32_C(0x2) << 0)
	/* As high as possible */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_PRI_HINT_HIGHEST (UINT32_C(0x3) << 0)
	/* As low as possible */
	#define HWRM_CFA_NTUPLE_FILTER_ALLOC_INPUT_PRI_HINT_LOWEST (UINT32_C(0x4) << 0)
	uint32_t src_ipaddr[4]; /* big endian */
	/*
	 * The value of source IP address to be used in filtering. For IPv4,
	 * first four bytes represent the IP address.
	 */
	uint32_t src_ipaddr_mask[4]; /* big endian */
	/*
	 * The value of source IP address mask to be used in filtering. For
	 * IPv4, first four bytes represent the IP address mask.
	 */
	uint32_t dst_ipaddr[4]; /* big endian */
	/*
	 * The value of destination IP address to be used in filtering. For
	 * IPv4, first four bytes represent the IP address.
	 */
	uint32_t dst_ipaddr_mask[4]; /* big endian */
	/*
	 * The value of destination IP address mask to be used in filtering. For
	 * IPv4, first four bytes represent the IP address mask.
	 */
	uint16_t src_port; /* big endian */
	/*
	 * The value of source port to be used in filtering. Applies to UDP and
	 * TCP traffic.
	 */
	uint16_t src_port_mask; /* big endian */
	/*
	 * The value of source port mask to be used in filtering. Applies to UDP
	 * and TCP traffic.
	 */
	uint16_t dst_port; /* big endian */
	/*
	 * The value of destination port to be used in filtering. Applies to UDP
	 * and TCP traffic.
	 */
	uint16_t dst_port_mask; /* big endian */
	/*
	 * The value of destination port mask to be used in filtering. Applies
	 * to UDP and TCP traffic.
	 */
	uint64_t ntuple_filter_id_hint;
	/* This is the ID of the filter that goes along with the pri_hint. */
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_cfa_ntuple_filter_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t ntuple_filter_id;
	/* This value is an opaque id into CFA data structures. */
	uint32_t flow_id;
	/*
	 * This is the ID of the flow associated with this filter. This value
	 * shall be used to match and associate the flow identifier returned in
	 * completion records. A value of 0xFFFFFFFF shall indicate no flow id.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_ntuple_filter_free */
/* Description: Free an ntuple filter */
/* Input (24 bytes) */

struct hwrm_cfa_ntuple_filter_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t ntuple_filter_id;
	/* This value is an opaque id into CFA data structures. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_ntuple_filter_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_ntuple_filter_cfg */
/* Description: Configure an ntuple filter with new destination VNIC */
/* Input (40 bytes) */

struct hwrm_cfa_ntuple_filter_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the new_dst_id field to be configured. */
	#define HWRM_CFA_NTUPLE_FILTER_CFG_INPUT_ENABLES_NEW_DST_ID UINT32_C(0x1)
	/*
	 * This bit must be '1' for the new_mirror_vnic_id field to be
	 * configured.
	 */
	#define HWRM_CFA_NTUPLE_FILTER_CFG_INPUT_ENABLES_NEW_MIRROR_VNIC_ID UINT32_C(0x2)
	uint32_t unused_0;
	uint64_t ntuple_filter_id;
	/* This value is an opaque id into CFA data structures. */
	uint32_t new_dst_id;
	/*
	 * If set, this value shall represent the new Logical VNIC ID of the
	 * destination VNIC for the RX path and new network port id of the
	 * destination port for the TX path.
	 */
	uint32_t new_mirror_vnic_id;
	/* New Logical VNIC ID of the VNIC where traffic is mirrored. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_ntuple_filter_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_em_flow_alloc */
/*
 * Description: This is a generic Exact Match (EM) flow that uses fields from
 * L4/L3/L2 headers. The EM flows apply to transmit and receive traffic. All
 * L2/L3/L4 header fields are specified in network byte order. For each EM flow,
 * there is an associated set of actions specified. For tunneled packets, all
 * L2/L3/L4 fields specified are fields of inner headers unless otherwise
 * specified. # If a field specified in this command is not enabled as a valid
 * field, then that field shall not be used in matching packet header fields
 * against this EM flow entry.
 */
/* Input (112 bytes) */

struct hwrm_cfa_em_flow_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * Enumeration denoting the RX, TX type of the resource. This
	 * enumeration is used for resources that are similar for both TX and RX
	 * paths of the chip.
	 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_PATH		UINT32_C(0x1)
	/* tx path */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_PATH_TX	(UINT32_C(0x0) << 0)
	/* rx path */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_PATH_RX	(UINT32_C(0x1) << 0)
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_PATH_LAST	HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_PATH_RX
	/*
	 * Setting of this flag indicates enabling of a byte counter for a given
	 * flow.
	 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_BYTE_CTR	UINT32_C(0x2)
	/*
	 * Setting of this flag indicates enabling of a packet counter for a
	 * given flow.
	 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_PKT_CTR	UINT32_C(0x4)
	/*
	 * Setting of this flag indicates de-capsulation action for the given
	 * flow.
	 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_DECAP	UINT32_C(0x8)
	/*
	 * Setting of this flag indicates encapsulation action for the given
	 * flow.
	 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_ENCAP	UINT32_C(0x10)
	/*
	 * Setting of this flag indicates drop action. If this flag is not set,
	 * then it should be considered accept action.
	 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_FLAGS_DROP		UINT32_C(0x20)
	uint32_t enables;
	/* This bit must be '1' for the l2_filter_id field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_L2_FILTER_ID  UINT32_C(0x1)
	/* This bit must be '1' for the tunnel_type field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_TUNNEL_TYPE   UINT32_C(0x2)
	/* This bit must be '1' for the tunnel_id field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_TUNNEL_ID	UINT32_C(0x4)
	/* This bit must be '1' for the src_macaddr field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_SRC_MACADDR   UINT32_C(0x8)
	/* This bit must be '1' for the dst_macaddr field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_DST_MACADDR   UINT32_C(0x10)
	/* This bit must be '1' for the ovlan_vid field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_OVLAN_VID	UINT32_C(0x20)
	/* This bit must be '1' for the ivlan_vid field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_IVLAN_VID	UINT32_C(0x40)
	/* This bit must be '1' for the ethertype field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_ETHERTYPE	UINT32_C(0x80)
	/* This bit must be '1' for the src_ipaddr field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_SRC_IPADDR	UINT32_C(0x100)
	/* This bit must be '1' for the dst_ipaddr field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_DST_IPADDR	UINT32_C(0x200)
	/* This bit must be '1' for the ipaddr_type field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_IPADDR_TYPE   UINT32_C(0x400)
	/* This bit must be '1' for the ip_protocol field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_IP_PROTOCOL   UINT32_C(0x800)
	/* This bit must be '1' for the src_port field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_SRC_PORT	UINT32_C(0x1000)
	/* This bit must be '1' for the dst_port field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_DST_PORT	UINT32_C(0x2000)
	/* This bit must be '1' for the dst_id field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_DST_ID	UINT32_C(0x4000)
	/* This bit must be '1' for the mirror_vnic_id field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_MIRROR_VNIC_ID UINT32_C(0x8000)
	/* This bit must be '1' for the encap_record_id field to be configured. */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_ENABLES_ENCAP_RECORD_ID UINT32_C(0x10000)
	uint64_t l2_filter_id;
	/*
	 * This value identifies a set of CFA data structures used for an L2
	 * context.
	 */
	uint8_t tunnel_type;
	/* Tunnel Type. */
	/* Non-tunnel */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_NONTUNNEL (UINT32_C(0x0) << 0)
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_VXLAN	(UINT32_C(0x1) << 0)
	/* Network Virtualization Generic Routing Encapsulation (NVGRE) */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_NVGRE	(UINT32_C(0x2) << 0)
	/* Generic Routing Encapsulation (GRE) inside Ethernet payload */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_L2GRE	(UINT32_C(0x3) << 0)
	/* IP in IP */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_IPIP	(UINT32_C(0x4) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_GENEVE   (UINT32_C(0x5) << 0)
	/* Multi-Protocol Lable Switching (MPLS) */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_MPLS	(UINT32_C(0x6) << 0)
	/* Stateless Transport Tunnel (STT) */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_STT	(UINT32_C(0x7) << 0)
	/* Generic Routing Encapsulation (GRE) inside IP datagram payload */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_IPGRE	(UINT32_C(0x8) << 0)
	/* Any tunneled traffic */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_TUNNEL_TYPE_ANYTUNNEL (UINT32_C(0xff) << 0)
	uint8_t unused_0;
	uint16_t unused_1;
	uint32_t tunnel_id;
	/*
	 * Tunnel identifier. Virtual Network Identifier (VNI). Only valid with
	 * tunnel_types VXLAN, NVGRE, and Geneve. Only lower 24-bits of VNI
	 * field are used in setting up the filter.
	 */
	uint8_t src_macaddr[6];
	/* This value indicates the source MAC address in the Ethernet header. */
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t dst_macaddr[6];
	/*
	 * This value indicates the destination MAC address in the Ethernet
	 * header.
	 */
	uint16_t ovlan_vid;
	/*
	 * This value indicates the VLAN ID of the outer VLAN tag in the
	 * Ethernet header.
	 */
	uint16_t ivlan_vid;
	/*
	 * This value indicates the VLAN ID of the inner VLAN tag in the
	 * Ethernet header.
	 */
	uint16_t ethertype; /* big endian */
	/* This value indicates the ethertype in the Ethernet header. */
	uint8_t ip_addr_type;
	/*
	 * This value indicates the type of IP address. 4 - IPv4 6 - IPv6 All
	 * others are invalid.
	 */
	/* invalid */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_IP_ADDR_TYPE_UNKNOWN (UINT32_C(0x0) << 0)
	/* IPv4 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_IP_ADDR_TYPE_IPV4	(UINT32_C(0x4) << 0)
	/* IPv6 */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_IP_ADDR_TYPE_IPV6	(UINT32_C(0x6) << 0)
	uint8_t ip_protocol;
	/*
	 * The value of protocol filed in IP header. Applies to UDP and TCP
	 * traffic. 6 - UDP 17 - TCP
	 */
	/* invalid */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_IP_PROTOCOL_UNKNOWN  (UINT32_C(0x0) << 0)
	/* UDP */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_IP_PROTOCOL_UDP	(UINT32_C(0x6) << 0)
	/* TCP */
	#define HWRM_CFA_EM_FLOW_ALLOC_INPUT_IP_PROTOCOL_TCP	(UINT32_C(0x11) << 0)
	uint8_t unused_4;
	uint8_t unused_5;
	uint32_t src_ipaddr[4]; /* big endian */
	/*
	 * The value of source IP address to be used in filtering. For IPv4,
	 * first four bytes represent the IP address.
	 */
	uint32_t dst_ipaddr[4]; /* big endian */
	/*
	 * The value of destination IP address to be used in filtering. For
	 * IPv4, first four bytes represent the IP address.
	 */
	uint16_t src_port; /* big endian */
	/*
	 * The value of source port to be used in filtering. Applies to UDP and
	 * TCP traffic.
	 */
	uint16_t dst_port; /* big endian */
	/*
	 * The value of destination port to be used in filtering. Applies to UDP
	 * and TCP traffic.
	 */
	uint16_t dst_id;
	/*
	 * If set, this value shall represent the Logical VNIC ID of the
	 * destination VNIC for the RX path and network port id of the
	 * destination port for the TX path.
	 */
	uint16_t mirror_vnic_id;
	/* Logical VNIC ID of the VNIC where traffic is mirrored. */
	uint32_t encap_record_id;
	/* Logical ID of the encapsulation record. */
	uint32_t unused_6;
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_cfa_em_flow_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t em_filter_id;
	/* This value is an opaque id into CFA data structures. */
	uint32_t flow_id;
	/*
	 * This is the ID of the flow associated with this filter. This value
	 * shall be used to match and associate the flow identifier returned in
	 * completion records. A value of 0xFFFFFFFF shall indicate no flow id.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_em_flow_free */
/* Description: Free an EM flow table entry */
/* Input (24 bytes) */

struct hwrm_cfa_em_flow_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t em_filter_id;
	/* This value is an opaque id into CFA data structures. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_em_flow_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_cfa_em_flow_cfg */
/* Description: Configure an EM flow with new destination VNIC */
/* Input (40 bytes) */

struct hwrm_cfa_em_flow_cfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the new_dst_id field to be configured. */
	#define HWRM_CFA_EM_FLOW_CFG_INPUT_ENABLES_NEW_DST_ID	UINT32_C(0x1)
	/*
	 * This bit must be '1' for the new_mirror_vnic_id field to be
	 * configured.
	 */
	#define HWRM_CFA_EM_FLOW_CFG_INPUT_ENABLES_NEW_MIRROR_VNIC_ID UINT32_C(0x2)
	uint32_t unused_0;
	uint64_t em_filter_id;
	/* This value is an opaque id into CFA data structures. */
	uint32_t new_dst_id;
	/*
	 * If set, this value shall represent the new Logical VNIC ID of the
	 * destination VNIC for the RX path and network port id of the
	 * destination port for the TX path.
	 */
	uint32_t new_mirror_vnic_id;
	/* New Logical VNIC ID of the VNIC where traffic is mirrored. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_cfa_em_flow_cfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_tunnel_dst_port_query */
/*
 * Description: This function is called by a driver to query tunnel type
 * specific destination port configuration.
 */
/* Input (24 bytes) */

struct hwrm_tunnel_dst_port_query_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t tunnel_type;
	/* Tunnel Type. */
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_TUNNEL_DST_PORT_QUERY_INPUT_TUNNEL_TYPE_VXLAN (UINT32_C(0x1) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_TUNNEL_DST_PORT_QUERY_INPUT_TUNNEL_TYPE_GENEVE (UINT32_C(0x5) << 0)
	uint8_t unused_0[7];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_tunnel_dst_port_query_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t tunnel_dst_port_id;
	/*
	 * This field represents the identifier of L4 destination port used for
	 * the given tunnel type. This field is valid for specific tunnel types
	 * that use layer 4 (e.g. UDP) transports for tunneling.
	 */
	uint16_t tunnel_dst_port_val; /* big endian */
	/*
	 * This field represents the value of L4 destination port identified by
	 * tunnel_dst_port_id. This field is valid for specific tunnel types
	 * that use layer 4 (e.g. UDP) transports for tunneling. This field is
	 * in network byte order. A value of 0 means that the destination port
	 * is not configured.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_tunnel_dst_port_alloc */
/*
 * Description: This function is called by a driver to allocate l4 destination
 * port for a specific tunnel type. The destination port value is provided in
 * the input. If the HWRM supports only one global destination port for a tunnel
 * type, then the HWRM shall keep track of its usage as described below. # The
 * first caller that allocates a destination port shall always succeed and the
 * HWRM shall save the destination port configuration for that tunnel type and
 * increment the usage count to 1. # Subsequent callers allocating the same
 * destination port for that tunnel type shall succeed and the HWRM shall
 * increment the usage count for that port for each subsequent caller that
 * succeeds. # Any subsequent caller trying to allocate a different destination
 * port for that tunnel type shall fail until the usage count for the original
 * destination port goes to zero. # A caller that frees a port will cause the
 * usage count for that port to decrement.
 */
/* Input (24 bytes) */

struct hwrm_tunnel_dst_port_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t tunnel_type;
	/* Tunnel Type. */
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_TUNNEL_DST_PORT_ALLOC_INPUT_TUNNEL_TYPE_VXLAN (UINT32_C(0x1) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_TUNNEL_DST_PORT_ALLOC_INPUT_TUNNEL_TYPE_GENEVE (UINT32_C(0x5) << 0)
	uint8_t unused_0;
	uint16_t tunnel_dst_port_val; /* big endian */
	/*
	 * This field represents the value of L4 destination port used for the
	 * given tunnel type. This field is valid for specific tunnel types that
	 * use layer 4 (e.g. UDP) transports for tunneling. This field is in
	 * network byte order. A value of 0 shall fail the command.
	 */
	uint32_t unused_1;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_tunnel_dst_port_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t tunnel_dst_port_id;
	/*
	 * Identifier of a tunnel L4 destination port value. Only applies to
	 * tunnel types that has l4 destination port parameters.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_tunnel_dst_port_free */
/*
 * Description: This function is called by a driver to free l4 destination port
 * for a specific tunnel type.
 */
/* Input (24 bytes) */

struct hwrm_tunnel_dst_port_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t tunnel_type;
	/* Tunnel Type. */
	/* Virtual eXtensible Local Area Network (VXLAN) */
	#define HWRM_TUNNEL_DST_PORT_FREE_INPUT_TUNNEL_TYPE_VXLAN (UINT32_C(0x1) << 0)
	/* Generic Network Virtualization Encapsulation (Geneve) */
	#define HWRM_TUNNEL_DST_PORT_FREE_INPUT_TUNNEL_TYPE_GENEVE (UINT32_C(0x5) << 0)
	uint8_t unused_0;
	uint16_t tunnel_dst_port_id;
	/*
	 * Identifier of a tunnel L4 destination port value. Only applies to
	 * tunnel types that has l4 destination port parameters.
	 */
	uint32_t unused_1;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_tunnel_dst_port_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_stat_ctx_alloc */
/*
 * Description: This command allocates and does basic preparation for a stat
 * context.
 */
/* Input (32 bytes) */

struct hwrm_stat_ctx_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t stats_dma_addr;
	/* This is the address for statistic block. */
	uint32_t update_period_ms;
	/*
	 * The statistic block update period in ms. e.g. 250ms, 500ms, 750ms,
	 * 1000ms.
	 */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_stat_ctx_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t stat_ctx_id;
	/* This is the statistics context ID value. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_stat_ctx_free */
/* Description: This command is used to free a stat context. */
/* Input (24 bytes) */

struct hwrm_stat_ctx_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t stat_ctx_id;
	/* ID of the statistics context that is being queried. */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_stat_ctx_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t stat_ctx_id;
	/* This is the statistics context ID value. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_stat_ctx_query */
/* Description: This command returns statistics of a context. */
/* Input (24 bytes) */

struct hwrm_stat_ctx_query_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t stat_ctx_id;
	/* ID of the statistics context that is being queried. */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (176 bytes) */

struct hwrm_stat_ctx_query_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint64_t tx_ucast_pkts;
	/* Number of transmitted unicast packets */
	uint64_t tx_mcast_pkts;
	/* Number of transmitted multicast packets */
	uint64_t tx_bcast_pkts;
	/* Number of transmitted broadcast packets */
	uint64_t tx_err_pkts;
	/* Number of transmitted packets with error */
	uint64_t tx_drop_pkts;
	/* Number of dropped packets on transmit path */
	uint64_t tx_ucast_bytes;
	/* Number of transmitted bytes for unicast traffic */
	uint64_t tx_mcast_bytes;
	/* Number of transmitted bytes for multicast traffic */
	uint64_t tx_bcast_bytes;
	/* Number of transmitted bytes for broadcast traffic */
	uint64_t rx_ucast_pkts;
	/* Number of received unicast packets */
	uint64_t rx_mcast_pkts;
	/* Number of received multicast packets */
	uint64_t rx_bcast_pkts;
	/* Number of received broadcast packets */
	uint64_t rx_err_pkts;
	/* Number of received packets with error */
	uint64_t rx_drop_pkts;
	/* Number of dropped packets on received path */
	uint64_t rx_ucast_bytes;
	/* Number of received bytes for unicast traffic */
	uint64_t rx_mcast_bytes;
	/* Number of received bytes for multicast traffic */
	uint64_t rx_bcast_bytes;
	/* Number of received bytes for broadcast traffic */
	uint64_t rx_agg_pkts;
	/* Number of aggregated unicast packets */
	uint64_t rx_agg_bytes;
	/* Number of aggregated unicast bytes */
	uint64_t rx_agg_events;
	/* Number of aggregation events */
	uint64_t rx_agg_aborts;
	/* Number of aborted aggregations */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_stat_ctx_clr_stats */
/* Description: This command clears statistics of a context. */
/* Input (24 bytes) */

struct hwrm_stat_ctx_clr_stats_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t stat_ctx_id;
	/* ID of the statistics context that is being queried. */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_stat_ctx_clr_stats_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_fw_reset */
/*
 * Description: This function is called by a driver to self reset the firmware
 * running on the processor indicated by the embedded_proc_type.
 */
/* Input (24 bytes) */

struct hwrm_fw_reset_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t embedded_proc_type;
	/* Type of embedded processor. */
	/* Boot Processor */
	#define HWRM_FW_RESET_INPUT_EMBEDDED_PROC_TYPE_BOOT	(UINT32_C(0x0) << 0)
	/* Management Processor */
	#define HWRM_FW_RESET_INPUT_EMBEDDED_PROC_TYPE_MGMT	(UINT32_C(0x1) << 0)
	/* Network control processor */
	#define HWRM_FW_RESET_INPUT_EMBEDDED_PROC_TYPE_NETCTRL	(UINT32_C(0x2) << 0)
	/* RoCE control processor */
	#define HWRM_FW_RESET_INPUT_EMBEDDED_PROC_TYPE_ROCE	(UINT32_C(0x3) << 0)
	/* Reserved */
	#define HWRM_FW_RESET_INPUT_EMBEDDED_PROC_TYPE_RSVD	(UINT32_C(0x4) << 0)
	uint8_t selfrst_status;
	/* Type of self reset. */
	/* No Self Reset */
	#define HWRM_FW_RESET_INPUT_SELFRST_STATUS_SELFRSTNONE	(UINT32_C(0x0) << 0)
	/* Self Reset as soon as possible to do so safely */
	#define HWRM_FW_RESET_INPUT_SELFRST_STATUS_SELFRSTASAP	(UINT32_C(0x1) << 0)
	/* Self Reset on PCIe Reset */
	#define HWRM_FW_RESET_INPUT_SELFRST_STATUS_SELFRSTPCIERST (UINT32_C(0x2) << 0)
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_fw_reset_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t selfrst_status;
	/* Type of self reset. */
	/* No Self Reset */
	#define HWRM_FW_RESET_OUTPUT_SELFRST_STATUS_SELFRSTNONE   (UINT32_C(0x0) << 0)
	/* Self Reset as soon as possible to do so safely */
	#define HWRM_FW_RESET_OUTPUT_SELFRST_STATUS_SELFRSTASAP   (UINT32_C(0x1) << 0)
	/* Self Reset on PCIe Reset */
	#define HWRM_FW_RESET_OUTPUT_SELFRST_STATUS_SELFRSTPCIERST (UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint16_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_fw_qstatus */
/*
 * Description: This function is called by a driver to query the status of the
 * firmware running on the processor indicated by the embedded_proc_type.
 */
/* Input (24 bytes) */

struct hwrm_fw_qstatus_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint8_t embedded_proc_type;
	/* Type of embedded processor. */
	/* Boot Processor */
	#define HWRM_FW_QSTATUS_INPUT_EMBEDDED_PROC_TYPE_BOOT	(UINT32_C(0x0) << 0)
	/* Management Processor */
	#define HWRM_FW_QSTATUS_INPUT_EMBEDDED_PROC_TYPE_MGMT	(UINT32_C(0x1) << 0)
	/* Network control processor */
	#define HWRM_FW_QSTATUS_INPUT_EMBEDDED_PROC_TYPE_NETCTRL  (UINT32_C(0x2) << 0)
	/* RoCE control processor */
	#define HWRM_FW_QSTATUS_INPUT_EMBEDDED_PROC_TYPE_ROCE	(UINT32_C(0x3) << 0)
	/* Reserved */
	#define HWRM_FW_QSTATUS_INPUT_EMBEDDED_PROC_TYPE_RSVD	(UINT32_C(0x4) << 0)
	uint8_t unused_0[7];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_fw_qstatus_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t selfrst_status;
	/* Type of self reset. */
	/* No Self Reset */
	#define HWRM_FW_QSTATUS_OUTPUT_SELFRST_STATUS_SELFRSTNONE (UINT32_C(0x0) << 0)
	/* Self Reset as soon as possible to do so safely */
	#define HWRM_FW_QSTATUS_OUTPUT_SELFRST_STATUS_SELFRSTASAP (UINT32_C(0x1) << 0)
	/* Self Reset on PCIe Reset */
	#define HWRM_FW_QSTATUS_OUTPUT_SELFRST_STATUS_SELFRSTPCIERST (UINT32_C(0x2) << 0)
	uint8_t unused_0;
	uint16_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_exec_fwd_resp */
/*
 * Description: This command is used to send an encapsulated request to the
 * HWRM. This command instructs the HWRM to execute the request and forward the
 * response of the encapsulated request to the location specified in the
 * original request that is encapsulated. The target id of this command shall be
 * set to 0xFFFF (HWRM). The response location in this command shall be used to
 * acknowledge the receipt of the encapsulated request and forwarding of the
 * response.
 */
/* Input (128 bytes) */

struct hwrm_exec_fwd_resp_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t encap_request[26];
	/*
	 * This is an encapsulated request. This request should be executed by
	 * the HWRM and the response should be provided in the response buffer
	 * inside the encapsulated request.
	 */
	uint16_t encap_resp_target_id;
	/*
	 * This value indicates the target id of the response to the
	 * encapsulated request. 0x0 - 0xFFF8 - Used for function ids 0xFFF8 -
	 * 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_exec_fwd_resp_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_reject_fwd_resp */
/*
 * Description: This command is used to send an encapsulated request to the
 * HWRM. This command instructs the HWRM to reject the request and forward the
 * error response of the encapsulated request to the location specified in the
 * original request that is encapsulated. The target id of this command shall be
 * set to 0xFFFF (HWRM). The response location in this command shall be used to
 * acknowledge the receipt of the encapsulated request and forwarding of the
 * response.
 */
/* Input (128 bytes) */

struct hwrm_reject_fwd_resp_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t encap_request[26];
	/*
	 * This is an encapsulated request. This request should be rejected by
	 * the HWRM and the error response should be provided in the response
	 * buffer inside the encapsulated request.
	 */
	uint16_t encap_resp_target_id;
	/*
	 * This value indicates the target id of the response to the
	 * encapsulated request. 0x0 - 0xFFF8 - Used for function ids 0xFFF8 -
	 * 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_reject_fwd_resp_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_fwd_resp */
/*
 * Description: This command is used to send an encapsulated response to the
 * HWRM. The HWRM shall forward this response based on the target id. The
 * response address provided in this command shall be used to acknowledge the
 * receipt of the encapsulated response. The encapsulated response address
 * provided in this command shall be used to provide the encapsulated response.
 */
/* Input (40 bytes) */

struct hwrm_fwd_resp_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t encap_resp_target_id;
	/*
	 * This value indicates the target id of the encapsulated response. 0x0
	 * - 0xFFF8 - Used for function ids 0xFFF8 - 0xFFFE - Reserved for
	 * internal processors 0xFFFF - HWRM
	 */
	uint16_t encap_resp_cmpl_ring;
	/*
	 * This value indicates the completion ring the encapsulated response
	 * will be optionally completed on. If the value is -1, then no CR
	 * completion shall be generated for the encapsulated response. Any
	 * other value must be a valid CR ring_id value. If a valid
	 * encap_resp_cmpl_ring is provided, then a CR completion shall be
	 * generated for the encapsulated response.
	 */
	uint16_t encap_resp_len;
	/* This field indicates the length of encapsulated response. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint64_t encap_resp_addr;
	/*
	 * This is the host address where the encapsulated response will be
	 * written. This area must be 16B aligned and must be cleared to zero
	 * before the original request is made.
	 */
	uint32_t encap_resp[24];
	/* This is an encapsulated response. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_fwd_resp_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_fwd_async_event_cmpl */
/*
 * Description: This command is used to send an encapsulated asynchronous event
 * completion to the HWRM. The HWRM shall forward this asynchronous event
 * completion to target(s) specified in the command. The HWRM shall complete
 * this command only after forwarding asynchronous event completion to specified
 * targets.
 */
/* Input (32 bytes) */

struct hwrm_fwd_async_event_cmpl_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t encap_async_event_target_id;
	/*
	 * This value indicates the target id of the encapsulated asynchronous
	 * event. 0x0 - 0xFFF8 - Used for function ids 0xFFF8 - 0xFFFE -
	 * Reserved for internal processors 0xFFFF - Broadcast to all children
	 * VFs (only applicable when a PF is the requester)
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2[3];
	uint8_t unused_3;
	uint32_t encap_async_event_cmpl[4];
	/* This is an encapsulated asynchronous event completion. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_fwd_async_event_cmpl_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_temp_monitor_query */
/*
 * Description: A temperature monitor is used to query the device temperature.
 */
/* Input (16 bytes) */

struct hwrm_temp_monitor_query_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_temp_monitor_query_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t temp;
	/* The HWRM shall provide the current temperature of device in Celsius. */
	uint8_t unused_0;
	uint16_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_wol_filter_alloc */
/*
 * Description: A Wake-On-LAN (WoL) filter is a filter resource that is used to
 * identify a WoL packet. # Among all function drivers, the HWRM shall only
 * allow PF drivers to allocate WoL filters. # The HWRM shall not allow VF
 * drivers to allocate any WoL filters. # When partitioning is enabled and WoL
 * is supported, the HWRM shall support at least one WoL filter per partition. #
 * The HWRM shall retain a WoL filter setting until the filter is freed. # If
 * the HWRM client is a function driver, then the HWRM shall not allow the HWRM
 * client to set up WoL filters on the port that the function is not associated
 * with. # If the HWRM client is one of the trusted embedded services (e.g.
 * management service), the the HWRM shall allow the HWRM client to set up WoL
 * filters on any port of the device.
 */
/* Input (64 bytes) */

struct hwrm_wol_filter_alloc_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	uint32_t enables;
	/* This bit must be '1' for the mac_address field to be configured. */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_ENABLES_MAC_ADDRESS	UINT32_C(0x1)
	/* This bit must be '1' for the pattern_offset field to be configured. */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_ENABLES_PATTERN_OFFSET UINT32_C(0x2)
	/* This bit must be '1' for the pattern_size field to be configured. */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_ENABLES_PATTERN_SIZE   UINT32_C(0x4)
	/* This bit must be '1' for the pattern_buf_addr field to be configured. */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_ENABLES_PATTERN_BUF_ADDR UINT32_C(0x8)
	/*
	 * This bit must be '1' for the pattern_mask_addr field to be
	 * configured.
	 */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_ENABLES_PATTERN_MASK_ADDR UINT32_C(0x10)
	uint16_t port_id;
	/* Port ID of port on which WoL filter is configured. */
	uint8_t wol_type;
	/* This value represents a Wake-on-LAN type. */
	/* Magic Paket */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_WOL_TYPE_MAGICPKT	(UINT32_C(0x0) << 0)
	/* Bitmap */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_WOL_TYPE_BMP	(UINT32_C(0x1) << 0)
	/* Invalid */
	#define HWRM_WOL_FILTER_ALLOC_INPUT_WOL_TYPE_INVALID	(UINT32_C(0xff) << 0)
	uint8_t unused_0;
	uint32_t unused_1;
	uint8_t mac_address[6];
	/*
	 * # If this field is enabled and magic packet WoL filter type is
	 * specified in this command, the value set in this field shall be used
	 * in setting the magic packet based WoL filter. # If this field is not
	 * enabled and magic packet WoL filter type is specified and port id is
	 * specified to 0xFF in this command, then the HWRM shall use default
	 * MAC address configured on the function associated with the HWRM
	 * client. # If this field is not enabled and magic packet WoL filter
	 * type is specified and port id is not specified to 0xFF in this
	 * command, then the HWRM shall use default MAC address configured on
	 * the port.
	 */
	uint16_t pattern_offset;
	/*
	 * The offset from the beginning of MAC header where pattern should be
	 * matched. Applies to bitmap WoL.
	 */
	uint16_t pattern_size;
	/* The size of the pattern that is being matched. Applies to bitmap WoL. */
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4[3];
	uint8_t unused_5;
	uint64_t pattern_buf_addr;
	/* Physical address of the pattern buffer. Applies to bitmap WoL. */
	uint64_t pattern_mask_addr;
	/* Physical address of the pattern mask. Applies to bitmap WoL. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_wol_filter_alloc_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint8_t wol_filter_id;
	/* This value identifies a Wake-on-LAN (WoL) filter. */
	uint8_t unused_0;
	uint16_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_wol_filter_free */
/*
 * Description: Free a WoL filter. # Among all function drivers, the HWRM shall
 * only allow PF drivers to free WoL filters. # The HWRM shall not allow VF
 * drivers to free any WoL filters. # The HWRM shall not allow a function driver
 * to free an Out-Of-Box WoL filter. # The HWRM shall not allow a function
 * driver to free a WoL filter on a port that the corresponding function is not
 * associated with. # The HWRM shall not allow a function driver to free a WoL
 * filter on a function that the function driver is not associated with.
 */
/* Input (32 bytes) */

struct hwrm_wol_filter_free_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t flags;
	/*
	 * # When this bit is set to '1', then all active WoL filters on the
	 * port are requested to be freed. # If the a function driver sets this
	 * flag to '1', then the HWRM shall free all active WoL filters that are
	 * not set by other function drivers on that port.
	 */
	#define HWRM_WOL_FILTER_FREE_INPUT_FLAGS_FREE_ALL_WOL_FILTERS UINT32_C(0x1)
	uint32_t enables;
	/* This bit must be '1' for the wol_filter_id field to be configured. */
	#define HWRM_WOL_FILTER_FREE_INPUT_ENABLES_WOL_FILTER_ID   UINT32_C(0x1)
	uint16_t port_id;
	/* Port ID of the port on which WoL filter(s) is (are) being freed. */
	uint8_t wol_filter_id;
	/* The HWRM shall ignore this field if free_all_wol_filters flag is set. */
	uint8_t unused_0[5];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_wol_filter_free_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_wol_filter_qcfg */
/*
 * Description: Query WoL filter configuration. # Among all function drivers,
 * the HWRM shall only allow PF drivers to query WoL filters. # The HWRM shall
 * not allow VF drivers to query any WoL filters. # The HWRM shall return WoL
 * filters that are active on the associated port for which this query is being
 * performed. # If the HWRM client is a function driver, then the HWRM shall not
 * allow the HWRM client to query WoL filters that are set up by other function
 * drivers. # If the HWRM client is a function driver, then the HWRM shall not
 * allow the HWRM client to query WoL filters on the port that the function is
 * not associated with. # If the HWRM client is one of the trusted embedded
 * service (e.g. management service), the the HWRM shall allow the HWRM client
 * to query WoL filters on any port of the device.
 */
/* Input (56 bytes) */

struct hwrm_wol_filter_qcfg_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t port_id;
	/* Port ID of port on which WoL filter that is being queried. */
	uint16_t handle;
	/*
	 * This is an opaque handle used to access filters. # The HWRM client
	 * shall set this field to 0x0000 to begin the query. # After the first
	 * query, the HWRM client shall retrieve next filters (if they exist)
	 * using the HWRM provided handle in the response.
	 */
	uint32_t unused_0;
	uint64_t pattern_buf_addr;
	/*
	 * Physical address of the pattern buffer. Applies to bitmap WoL filter
	 * only. # Value of 0 indicates an invalid buffer address. If this field
	 * is set to 0, then HWRM shall ignore pattern_buf_size. # If the HWRM
	 * client provides an invalid buffer address for the pattern, then the
	 * HWRM is not required to provide pattern when the response contains a
	 * bitmap WoL filter.
	 */
	uint64_t pattern_buf_size;
	/* The sixe of the pattern buffer. Applies to bitmap WoL filter only. */
	uint64_t pattern_mask_addr;
	/*
	 * Physical address of the pattern mask. Applies to bitmap WoL filter
	 * only. # Value of 0 indicates an invalid pattern mask address. If this
	 * field is set to 0, then HWRM shall ignore pattern_mask_size. # If the
	 * HWRM client provides an invalid mask address for the pattern, then
	 * the HWRM is not required to provide mask when the response contains a
	 * bitmap WoL filter.
	 */
	uint64_t pattern_mask_size;
	/*
	 * The size of the buffer for pattern mask. Applies to bitmap WoL filter
	 * only.
	 */
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_wol_filter_qcfg_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t next_handle;
	/*
	 * This is the next handle that is used to access filters. # If this
	 * field is set to 0x0000, then no WoL filters are currently configured
	 * on this port and all other fields in the output shall be ignored by
	 * the HWRM client. # If this field is set to neither 0x0000 nor 0xFFFF,
	 * then the wol_filter_id is valid and the parameters provided in the
	 * response are based on the wol_type. # If this field is set to 0xFFFF,
	 * then there are no remaining configured WoL filters to be queried for
	 * the queried function after this response, wol_filter_id is valid and
	 * the parameters provided in the response are based on the wol_type.
	 */
	uint8_t wol_filter_id;
	/* This value identifies the filter returned in this response. */
	uint8_t wol_type;
	/*
	 * This value identifies the type of WoL filter returned in this
	 * response.
	 */
	/* Magic Paket */
	#define HWRM_WOL_FILTER_QCFG_OUTPUT_WOL_TYPE_MAGICPKT	(UINT32_C(0x0) << 0)
	/* Bitmap */
	#define HWRM_WOL_FILTER_QCFG_OUTPUT_WOL_TYPE_BMP	(UINT32_C(0x1) << 0)
	/* Invalid */
	#define HWRM_WOL_FILTER_QCFG_OUTPUT_WOL_TYPE_INVALID	(UINT32_C(0xff) << 0)
	uint32_t unused_0;
	uint8_t mac_address[6];
	/*
	 * The MAC address value used by the WoL filter. Applies to magic packet
	 * based WoL.
	 */
	uint16_t pattern_offset;
	/*
	 * The offset from the beginning of MAC header where pattern should be
	 * matched. Applies to bitmap WoL.
	 */
	uint16_t pattern_size;
	/* The size of the pattern that is being matched. Applies to bitmap WoL. */
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t unused_4;
	uint8_t unused_5;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_dbg_dump */
/*
 * Description: This command is used by to initiate the dump of debug
 * information to a driver specified address.
 */
/* Input (40 bytes) */

struct hwrm_dbg_dump_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t handle;
	/*
	 * Handle used to dump debug data. handle = 0 indicates the beginning of
	 * the dump. handle != 0 indicates the request to dump the next part.
	 */
	uint32_t unused_0;
	uint64_t host_dbg_dump_addr;
	/*
	 * Address of the host buffer where the debug data is requested to be
	 * dumped.
	 */
	uint64_t host_dbg_dump_addr_len;
	/* Length of host buffer used for transferring debug data. */
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_dbg_dump_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t nexthandle;
	/*
	 * Handle used to indicate availability of additional debug data.
	 * nexthandle = 0 indicates that there is no more debug data available.
	 * nexthandle != 0 indicates the handle value that should be used to
	 * request the next part of debug data.
	 */
	uint32_t dbg_data_len;
	/* The number of bytes of debug data written to debug dump buffer. */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_raw_write_blk */
/*
 * Note: Write an unmanaged block of data at any physical offset within the
 * NVRAM. Used for initial provisioning/manufacturing purposes only. Implemented
 * in the ChiMP boot-strap firmware (fwutil.bin) only.
 */
/* Input (32 bytes) */

struct hwrm_nvm_raw_write_blk_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t host_src_addr;
	/*
	 * 64-bit Host Source Address. This is the loation of the source data to
	 * be written.
	 */
	uint32_t dest_addr;
	/*
	 * 32-bit Destination Address. This is the NVRAM byte-offset where the
	 * source data will be written to.
	 */
	uint32_t len;
	/* Length of data to be written, in bytes. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_raw_write_blk_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_read */
/*
 * Note: Read the contents of an NVRAM item as referenced (indexed) by an
 * existing directory entry.
 */
/* Input (40 bytes) */

struct hwrm_nvm_read_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t host_dest_addr;
	/*
	 * 64-bit Host Destination Address. This is the host address where the
	 * data will be written to.
	 */
	uint16_t dir_idx;
	/* The 0-based index of the directory entry. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint32_t offset;
	/* The NVRAM byte-offset to read from. */
	uint32_t len;
	/* The length of the data to be read, in bytes. */
	uint32_t unused_2;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_read_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_raw_dump */
/* Note: Dump a raw block of data from NVRAM. */
/* Input (32 bytes) */

struct hwrm_nvm_raw_dump_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t host_dest_addr;
	/*
	 * 64-bit Host Destination Address. This is the host address where the
	 * data will be written to.
	 */
	uint32_t offset;
	/* 32-bit NVRAM byte-offset to read from. */
	uint32_t len;
	/* Total length of NVRAM contents to be read, in bytes. */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_raw_dump_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_get_dir_entries */
/*
 * Description: Read the NVRAM directory. Each directory entry is at least 24
 * bytes in length and contains the: - 16-bit directory entry type
 * (BNX_DIR_TYPE_* value) - 16-bit ordinal (instance of this directory entry
 * type) - 16-bit extension flags (identifies inactive entries and entries for
 * firmware update) - 16-bit attribute flags (identifies entries with a
 * purposely invalid chksum value) - 32-bit byte-offset into NVRAM where this
 * item data is located - 32-bit length of allocated NVRAM for item, in bytes
 * (multiple of block size) - 32-bit length of data (excluding padding), in
 * bytes (may be 0) - 32-bit data checksum (CRC-32) See the
 * bnxnvm_directory_entry_t definition in the file bnxnvm_defs.h.
 */
/* Input (24 bytes) */

struct hwrm_nvm_get_dir_entries_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t host_dest_addr;
	/*
	 * 64-bit Host Destination Address. This is the host address where the
	 * directory will be written.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_get_dir_entries_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_get_dir_info */
/* Note: Get Directory Header info. */
/* Input (16 bytes) */

struct hwrm_nvm_get_dir_info_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (24 bytes) */

struct hwrm_nvm_get_dir_info_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t entries;
	/* Number of directory entries in the directory. */
	uint32_t entry_length;
	/* Size of each directory entry, in bytes. */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_write */
/*
 * Note: Write to the allocated NVRAM of an item referenced by an existing
 * directory entry.
 */
/* Input (48 bytes) */

struct hwrm_nvm_write_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t host_src_addr;
	/* 64-bit Host Source Address. This is where the source data is. */
	uint16_t dir_type;
	/*
	 * The Directory Entry Type (valid values are defined in the
	 * bnxnvm_directory_type enum defined in the file bnxnvm_defs.h).
	 */
	uint16_t dir_ordinal;
	/*
	 * Directory ordinal. The 0-based instance of the combined Directory
	 * Entry Type and Extension.
	 */
	uint16_t dir_ext;
	/*
	 * The Directory Entry Extension flags (see BNX_DIR_EXT_* in the file
	 * bnxnvm_defs.h).
	 */
	uint16_t dir_attr;
	/*
	 * Directory Entry Attribute flags (see BNX_DIR_ATTR_* in the file
	 * bnxnvm_defs.h).
	 */
	uint32_t dir_data_length;
	/*
	 * Length of data to write, in bytes. May be less than or equal to the
	 * allocated size for the directory entry. The data length stored in the
	 * directory entry will be updated to reflect this value once the write
	 * is complete.
	 */
	uint16_t option;
	/* Option. */
	uint16_t flags;
	/*
	 * When this bit is '1', the original active image will not be removed.
	 * TBD: what purpose is this?
	 */
	#define HWRM_NVM_WRITE_INPUT_FLAGS_KEEP_ORIG_ACTIVE_IMG	UINT32_C(0x1)
	uint32_t dir_item_length;
	/*
	 * The requested length of the allocated NVM for the item, in bytes.
	 * This value may be greater than or equal to the specified data length
	 * (dir_data_length). If this value is less than the specified data
	 * length, it will be ignored. The response will contain the actual
	 * allocated item length, which may be greater than the requested item
	 * length. The purpose for allocating more than the required number of
	 * bytes for an item's data is to pre-allocate extra storage (padding)
	 * to accomodate the potential future growth of an item (e.g. upgraded
	 * firmware with a size increase, log growth, expanded configuration
	 * data).
	 */
	uint32_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_write_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t dir_item_length;
	/*
	 * Length of the allocated NVM for the item, in bytes. The value may be
	 * greater than or equal to the specified data length or the requested
	 * item length. The actual item length used when creating a new
	 * directory entry will be a multiple of an NVM block size.
	 */
	uint16_t dir_idx;
	/* The directory index of the created or modified item. */
	uint8_t unused_0;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_modify */
/*
 * Note: Modify the contents of an NVRAM item as referenced (indexed) by an
 * existing directory entry.
 */
/* Input (40 bytes) */

struct hwrm_nvm_modify_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint64_t host_src_addr;
	/* 64-bit Host Source Address. This is where the modified data is. */
	uint16_t dir_idx;
	/* 16-bit directory entry index. */
	uint8_t unused_0;
	uint8_t unused_1;
	uint32_t offset;
	/* 32-bit NVRAM byte-offset to modify content from. */
	uint32_t len;
	/*
	 * Length of data to be modified, in bytes. The length shall be non-
	 * zero.
	 */
	uint32_t unused_2;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_modify_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_find_dir_entry */
/*
 * Note: Search a directory entry in the directory by either directory entry
 * index or directory entry parameters.
 */
/* Input (32 bytes) */

struct hwrm_nvm_find_dir_entry_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the dir_idx_valid field to be configured. */
	#define HWRM_NVM_FIND_DIR_ENTRY_INPUT_ENABLES_DIR_IDX_VALID UINT32_C(0x1)
	uint16_t dir_idx;
	/* Directory Entry Index */
	uint16_t dir_type;
	/* Directory Entry (Image) Type */
	uint16_t dir_ordinal;
	/* Directory ordinal. The instance of this Directory Type */
	uint16_t dir_ext;
	/* The Directory Entry Extension flags. */
	uint8_t opt_ordinal;
	/* This value indicates the search option using dir_ordinal. */
	#define HWRM_NVM_FIND_DIR_ENTRY_INPUT_OPT_ORDINAL_MASK	UINT32_C(0x3)
	#define HWRM_NVM_FIND_DIR_ENTRY_INPUT_OPT_ORDINAL_SFT	0
	/* Equal to specified ordinal value. */
	#define HWRM_NVM_FIND_DIR_ENTRY_INPUT_OPT_ORDINAL_EQ	(UINT32_C(0x0) << 0)
	/* Greater than or equal to specified ordinal value */
	#define HWRM_NVM_FIND_DIR_ENTRY_INPUT_OPT_ORDINAL_GE	(UINT32_C(0x1) << 0)
	/* Greater than specified ordinal value */
	#define HWRM_NVM_FIND_DIR_ENTRY_INPUT_OPT_ORDINAL_GT	(UINT32_C(0x2) << 0)
	uint8_t unused_1[3];
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_nvm_find_dir_entry_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t dir_item_length;
	/* Allocated NVRAM for this directory entry, in bytes. */
	uint32_t dir_data_length;
	/* Size of the stored data for this directory entry, in bytes. */
	uint32_t fw_ver;
	/*
	 * Firmware version. Only valid if the directory entry is for embedded
	 * firmware stored in APE_BIN Format.
	 */
	uint16_t dir_ordinal;
	/* Directory ordinal. */
	uint16_t dir_idx;
	/* Directory Entry Index */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_erase_dir_entry */
/*
 * Note: Remove a directory entry specified by the directory entry index from
 * the directory.
 */
/* Input (24 bytes) */

struct hwrm_nvm_erase_dir_entry_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t dir_idx;
	/* Directory Entry Index */
	uint16_t unused_0[3];
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_erase_dir_entry_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_get_dev_info */
/*
 * Note: Get device info. Return Manufacturer_ID, Device_ID, block_size,
 * nvram_size, reserved_size and available_size.
 */
/* Input (16 bytes) */

struct hwrm_nvm_get_dev_info_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
} __attribute__((packed));

/* Output (32 bytes) */

struct hwrm_nvm_get_dev_info_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint16_t manufacturer_id;
	/* Manufacturer ID. */
	uint16_t device_id;
	/* Device ID. */
	uint32_t sector_size;
	/* Sector size of the NVRAM device. */
	uint32_t nvram_size;
	/* Total size, in bytes of the NVRAM device. */
	uint32_t reserved_size;
	uint32_t available_size;
	/*
	 * Available size that can be used, in bytes. Available size is the
	 * NVRAM size take away the used size and reserved size.
	 */
	uint8_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_mod_dir_entry */
/* Note: Modify a directory entry parameters in the directory. */
/* Input (32 bytes) */

struct hwrm_nvm_mod_dir_entry_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint32_t enables;
	/* This bit must be '1' for the checksum field to be configured. */
	#define HWRM_NVM_MOD_DIR_ENTRY_INPUT_ENABLES_CHECKSUM	UINT32_C(0x1)
	uint16_t dir_idx;
	/* Directory Entry Index */
	uint16_t dir_ordinal;
	/* Directory ordinal. The (0-based) instance of this Directory Type. */
	uint16_t dir_ext;
	/*
	 * The Directory Entry Extension flags (see BNX_DIR_EXT_* for extension
	 * flag definitions).
	 */
	uint16_t dir_attr;
	/*
	 * Directory Entry Attribute flags (see BNX_DIR_ATTR_* for attribute
	 * flag definitions).
	 */
	uint32_t checksum;
	/*
	 * If valid, then this field updates the checksum value of the content
	 * in the directory entry.
	 */
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_mod_dir_entry_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

/* hwrm_nvm_verify_update */
/*
 * Description: Verify updated content of a directory entry. Before this
 * verification, there should be two valid directory entries of the given
 * directory type (one with "UPDATE" directory extension flag and the current
 * one "ACTIVE"). Below are steps the HWRM performs for executing this command:
 * # The HWRM finds the directory entry with "UDPATE" extension flag based on
 * input parameters. The new directory entry should already have updated
 * contents. # The HWRM performs signature verification of the updated content.
 * # If the signature verification is successful, the two directory entries are
 * switched (the verified updated entry is made active and the current "ACTIVE"
 * entry is marked with "UPDATE" extension flag). Implementation notes: # The
 * HWRM shall allow this command to be requested against any dir_type value (and
 * not limit it to a subset). # In the case of an updated HWRM firmware, the new
 * firmware version shall not automatically take effect (i.e. be executed).
 */
/* Input (24 bytes) */

struct hwrm_nvm_verify_update_input {
	uint16_t req_type;
	/*
	 * This value indicates what type of request this is. The format for the
	 * rest of the command is determined by this field.
	 */
	uint16_t cmpl_ring;
	/*
	 * This value indicates the what completion ring the request will be
	 * optionally completed on. If the value is -1, then no CR completion
	 * will be generated. Any other value must be a valid CR ring_id value
	 * for this function.
	 */
	uint16_t seq_id;
	/* This value indicates the command sequence number. */
	uint16_t target_id;
	/*
	 * Target ID of this command. 0x0 - 0xFFF8 - Used for function ids
	 * 0xFFF8 - 0xFFFE - Reserved for internal processors 0xFFFF - HWRM
	 */
	uint64_t resp_addr;
	/*
	 * This is the host address where the response will be written when the
	 * request is complete. This area must be 16B aligned and must be
	 * cleared to zero before the request is made.
	 */
	uint16_t dir_type;
	/* Directory Entry Type, to be verified. */
	uint16_t dir_ordinal;
	/* Directory ordinal. The instance of the Directory Type to be verified. */
	uint16_t dir_ext;
	/*
	 * The Directory Entry Extension flags. The "UPDATE" extension flag must
	 * be set in this value. A corresponding directory entry with the same
	 * type and ordinal values but *without* the "UPDATE" extension flag
	 * must also exist. The other flags of the extension must be identical
	 * between the active and update entries.
	 */
	uint16_t unused_0;
} __attribute__((packed));

/* Output (16 bytes) */

struct hwrm_nvm_verify_update_output {
	uint16_t error_code;
	/*
	 * Pass/Fail or error type Note: receiver to verify the in parameters,
	 * and fail the call with an error when appropriate
	 */
	uint16_t req_type;
	/* This field returns the type of original request. */
	uint16_t seq_id;
	/* This field provides original sequence number of the command. */
	uint16_t resp_len;
	/*
	 * This field is the length of the response in bytes. The last byte of
	 * the response is a valid flag that will read as '1' when the command
	 * has been completely written to memory.
	 */
	uint32_t unused_0;
	uint8_t unused_1;
	uint8_t unused_2;
	uint8_t unused_3;
	uint8_t valid;
	/*
	 * This field is used in Output records to indicate that the output is
	 * completely written to RAM. This field should be read as '1' to
	 * indicate that the output has been completely written. When writing a
	 * command completion or response to an internal processor, the order of
	 * writes has to be such that this field is written last.
	 */
} __attribute__((packed));

#endif /* _HSI_STRUCT_DEF_EXTERNAL_H_ */
