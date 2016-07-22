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
#include <sys/sysctl.h>

#include "bnxt.h"
#include "bnxt_hwrm.h"
#include "bnxt_sysctl.h"

static int bnxt_vlan_only_sysctl(SYSCTL_HANDLER_ARGS);
/*
 * We want to create:
 * dev.bnxt.0.hwstats.txq0
 * dev.bnxt.0.hwstats.txq0.txmbufs
 * dev.bnxt.0.hwstats.rxq0
 * dev.bnxt.0.hwstats.txq0.rxmbufs
 * so the hwstats ctx list needs to be created in attach_post and populated
 * during init.
 *
 * Then, it needs to be cleaned up in stop.
 */

int
bnxt_init_sysctl_ctx(struct bnxt_softc *softc)
{
	struct sysctl_ctx_list *ctx;

	sysctl_ctx_init(&softc->hw_stats);
	ctx = device_get_sysctl_ctx(softc->dev);
	softc->hw_stats_oid = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(softc->dev)), OID_AUTO,
	    "hwstats", CTLFLAG_RD, 0, "hardware statistics");
	if (!softc->hw_stats_oid) {
		sysctl_ctx_free(&softc->hw_stats);
		return ENOMEM;
	}

	sysctl_ctx_init(&softc->ver_info->ver_ctx);
	ctx = device_get_sysctl_ctx(softc->dev);
	softc->ver_info->ver_oid = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(softc->dev)), OID_AUTO,
	    "ver", CTLFLAG_RD, 0, "hardware/firmware version information");
	if (!softc->ver_info->ver_oid) {
		sysctl_ctx_free(&softc->ver_info->ver_ctx);
		return ENOMEM;
	}

	return 0;
}

int
bnxt_free_sysctl_ctx(struct bnxt_softc *softc)
{
	int orc;
	int rc = 0;

	if (softc->hw_stats_oid != NULL) {
		orc = sysctl_ctx_free(&softc->hw_stats);
		if (orc)
			rc = orc;
		else
			softc->hw_stats_oid = NULL;
	}
	if (softc->ver_info->ver_oid != NULL) {
		orc = sysctl_ctx_free(&softc->ver_info->ver_ctx);
		if (orc)
			rc = orc;
		else
			softc->ver_info->ver_oid = NULL;
	}

	return rc;
}

int
bnxt_create_tx_sysctls(struct bnxt_softc *softc, int txr)
{
	struct sysctl_oid *oid;
	struct ctx_hw_stats *tx_stats = (void *)softc->tx_stats.idi_vaddr;
	char	name[32];
	char	desc[64];

	sprintf(name, "txq%d", txr);
	sprintf(desc, "transmit queue %d", txr);
	oid = SYSCTL_ADD_NODE(&softc->hw_stats,
	    SYSCTL_CHILDREN(softc->hw_stats_oid), OID_AUTO, name, CTLFLAG_RD, 0,
	    desc);
	if (!oid)
		return ENOMEM;


	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "ucast_pkts", CTLFLAG_RD, &tx_stats[txr].tx_ucast_pkts,
	    "unicast packets sent");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "mcast_pkts", CTLFLAG_RD, &tx_stats[txr].tx_mcast_pkts,
	    "multicast packets sent");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "bcast_pkts", CTLFLAG_RD, &tx_stats[txr].tx_bcast_pkts,
	    "broadcast packets sent");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "discard_pkts", CTLFLAG_RD,
	    &tx_stats[txr].tx_discard_pkts, "discarded transmit packets");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "drop_pkts", CTLFLAG_RD, &tx_stats[txr].tx_drop_pkts,
	    "dropped transmit packets");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "ucast_bytes", CTLFLAG_RD, &tx_stats[txr].tx_ucast_bytes,
	    "unicast bytes sent");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "mcast_bytes", CTLFLAG_RD, &tx_stats[txr].tx_mcast_bytes,
	    "multicast bytes sent");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "bcast_bytes", CTLFLAG_RD, &tx_stats[txr].tx_bcast_bytes,
	    "broadcast bytes sent");

	return 0;
}

int
bnxt_create_rx_sysctls(struct bnxt_softc *softc, int rxr)
{
	struct sysctl_oid *oid;
	struct ctx_hw_stats *rx_stats = (void *)softc->rx_stats.idi_vaddr;
	char	name[32];
	char	desc[64];

	sprintf(name, "rxq%d", rxr);
	sprintf(desc, "receive queue %d", rxr);
	oid = SYSCTL_ADD_NODE(&softc->hw_stats,
	    SYSCTL_CHILDREN(softc->hw_stats_oid), OID_AUTO, name, CTLFLAG_RD, 0,
	    desc);
	if (!oid)
		return ENOMEM;

	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "ucast_pkts", CTLFLAG_RD, &rx_stats[rxr].rx_ucast_pkts,
	    "unicast packets received");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "mcast_pkts", CTLFLAG_RD, &rx_stats[rxr].rx_mcast_pkts,
	    "multicast packets received");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "bcast_pkts", CTLFLAG_RD, &rx_stats[rxr].rx_bcast_pkts,
	    "broadcast packets received");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "discard_pkts", CTLFLAG_RD,
	    &rx_stats[rxr].rx_discard_pkts, "discarded receive packets");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "drop_pkts", CTLFLAG_RD, &rx_stats[rxr].rx_drop_pkts,
	    "dropped receive packets");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "ucast_bytes", CTLFLAG_RD, &rx_stats[rxr].rx_ucast_bytes,
	    "unicast bytes received");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "mcast_bytes", CTLFLAG_RD, &rx_stats[rxr].rx_mcast_bytes,
	    "multicast bytes received");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "bcast_bytes", CTLFLAG_RD, &rx_stats[rxr].rx_bcast_bytes,
	    "broadcast bytes received");

	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tpa_pkts", CTLFLAG_RD, &rx_stats[rxr].tpa_pkts,
	    "TPA packets");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tpa_bytes", CTLFLAG_RD, &rx_stats[rxr].tpa_bytes,
	    "TPA bytes");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tpa_events", CTLFLAG_RD, &rx_stats[rxr].tpa_events,
	    "TPA events");
	SYSCTL_ADD_QUAD(&softc->hw_stats, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tpa_aborts", CTLFLAG_RD, &rx_stats[rxr].tpa_aborts,
	    "TPA aborts");

	return 0;
}

static char *bnxt_chip_type[] = {
	"ASIC",
	"FPGA",
	"Palladium",
	"Unknown"
};
#define MAX_CHIP_TYPE 3

void *bnxt_hwrm_nvm_read(struct bnxt_softc *softc, uint16_t index,
    uint32_t offset, uint32_t length);

static int
bnxt_package_ver_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct bnxt_softc *softc = arg1;
	char *pkglog = NULL;
	char *p;
	char *next;
	char unk[] = "<unknown>";
	char *buf = unk;
	int rc;
	int field;
	uint16_t ordinal = BNX_DIR_ORDINAL_FIRST;
	uint16_t index;
	uint32_t data_len;

	rc = bnxt_hwrm_nvm_find_dir_entry(softc, BNX_DIR_TYPE_PKG_LOG,
	    &ordinal, BNX_DIR_EXT_NONE, &index, false,
	    BNXT_DIR_SEARCH_OPT_EQUAL, &data_len, NULL, NULL);
	if (rc == 0 && data_len) {
		pkglog = bnxt_hwrm_nvm_read(softc, index, 0, data_len);
		if (pkglog) {
			/* NULL terminate (removes last \n) */
			pkglog[data_len-1] = 0;

			/* Set p = start of last line */
			p = strrchr(pkglog, '\n');
			if (p == NULL)
				p = pkglog;

			/* Now find the correct tab delimited field */
			for (field = 0, next = p, p = strsep(&next, "\t");
			    field < BNX_PKG_LOG_FIELD_IDX_PKG_VERSION && p;
			    p = strsep(&next, "\t")) {
				field++;
			}
			if (field == BNX_PKG_LOG_FIELD_IDX_PKG_VERSION)
				buf = p;
		}
	}

	rc = sysctl_handle_string(oidp, buf, 0, req);
	if (pkglog)
		free(pkglog, M_DEVBUF);
	return rc;
}

int
bnxt_create_ver_sysctls(struct bnxt_softc *softc)
{
	struct bnxt_ver_info *vi = softc->ver_info;
	struct sysctl_oid *oid = vi->ver_oid;

	if (!oid)
		return ENOMEM;

	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "hwrm_if", CTLFLAG_RD, vi->hwrm_if_ver, 0,
	    "HWRM interface version");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "hwrm_fw", CTLFLAG_RD, vi->hwrm_fw_ver, 0,
	    "HWRM firmware version");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "mgmt_fw", CTLFLAG_RD, vi->mgmt_fw_ver, 0,
	    "management firmware version");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "netctrl_fw", CTLFLAG_RD, vi->netctrl_fw_ver, 0,
	    "network control firmware version");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "roce_fw", CTLFLAG_RD, vi->roce_fw_ver, 0,
	    "RoCE firmware version");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "phy", CTLFLAG_RD, vi->phy_ver, 0,
	    "PHY version");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "hwrm_fw_name", CTLFLAG_RD, vi->hwrm_fw_name, 0,
	    "HWRM firmware name");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "mgmt_fw_name", CTLFLAG_RD, vi->mgmt_fw_name, 0,
	    "management firmware name");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "netctrl_fw_name", CTLFLAG_RD, vi->netctrl_fw_name, 0,
	    "network control firmware name");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "roce_fw_name", CTLFLAG_RD, vi->roce_fw_name, 0,
	    "RoCE firmware name");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "phy_vendor", CTLFLAG_RD, vi->phy_vendor, 0,
	    "PHY vendor name");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "phy_partnumber", CTLFLAG_RD, vi->phy_partnumber, 0,
	    "PHY vendor part number");
	SYSCTL_ADD_U16(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "chip_num", CTLFLAG_RD, &vi->chip_num, 0, "chip number");
	SYSCTL_ADD_U8(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "chip_rev", CTLFLAG_RD, &vi->chip_rev, 0, "chip revision");
	SYSCTL_ADD_U8(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "chip_metal", CTLFLAG_RD, &vi->chip_metal, 0, "chip metal number");
	SYSCTL_ADD_U8(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "chip_bond_id", CTLFLAG_RD, &vi->chip_bond_id, 0,
	    "chip bond id");
	SYSCTL_ADD_STRING(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "chip_type", CTLFLAG_RD, vi->chip_type > MAX_CHIP_TYPE ?
	    bnxt_chip_type[MAX_CHIP_TYPE] : bnxt_chip_type[vi->chip_type], 0,
	    "RoCE firmware name");
	SYSCTL_ADD_PROC(&vi->ver_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "package_ver", CTLTYPE_STRING|CTLFLAG_RD, softc, 0,
	    bnxt_package_ver_sysctl, "A",
	    "currently installed package version");

	return 0;
}

static int
bnxt_rss_key_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct bnxt_softc *softc = arg1;
	char buf[HW_HASH_KEY_SIZE*2+1] = {0};
	char *p;
	int i;
	int rc;

	for (p = buf, i=0; i<HW_HASH_KEY_SIZE; i++)
		p += sprintf(p, "%02x", softc->vnic_info.rss_hash_key[i]);

	rc = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (rc || req->newptr == NULL)
		return rc;

	if (strspn(buf, "0123456789abcdefABCDEF") != (HW_HASH_KEY_SIZE * 2))
		return EINVAL;

	for (p = buf, i=0; i<HW_HASH_KEY_SIZE; i++) {
		if (sscanf(p, "%02hhx", &softc->vnic_info.rss_hash_key[i]) != 1)
			return EINVAL;
		p += 2;
	}

	if (if_getdrvflags(iflib_get_ifp(softc->ctx)) & IFF_DRV_RUNNING)
		bnxt_hwrm_rss_cfg(softc, &softc->vnic_info,
		    softc->vnic_info.rss_hash_type);

	return rc;
}

static const char *bnxt_hash_types[] = {"ipv4", "tcp_ipv4", "udp_ipv4", "ipv6",
    "tcp_ipv6", "udp_ipv6", NULL};

static int bnxt_get_rss_type_str_bit(char *str)
{
	int i;

	for (i=0; bnxt_hash_types[i]; i++)
		if (strcmp(bnxt_hash_types[i], str) == 0)
			return i;

	return -1;
}

static int
bnxt_rss_type_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct bnxt_softc *softc = arg1;
	char buf[256] = {0};
	char *p;
	char *next;
	int rc;
	int type;
	int bit;

	for (type = softc->vnic_info.rss_hash_type; type;
	    type &= ~(1<<bit)) {
		bit = ffs(type) - 1;
		if (bit >= sizeof(bnxt_hash_types) / sizeof(const char *))
			continue;
		if (type != softc->vnic_info.rss_hash_type)
			strcat(buf, ",");
		strcat(buf, bnxt_hash_types[bit]);
	}

	rc = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (rc || req->newptr == NULL)
		return rc;

	for (type = 0, next = buf, p = strsep(&next, " ,"); p;
	    p = strsep(&next, " ,")) {
		bit = bnxt_get_rss_type_str_bit(p);
		if (bit == -1)
			return EINVAL;
		type |= 1<<bit;
	}
	if (type != softc->vnic_info.rss_hash_type) {
		softc->vnic_info.rss_hash_type = type;
		if (if_getdrvflags(iflib_get_ifp(softc->ctx)) & IFF_DRV_RUNNING)
			bnxt_hwrm_rss_cfg(softc, &softc->vnic_info,
			    softc->vnic_info.rss_hash_type);
	}

	return rc;
}

static int
bnxt_rx_stall_sysctl(SYSCTL_HANDLER_ARGS) {
	struct bnxt_softc *softc = arg1;
	int rc;
	int val;

	if (softc == NULL)
		return EBUSY;

	val = (bool)(softc->vnic_info.flags & BNXT_VNIC_FLAG_BD_STALL);
	rc = sysctl_handle_int(oidp, &val, 0, req);
	if (rc || !req->newptr)
		return rc;

	if (val)
		softc->vnic_info.flags |= BNXT_VNIC_FLAG_BD_STALL;
	else
		softc->vnic_info.flags &= ~BNXT_VNIC_FLAG_BD_STALL;

	if (if_getdrvflags(iflib_get_ifp(softc->ctx)) & IFF_DRV_RUNNING)
		rc = bnxt_hwrm_vnic_cfg(softc, &softc->vnic_info);

	return rc;
}

static int
bnxt_vlan_strip_sysctl(SYSCTL_HANDLER_ARGS) {
	struct bnxt_softc *softc = arg1;
	int rc;
	int val;

	if (softc == NULL)
		return EBUSY;

	val = (bool)(softc->vnic_info.flags & BNXT_VNIC_FLAG_VLAN_STRIP);
	rc = sysctl_handle_int(oidp, &val, 0, req);
	if (rc || !req->newptr)
		return rc;

	if (val)
		softc->vnic_info.flags |= BNXT_VNIC_FLAG_VLAN_STRIP;
	else
		softc->vnic_info.flags &= ~BNXT_VNIC_FLAG_VLAN_STRIP;

	if (if_getdrvflags(iflib_get_ifp(softc->ctx)) & IFF_DRV_RUNNING)
		rc = bnxt_hwrm_vnic_cfg(softc, &softc->vnic_info);

	return rc;
}

int
bnxt_create_config_sysctls_pre(struct bnxt_softc *softc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(softc->dev);
	struct sysctl_oid_list *children;

	children = SYSCTL_CHILDREN(device_get_sysctl_tree(softc->dev));;

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "rss_key",
	    CTLTYPE_STRING|CTLFLAG_RWTUN, softc, 0, bnxt_rss_key_sysctl, "A",
	    "RSS key");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "rss_type",
	    CTLTYPE_STRING|CTLFLAG_RWTUN, softc, 0, bnxt_rss_type_sysctl, "A",
	    "RSS key");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "rx_stall",
	    CTLTYPE_INT|CTLFLAG_RWTUN, softc, 0, bnxt_rx_stall_sysctl, "I",
	    "buffer rx packets in hardware until the host posts new buffers");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "vlan_strip",
	    CTLTYPE_INT|CTLFLAG_RWTUN, softc, 0, bnxt_vlan_strip_sysctl, "I",
	    "strip VLAN tag in the RX path");

	return 0;
}

static int
bnxt_vlan_only_sysctl(SYSCTL_HANDLER_ARGS) {
	struct bnxt_softc *softc = arg1;
	int rc;
	int val;

	if (softc == NULL)
		return EBUSY;

	val = softc->vnic_info.vlan_only;
	rc = sysctl_handle_int(oidp, &val, 0, req);
	if (rc || !req->newptr)
		return rc;

	if (val)
		val = 1;

	if (val != softc->vnic_info.vlan_only) {
		softc->vnic_info.vlan_only = val;
		if (if_getdrvflags(iflib_get_ifp(softc->ctx)) & IFF_DRV_RUNNING)
			rc = bnxt_hwrm_cfa_l2_set_rx_mask(softc,
			    &softc->vnic_info);
	}

	return rc;
}

int
bnxt_create_config_sysctls_post(struct bnxt_softc *softc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(softc->dev);
	struct sysctl_oid_list *children;

	children = SYSCTL_CHILDREN(device_get_sysctl_tree(softc->dev));;

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "vlan_only",
	    CTLTYPE_INT|CTLFLAG_RWTUN, softc, 0, bnxt_vlan_only_sysctl, "I",
	    "require vlan tag on received packets when vlan is enabled");

	return 0;
}
