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

#ifndef _BNXT_HWRM_H
#define _BNXT_HWRM_H

/* HWRM Function Prototypes */
int bnxt_alloc_hwrm_dma_mem(struct bnxt_softc *);
void bnxt_free_hwrm_dma_mem(struct bnxt_softc *);
int bnxt_hwrm_ring_alloc(struct bnxt_softc *softc, uint8_t type,
    struct bnxt_ring *ring, uint16_t cmpl_ring_id, uint32_t stat_ctx_id,
    bool irq);
int bnxt_hwrm_ring_free(struct bnxt_softc *softc, uint8_t type,
    struct bnxt_ring *ring);
void bnxt_hwrm_cmd_hdr_init(struct bnxt_softc *, void *, uint16_t, uint16_t,
    uint16_t);
int _hwrm_send_message(struct bnxt_softc *, void *, uint32_t);
int hwrm_send_message(struct bnxt_softc *, void *, uint32_t);
int bnxt_hwrm_ver_get(struct bnxt_softc *);
int bnxt_hwrm_queue_qportcfg(struct bnxt_softc *);
int bnxt_hwrm_func_drv_rgtr(struct bnxt_softc *);
int bnxt_hwrm_func_drv_unrgtr(struct bnxt_softc *, bool);
int bnxt_hwrm_func_qcaps(struct bnxt_softc *);
int bnxt_hwrm_func_reset(struct bnxt_softc *);
int bnxt_hwrm_set_link_setting(struct bnxt_softc *, bool, bool);
int bnxt_hwrm_set_pause(struct bnxt_softc *);
int bnxt_hwrm_vnic_ctx_alloc(struct bnxt_softc *, struct bnxt_vnic_info *);
int bnxt_hwrm_vnic_ctx_free(struct bnxt_softc *, struct bnxt_vnic_info *);
int bnxt_hwrm_vnic_cfg(struct bnxt_softc *, struct bnxt_vnic_info *);
int bnxt_hwrm_stat_ctx_alloc(struct bnxt_softc *softc, struct bnxt_cp_ring *cpr,
    uint64_t paddr);
int bnxt_hwrm_stat_ctx_free(struct bnxt_softc *softc, struct bnxt_cp_ring *cpr);
int bnxt_hwrm_ring_grp_alloc(struct bnxt_softc *, struct bnxt_grp_info *grp);
int bnxt_hwrm_ring_grp_free(struct bnxt_softc *softc,
    struct bnxt_grp_info *grp);
int bnxt_hwrm_vnic_alloc(struct bnxt_softc *, struct bnxt_vnic_info *);
int bnxt_hwrm_vnic_free(struct bnxt_softc *, struct bnxt_vnic_info *);
int bnxt_hwrm_set_vnic_filter(struct bnxt_softc *, uint16_t, uint16_t,
    uint8_t *);
int bnxt_hwrm_set_coal(struct bnxt_softc *);
int bnxt_hwrm_port_qstats(struct bnxt_softc *);
int bnxt_hwrm_cfa_l2_set_rx_mask(struct bnxt_softc *softc,
    struct bnxt_vnic_info *vnic);
int bnxt_hwrm_set_filter(struct bnxt_softc *, struct bnxt_vnic_info *);
int bnxt_hwrm_clear_filter(struct bnxt_softc *, struct bnxt_vnic_info *);

#endif
