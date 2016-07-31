/*-
 * Copyright (c) 2016, Matthew Macy <mmacy@nextbsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Neither the name of Matthew Macy nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include "ixgbe.h"
/********************************************************************
 *
 * Dump Registers
 *
 ********************************************************************/
#define IXGBE_REGS_LEN  1139
extern if_shared_ctx_t ixgbe_sctx;

int ixgbe_get_regs(SYSCTL_HANDLER_ARGS)
{
        struct adapter *adapter = (struct adapter *) arg1;
	struct ixgbe_hw *hw = &adapter->hw;
#if 0
	struct ix_tx_queue *tx_que = &adapter->tx_queues[0];
	struct ix_rx_queue *rx_que = &adapter->rx_queues[0];
	struct rx_ring *rxr = &rx_que->rxr;
	struct tx_ring *txr = &tx_que->txr;
	int ntxd = ixgbe_sctx->isc_ntxd[0];
	int nrxd = ixgbe_sctx->isc_nrxd[0];
	int j;
#endif

	struct sbuf *sb;
	u32 *regs_buff = (u32 *)malloc(sizeof(u32) * IXGBE_REGS_LEN, M_DEVBUF, M_NOWAIT);
	int i;
	int rc;

	memset(regs_buff, 0, IXGBE_REGS_LEN * sizeof(u32));

	rc = sysctl_wire_old_buffer(req, 0);
	MPASS(rc == 0);
	if (rc != 0)
	  return (rc);

	sb = sbuf_new_for_sysctl(NULL, NULL, 32*PAGE_SIZE, req);
	MPASS(sb != NULL);
	if (sb == NULL)
	  return (ENOMEM);

		/* General Registers */
	sbuf_printf(sb, "General Registers\n");
	regs_buff[0] = IXGBE_READ_REG(hw, IXGBE_CTRL);
	sbuf_printf(sb, "\tIXGBE_CTRL\t %08x\n", regs_buff[0]);
	regs_buff[1] = IXGBE_READ_REG(hw, IXGBE_STATUS);
	sbuf_printf(sb, "\tIXGBE_STATUS\t %08x\n", regs_buff[1]);
	regs_buff[2] = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	sbuf_printf(sb, "\tIXGBE_CTRL_EXT\t %08x\n", regs_buff[2]);
	regs_buff[3] = IXGBE_READ_REG(hw, IXGBE_ESDP);
	sbuf_printf(sb, "\tIXGBE_ESDP\t %08x\n", regs_buff[3]);
	regs_buff[4] = IXGBE_READ_REG(hw, IXGBE_EODSDP);
	sbuf_printf(sb, "\tIXGBE_EODSDP\t %08x\n", regs_buff[4]);
	regs_buff[5] = IXGBE_READ_REG(hw, IXGBE_LEDCTL);
	sbuf_printf(sb, "\tIXGBE_LEDCTL\t %08x\n", regs_buff[5]);
	regs_buff[6] = IXGBE_READ_REG(hw, IXGBE_FRTIMER);
	sbuf_printf(sb, "\tIXGBE_FRTIMER\t %08x\n", regs_buff[6]);
	regs_buff[7] = IXGBE_READ_REG(hw, IXGBE_TCPTIMER);
	sbuf_printf(sb, "\tIXGBE_TCPTIMER\t %08x\n\n", regs_buff[7]);

	/* Interrupt */
	/* don't read EICR because it can clear interrupt causes, instead
	 * read EICS which is a shadow but doesn't clear EICR */
	sbuf_printf(sb, "Interrupt\n");
	regs_buff[18] = IXGBE_READ_REG(hw, IXGBE_EICS);
	sbuf_printf(sb, "\tIXGBE_EICS\t %08x\n", regs_buff[18]);
	regs_buff[19] = IXGBE_READ_REG(hw, IXGBE_EICS);
	sbuf_printf(sb, "\tIXGBE_EICS\t %08x\n", regs_buff[19]);
	regs_buff[20] = IXGBE_READ_REG(hw, IXGBE_EIMS);
	sbuf_printf(sb, "\tIXGBE_EIMS\t %08x\n", regs_buff[20]);
	regs_buff[21] = IXGBE_READ_REG(hw, IXGBE_EIMC);
	sbuf_printf(sb, "\tIXGBE_EIMC\t %08x\n", regs_buff[21]);
	regs_buff[22] = IXGBE_READ_REG(hw, IXGBE_EIAC);
	sbuf_printf(sb, "\tIXGBE_EIAC\t %08x\n", regs_buff[22]);
	regs_buff[23] = IXGBE_READ_REG(hw, IXGBE_EIAM);
	sbuf_printf(sb, "\tIXGBE_EIAM\t %08x\n", regs_buff[23]);
	regs_buff[24] = IXGBE_READ_REG(hw, IXGBE_EITR(0));
	sbuf_printf(sb, "\tIXGBE_EITR(0)\t %08x\n", regs_buff[24]);
	regs_buff[25] = IXGBE_READ_REG(hw, IXGBE_IVAR(0));
	sbuf_printf(sb, "\tIXGBE_IVAR(0)\t %08x\n", regs_buff[25]);
	regs_buff[26] = IXGBE_READ_REG(hw, IXGBE_MSIXT);
	sbuf_printf(sb, "\tIXGBE_MSIXT\t %08x\n", regs_buff[26]);
	regs_buff[27] = IXGBE_READ_REG(hw, IXGBE_MSIXPBA);
	sbuf_printf(sb, "\tIXGBE_MSIXPBA\t %08x\n", regs_buff[27]);
	regs_buff[28] = IXGBE_READ_REG(hw, IXGBE_PBACL(0));
	sbuf_printf(sb, "\tIXGBE_PBACL(0)\t %08x\n", regs_buff[28]);
	regs_buff[29] = IXGBE_READ_REG(hw, IXGBE_GPIE);
	sbuf_printf(sb, "\tIXGBE_GPIE\t %08x\n", regs_buff[29]);

	/* Flow Control */
	sbuf_printf(sb, "\nFlow Control\n");
	regs_buff[30] = IXGBE_READ_REG(hw, IXGBE_PFCTOP);
	sbuf_printf(sb, "\tIXGBE_PFCTOP\t %08x\n", regs_buff[30]);
	regs_buff[31] = IXGBE_READ_REG(hw, IXGBE_FCTTV(0));
	sbuf_printf(sb, "\tIXGBE_FCTTV(0)\t %08x\n", regs_buff[31]);
	regs_buff[32] = IXGBE_READ_REG(hw, IXGBE_FCTTV(1));
	sbuf_printf(sb, "\tIXGBE_FCTTV(1)\t %08x\n", regs_buff[32]);
	regs_buff[33] = IXGBE_READ_REG(hw, IXGBE_FCTTV(2));
	sbuf_printf(sb, "\tIXGBE_FCTTV(2)\t %08x\n", regs_buff[33]);
	regs_buff[34] = IXGBE_READ_REG(hw, IXGBE_FCTTV(3));
	sbuf_printf(sb, "\tIXGBE_FCTTV(3)\t %08x\n", regs_buff[34]);

	for (i = 0; i < adapter->num_tx_queues; i++) {
		switch (hw->mac.type) {
		case ixgbe_mac_82598EB:
			regs_buff[35 + i] = IXGBE_READ_REG(hw, IXGBE_FCRTL(i));
			sbuf_printf(sb, "\tIXGBE_FCRTL(%2d)\t %08x\n", i, regs_buff[35+i]);
			regs_buff[43 + i] = IXGBE_READ_REG(hw, IXGBE_FCRTH(i));
			sbuf_printf(sb, "\tIXGBE_FCRTH(%2d)\t %08x\n", i, regs_buff[43+i]);
			break;
		case ixgbe_mac_82599EB:
		case ixgbe_mac_X540:
		case ixgbe_mac_X550:
		case ixgbe_mac_X550EM_x:
			regs_buff[35 + i] = IXGBE_READ_REG(hw, IXGBE_FCRTL_82599(i));
			regs_buff[43 + i] = IXGBE_READ_REG(hw, IXGBE_FCRTH_82599(i));
			break;
		default:
			break;
		}
	}
	regs_buff[51] = IXGBE_READ_REG(hw, IXGBE_FCRTV);
	sbuf_printf(sb, "\tIXGBE_FCRTV\t %08x\n", regs_buff[51]);
	regs_buff[52] = IXGBE_READ_REG(hw, IXGBE_TFCS);
	sbuf_printf(sb, "\tIXGBE_TFCS\t %08x\n", regs_buff[52]);

	/* Receive DMA */
	sbuf_printf(sb, "\nReceive DMA\n");
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[53 + i] = IXGBE_READ_REG(hw, IXGBE_RDBAL(i));
		sbuf_printf(sb, "\tIXGBE_RDBAL(%2d)\t %08x\n", i, regs_buff[53+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[117 + i] = IXGBE_READ_REG(hw, IXGBE_RDBAH(i));
		sbuf_printf(sb, "\tIXGBE_RDBAH(%2d)\t %08x\n", i, regs_buff[117+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[181 + i] = IXGBE_READ_REG(hw, IXGBE_RDLEN(i));
		sbuf_printf(sb, "\tIXGBE_RDLEN(%2d)\t %08x\n", i, regs_buff[181+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[245 + i] = IXGBE_READ_REG(hw, IXGBE_RDH(i));
		sbuf_printf(sb, "\tIXGBE_RDH(%2d)\t %08x\n", i, regs_buff[245+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[309 + i] = IXGBE_READ_REG(hw, IXGBE_RDT(i));
		sbuf_printf(sb, "\tIXGBE_RDT(%2d)\t %08x\n", i, regs_buff[309+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[373 + i] = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
		sbuf_printf(sb, "\tIXGBE_RXDCTL(%2d)\t %08x\n", i, regs_buff[373+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[437 + i] = IXGBE_READ_REG(hw, IXGBE_SRRCTL(i));
		sbuf_printf(sb, "\tIXGBE_SRRCTL(%2d)\t %08x\n", i, regs_buff[437+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[453 + i] = IXGBE_READ_REG(hw, IXGBE_DCA_RXCTRL(i));
		sbuf_printf(sb, "\tIXGBE_DCA_RXCTRL(%2d)\t %08x\n", i, regs_buff[453+i]);
	}
	regs_buff[469] = IXGBE_READ_REG(hw, IXGBE_RDRXCTL);
        sbuf_printf(sb, "\tIXGBE_RDRXCTL\t %08x\n", regs_buff[469]);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[470 + i] = IXGBE_READ_REG(hw, IXGBE_RXPBSIZE(i));
		sbuf_printf(sb, "\tIXGBE_RXPBSIZE(%2d)\t %08x\n", i, regs_buff[470+i]);
	}
	regs_buff[478] = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	sbuf_printf(sb, "\tIXGBE_RXCTRL\t %08x\n", regs_buff[478]);
	regs_buff[479] = IXGBE_READ_REG(hw, IXGBE_DROPEN);
	sbuf_printf(sb, "\tIXGBE_DROPEN\t %08x\n", regs_buff[479]);

	/* Receive */
	sbuf_printf(sb, "\nReceive\n");
	regs_buff[480] = IXGBE_READ_REG(hw, IXGBE_RXCSUM);
	sbuf_printf(sb, "\tIXGBE_RXCSUM\t %08x\n", regs_buff[480]);
	regs_buff[481] = IXGBE_READ_REG(hw, IXGBE_RFCTL);
	sbuf_printf(sb, "\tIXGBE_RFCTL\t %08x\n", regs_buff[481]);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[482 + i] = IXGBE_READ_REG(hw, IXGBE_RAL(i));
		sbuf_printf(sb, "\tIXGBE_RAL(%2d)\t %08x\n", i, regs_buff[482+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[498 + i] = IXGBE_READ_REG(hw, IXGBE_RAH(i));
		sbuf_printf(sb, "\tIXGBE_RAH(%2d)\t %08x\n", i, regs_buff[498+i]);
	}
	regs_buff[514] = IXGBE_READ_REG(hw, IXGBE_PSRTYPE(0));
	sbuf_printf(sb, "\tIXGBE_PSRTYPE\t %08x\n", regs_buff[514]);
	regs_buff[515] = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	sbuf_printf(sb, "\tIXGBE_FCTRL\t %08x\n", regs_buff[515]);
	regs_buff[516] = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
	sbuf_printf(sb, "\tIXGBE_VLNCTRL\t %08x\n", regs_buff[516]);
	regs_buff[517] = IXGBE_READ_REG(hw, IXGBE_MCSTCTRL);
	sbuf_printf(sb, "\tIXGBE_MCSTCTRL\t %08x\n", regs_buff[517]);
	regs_buff[518] = IXGBE_READ_REG(hw, IXGBE_MRQC);
	sbuf_printf(sb, "\tIXGBE_MRQC\t %08x\n", regs_buff[518]);
	regs_buff[519] = IXGBE_READ_REG(hw, IXGBE_VMD_CTL);
	sbuf_printf(sb, "\tIXGBE_VMD_CTL\t %08x\n", regs_buff[519]);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[520 + i] = IXGBE_READ_REG(hw, IXGBE_IMIR(i));
		sbuf_printf(sb, "\tIXGBE_IMIR(%2d)\t %08x\n", i, regs_buff[520+i]);
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[528 + i] = IXGBE_READ_REG(hw, IXGBE_IMIREXT(i));
		sbuf_printf(sb, "\tIXGBE_IMIREXT(%2d)\t %08x\n", i, regs_buff[528+i]);
	}
	regs_buff[536] = IXGBE_READ_REG(hw, IXGBE_IMIRVP);
	sbuf_printf(sb, "\tIXGBE_IMIRVP\t %08x\n", regs_buff[536]);

	/* Transmit */
	sbuf_printf(sb, "\nTransmit\n");
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[537 + i] = IXGBE_READ_REG(hw, IXGBE_TDBAL(i));
		sbuf_printf(sb, "\tIXGBE_TDBAL(%2d)\t %08x\n", i, regs_buff[537+i]);
	}

        for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[569 + i] = IXGBE_READ_REG(hw, IXGBE_TDBAH(i));
		sbuf_printf(sb, "\tIXGBE_TDBAH(%2d)\t %08x\n", i, regs_buff[569+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[601 + i] = IXGBE_READ_REG(hw, IXGBE_TDLEN(i));
		sbuf_printf(sb, "\tIXGBE_TDLEN(%2d)\t %08x\n", i, regs_buff[601+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[633 + i] = IXGBE_READ_REG(hw, IXGBE_TDH(i));
		sbuf_printf(sb, "\tIXGBE_TDH(%2d)\t %08x\n", i, regs_buff[633+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++){
		regs_buff[665 + i] = IXGBE_READ_REG(hw, IXGBE_TDT(i));
		sbuf_printf(sb, "\tIXGBE_TDT(%2d)\t %08x\n", i, regs_buff[665+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[697 + i] = IXGBE_READ_REG(hw, IXGBE_TXDCTL(i));
		sbuf_printf(sb, "\tIXGBE_TXDCTL(%2d)\t %08x\n", i, regs_buff[697+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[729 + i] = IXGBE_READ_REG(hw, IXGBE_TDWBAL(i));
		sbuf_printf(sb, "\tIXGBE_TDWBAL(%2d)\t %08x\n", i, regs_buff[729+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[761 + i] = IXGBE_READ_REG(hw, IXGBE_TDWBAH(i));
		sbuf_printf(sb, "\tIXGBE_TDWBAH(%2d)\t %08x\n", i, regs_buff[761+i]);
	}
	regs_buff[793] = IXGBE_READ_REG(hw, IXGBE_DTXCTL);
	sbuf_printf(sb, "\tIXGBE_DTXCTL\t %08x\n", regs_buff[793]);
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[794 + i] = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL(i));
		sbuf_printf(sb, "\tIXGBE_DCA_TXCTRL(%2d)\t %08x\n", i, regs_buff[794+i]);
	}
	regs_buff[810] = IXGBE_READ_REG(hw, IXGBE_TIPG);
	sbuf_printf(sb, "\tIXGBE_TIPG\t %08x\n", regs_buff[810]);

	for (i = 0; i < adapter->num_tx_queues; i++) {
	  regs_buff[811 + i] = IXGBE_READ_REG(hw, IXGBE_TXPBSIZE(i));
	  sbuf_printf(sb, "\tIXGBE_TXPBSIZE(%2d)\t %08x\n", i, regs_buff[811+i]);
	}
	regs_buff[819] = IXGBE_READ_REG(hw, IXGBE_MNGTXMAP);
        sbuf_printf(sb, "\tIXGBE_MNGTXMAP\t %08x\n", regs_buff[819]);

	/* Wake Up */
	sbuf_printf(sb, "\nWake Up\n");
	regs_buff[820] = IXGBE_READ_REG(hw, IXGBE_WUC);
	sbuf_printf(sb, "\tIXGBE_WUC\t %08x\n", regs_buff[820]);
	regs_buff[821] = IXGBE_READ_REG(hw, IXGBE_WUFC);
	sbuf_printf(sb, "\tIXGBE_WUFC\t %08x\n", regs_buff[821]);
	regs_buff[822] = IXGBE_READ_REG(hw, IXGBE_WUS);
	sbuf_printf(sb, "\tIXGBE_WUS\t %08x\n", regs_buff[822]);
	regs_buff[823] = IXGBE_READ_REG(hw, IXGBE_IPAV);
	sbuf_printf(sb, "\tIXGBE_IPAV\t %08x\n", regs_buff[823]);
	regs_buff[824] = IXGBE_READ_REG(hw, IXGBE_IP4AT);
	sbuf_printf(sb, "\tIXGBE_IP4AT\t %08x\n", regs_buff[824]);
	regs_buff[825] = IXGBE_READ_REG(hw, IXGBE_IP6AT);
	sbuf_printf(sb, "\tIXGBE_IP6AT\t %08x\n", regs_buff[825]);
	regs_buff[826] = IXGBE_READ_REG(hw, IXGBE_WUPL);
	sbuf_printf(sb, "\tIXGBE_WUPL\t %08x\n", regs_buff[826]);
	regs_buff[827] = IXGBE_READ_REG(hw, IXGBE_WUPM);
	sbuf_printf(sb, "\tIXGBE_WUPM\t %08x\n", regs_buff[827]);
	regs_buff[828] = IXGBE_READ_REG(hw, IXGBE_FHFT(0));
	sbuf_printf(sb, "\tIXGBE_FHFT\t %08x\n", regs_buff[828]);

	/* DCB */
	sbuf_printf(sb, "\nDCB\n");
	regs_buff[829] = IXGBE_READ_REG(hw, IXGBE_RMCS);   /* same as FCCFG  */
	sbuf_printf(sb, "\tIXGBE_RMCS\t %08x\n", regs_buff[829]);
	regs_buff[831] = IXGBE_READ_REG(hw, IXGBE_PDPMCS); /* same as RTTPCS */
        sbuf_printf(sb, "\tIXGBE_PDPMCS\t %08x\n", regs_buff[831]);

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		regs_buff[830] = IXGBE_READ_REG(hw, IXGBE_DPMCS);
		  sbuf_printf(sb, "\tIXGBE_DPMCS\t %08x\n", regs_buff[830]);
		regs_buff[832] = IXGBE_READ_REG(hw, IXGBE_RUPPBMR);
		  sbuf_printf(sb, "\tIXGBE_RUPPBMR\t %08x\n", regs_buff[832]);
		  for (i = 0; i < adapter->num_tx_queues; i++) {
			regs_buff[833 + i] = IXGBE_READ_REG(hw, IXGBE_RT2CR(i));
			sbuf_printf(sb, "\tIXGBE_RT2CR(%2d)\t %08x\n", i, regs_buff[833+i]);
		  }
		  for (i = 0; i < adapter->num_tx_queues; i++) {
			regs_buff[841 + i] = IXGBE_READ_REG(hw, IXGBE_RT2SR(i));
			sbuf_printf(sb, "\tIXGBE_RT2SR(%2d)\t %08x\n", i, regs_buff[841+i]);
		  }
		  for (i = 0; i < adapter->num_tx_queues; i++) {
		    regs_buff[849 + i] = IXGBE_READ_REG(hw, IXGBE_TDTQ2TCCR(i));
		    sbuf_printf(sb, "\tIXGBE_TDTQ2TCCR(%2d)\t %08x\n", i, regs_buff[849+i]);
		  }
		  for (i = 0; i < adapter->num_tx_queues; i++) {
			regs_buff[857 + i] = IXGBE_READ_REG(hw, IXGBE_TDTQ2TCSR(i));
			sbuf_printf(sb, "\tIXGBE_TDTQ2TCSR(%2d)\t %08x\n", i, regs_buff[857+i]);
		  }
		break;
	case ixgbe_mac_82599EB:
	case ixgbe_mac_X540:
	case ixgbe_mac_X550:
	case ixgbe_mac_X550EM_x:
		regs_buff[830] = IXGBE_READ_REG(hw, IXGBE_RTTDCS);
		regs_buff[832] = IXGBE_READ_REG(hw, IXGBE_RTRPCS);
		for (i = 0; i < adapter->num_tx_queues; i++)
			regs_buff[833 + i] =
				IXGBE_READ_REG(hw, IXGBE_RTRPT4C(i));
		for (i = 0; i < adapter->num_tx_queues; i++)
			regs_buff[841 + i] =
				IXGBE_READ_REG(hw, IXGBE_RTRPT4S(i));
		for (i = 0; i < adapter->num_tx_queues; i++)
			regs_buff[849 + i] =
				IXGBE_READ_REG(hw, IXGBE_RTTDT2C(i));
		for (i = 0; i < adapter->num_tx_queues; i++)
			regs_buff[857 + i] =
				IXGBE_READ_REG(hw, IXGBE_RTTDT2S(i));
		break;
	default:
		break;
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[865 + i] = IXGBE_READ_REG(hw, IXGBE_TDPT2TCCR(i)); /* same as RTTPT2C */
		sbuf_printf(sb, "\tIXGBE_TDPT2TCCR(%2d)\t %08x\n", i, regs_buff[865+i]);
	}
	for (i = 0; i < adapter->num_tx_queues; i++) {
		regs_buff[873 + i] = IXGBE_READ_REG(hw, IXGBE_TDPT2TCSR(i)); /* same as RTTPT2S */
		sbuf_printf(sb, "\tIXGBE_TDPT2TCSR(%2d)\t %08x\n", i, regs_buff[873+i]);
	}
		/* MAC */
	sbuf_printf(sb, "\nMAC\n");
	regs_buff[1038] = IXGBE_READ_REG(hw, IXGBE_PCS1GCFIG);
	sbuf_printf(sb, "\tIXGBE_PCS1GCFIG\t %08x\n", regs_buff[1038]);
	regs_buff[1039] = IXGBE_READ_REG(hw, IXGBE_PCS1GLCTL);
	sbuf_printf(sb, "\tIXGBE_PCSIG1CTL\t %08x\n", regs_buff[1039]);
	regs_buff[1040] = IXGBE_READ_REG(hw, IXGBE_PCS1GLSTA);
	sbuf_printf(sb, "\tIXGBE_PCSIGLSTA\t %08x\n", regs_buff[1040]);
	regs_buff[1041] = IXGBE_READ_REG(hw, IXGBE_PCS1GDBG0);
	sbuf_printf(sb, "\tIXGBE_PCS1GDBG0\t %08x\n", regs_buff[1041]);
	regs_buff[1042] = IXGBE_READ_REG(hw, IXGBE_PCS1GDBG1);
	sbuf_printf(sb, "\tIXGBE_PCS1GDBG1\t %08x\n", regs_buff[1042]);
	regs_buff[1043] = IXGBE_READ_REG(hw, IXGBE_PCS1GANA);
	sbuf_printf(sb, "\tIXGBE_PCS1GANA\t %08x\n", regs_buff[1043]);
	regs_buff[1044] = IXGBE_READ_REG(hw, IXGBE_PCS1GANLP);
	sbuf_printf(sb, "\tIXGBE_PCS1GANLP\t %08x\n", regs_buff[1044]);
	regs_buff[1045] = IXGBE_READ_REG(hw, IXGBE_PCS1GANNP);
	sbuf_printf(sb, "\tIXGBE_PCS1GANNP\t %08x\n", regs_buff[1045]);
	regs_buff[1046] = IXGBE_READ_REG(hw, IXGBE_PCS1GANLPNP);
	sbuf_printf(sb, "\tIXGBE_PCS1GANLPNP\t %08x\n", regs_buff[1046]);
	regs_buff[1047] = IXGBE_READ_REG(hw, IXGBE_HLREG0);
	sbuf_printf(sb, "\tIXGBE_HILREG0\t %08x\n", regs_buff[1047]);
	regs_buff[1048] = IXGBE_READ_REG(hw, IXGBE_HLREG1);
	sbuf_printf(sb, "\tIXGBE_HILREG1\t %08x\n", regs_buff[1048]);
	regs_buff[1049] = IXGBE_READ_REG(hw, IXGBE_PAP);
	sbuf_printf(sb, "\tIXGBE_PAP\t %08x\n", regs_buff[1049]);
	regs_buff[1050] = IXGBE_READ_REG(hw, IXGBE_MACA);
	sbuf_printf(sb, "\tIXGBE_MACA\t %08x\n", regs_buff[1050]);
	regs_buff[1051] = IXGBE_READ_REG(hw, IXGBE_APAE);
	sbuf_printf(sb, "\tIXGBE_APAE\t %08x\n", regs_buff[1051]);
	regs_buff[1052] = IXGBE_READ_REG(hw, IXGBE_ARD);
	sbuf_printf(sb, "\tIXGBE_ARD\t %08x\n", regs_buff[1052]);
	regs_buff[1053] = IXGBE_READ_REG(hw, IXGBE_AIS);
	sbuf_printf(sb, "\tIXGBE_AIS\t %08x\n", regs_buff[1053]);
	regs_buff[1054] = IXGBE_READ_REG(hw, IXGBE_MSCA);
	sbuf_printf(sb, "\tIXGBE_MSCA\t %08x\n", regs_buff[1054]);
	regs_buff[1055] = IXGBE_READ_REG(hw, IXGBE_MSRWD);
	sbuf_printf(sb, "\tIXGBE_MSRWD\t %08x\n", regs_buff[1055]);
	regs_buff[1056] = IXGBE_READ_REG(hw, IXGBE_MLADD);
	sbuf_printf(sb, "\tIXGBE_MLADD\t %08x\n", regs_buff[1056]);
	regs_buff[1057] = IXGBE_READ_REG(hw, IXGBE_MHADD);
	sbuf_printf(sb, "\tIXGBE_MHADD\t %08x\n", regs_buff[1057]);
	regs_buff[1058] = IXGBE_READ_REG(hw, IXGBE_TREG);
	sbuf_printf(sb, "\tIXGBE_TREG\t %08x\n", regs_buff[1058]);
	regs_buff[1059] = IXGBE_READ_REG(hw, IXGBE_PCSS1);
	sbuf_printf(sb, "\tIXGBE_PCSS1\t %08x\n", regs_buff[1059]);
	regs_buff[1060] = IXGBE_READ_REG(hw, IXGBE_PCSS2);
	sbuf_printf(sb, "\tIXGBE_PCSS2\t %08x\n", regs_buff[1060]);
	regs_buff[1061] = IXGBE_READ_REG(hw, IXGBE_XPCSS);
	sbuf_printf(sb, "\tIXGBE_XPCSS\t %08x\n", regs_buff[1061]);
	regs_buff[1062] = IXGBE_READ_REG(hw, IXGBE_SERDESC);
	sbuf_printf(sb, "\tIXGBE_SERDESC\t %08x\n", regs_buff[1062]);
	regs_buff[1063] = IXGBE_READ_REG(hw, IXGBE_MACS);
	sbuf_printf(sb, "\tIXGBE_MACS\t %08x\n", regs_buff[1063]);
	regs_buff[1064] = IXGBE_READ_REG(hw, IXGBE_AUTOC);
	sbuf_printf(sb, "\tIXGBE_AUTOC\t %08x\n", regs_buff[1064]);
	regs_buff[1065] = IXGBE_READ_REG(hw, IXGBE_LINKS);
	sbuf_printf(sb, "\tIXGBE_LINKS\t %08x\n", regs_buff[1065]);
	regs_buff[1066] = IXGBE_READ_REG(hw, IXGBE_AUTOC2);
	sbuf_printf(sb, "\tIXGBE_AUTOC2\t %08x\n", regs_buff[1066]);
	regs_buff[1067] = IXGBE_READ_REG(hw, IXGBE_AUTOC3);
	sbuf_printf(sb, "\tIXGBE_AUTOC3\t %08x\n", regs_buff[1067]);
	regs_buff[1068] = IXGBE_READ_REG(hw, IXGBE_ANLP1);
	sbuf_printf(sb, "\tIXGBE_ANLP1\t %08x\n", regs_buff[1068]);
	regs_buff[1069] = IXGBE_READ_REG(hw, IXGBE_ANLP2);
	sbuf_printf(sb, "\tIXGBE_ANLP2\t %08x\n", regs_buff[1069]);
	regs_buff[1070] = IXGBE_READ_REG(hw, IXGBE_ATLASCTL);
	sbuf_printf(sb, "\tIXGBE_ATLASCTL\t %08x\n", regs_buff[1070]);

	/* Diagnostic */
        sbuf_printf(sb, "\nDiagnostic\n");
	regs_buff[1071] = IXGBE_READ_REG(hw, IXGBE_RDSTATCTL);
	sbuf_printf(sb, "\tIXGBE_RDSTATCTL\t %08x\n", regs_buff[1071]);
	for (i = 0; i < adapter->num_rx_queues; i++) {
		regs_buff[1072 + i] = IXGBE_READ_REG(hw, IXGBE_RDSTAT(i));
		sbuf_printf(sb, "\tIXGBE_RDSTAT(%2d)\t %08x\n", i, regs_buff[1072+i]);
	}
	regs_buff[1080] = IXGBE_READ_REG(hw, IXGBE_RDHMPN);
	sbuf_printf(sb, "\tIXGBE_RDHMPN\t %08x\n", regs_buff[1080]);
	for (i = 0; i < 4; i++) {
		regs_buff[1081 + i] = IXGBE_READ_REG(hw, IXGBE_RIC_DW(i));
		sbuf_printf(sb, "\tIXGBE_RIC_DW(%2d)\t %08x\n", i, regs_buff[1081+i]);
	}
	regs_buff[1085] = IXGBE_READ_REG(hw, IXGBE_RDPROBE);
	sbuf_printf(sb, "\tIXGBE_RDPROBE\t %08x\n", regs_buff[1085]);
       /* regs_buff[1086] = IXGBE_READ_REG(hw, IXGBE_TDSTATCTL);
	  sbuf_printf(sb, "\tIXGBE_TDSTATCTL\t %08x\n", regs_buff[1086]); */
       /*    for (i = 0; i < adapter->num_queues; i++) {
		regs_buff[1087 + i] = IXGBE_READ_REG(hw, IXGBE_TDSTAT(i));
		sbuf_printf(sb, "\n\tIXGBE_TDSTAT(%2d)\t %08x\n", i, regs_buff[1087+i]);
	     } */
	regs_buff[1095] = IXGBE_READ_REG(hw, IXGBE_TDHMPN);
	sbuf_printf(sb, "\tIXGBE_TDHMPN\t %08x\n", regs_buff[1095]);

	for (i = 0; i < 4; i++) {
		regs_buff[1096 + i] = IXGBE_READ_REG(hw, IXGBE_TIC_DW(i));
		sbuf_printf(sb, "\tIXGBE_TIC_DW(%2d)\t %08x\n", i, regs_buff[1096+i]);
	}
	regs_buff[1100] = IXGBE_READ_REG(hw, IXGBE_TDPROBE);
	sbuf_printf(sb, "\tIXGBE_TDPROBE\t %08x\n", regs_buff[1100]);
	regs_buff[1101] = IXGBE_READ_REG(hw, IXGBE_TXBUFCTRL);
	sbuf_printf(sb, "\tIXGBE_TXBUFCTRL\t %08x\n", regs_buff[1101]);
	regs_buff[1102] = IXGBE_READ_REG(hw, IXGBE_TXBUFDATA0);
	sbuf_printf(sb, "\tIXGBE_TXBUFDATA0\t %08x\n", regs_buff[1102]);
	regs_buff[1103] = IXGBE_READ_REG(hw, IXGBE_TXBUFDATA1);
	sbuf_printf(sb, "\tIXGBE_TXBUFDATA1\t %08x\n", regs_buff[1103]);
	regs_buff[1104] = IXGBE_READ_REG(hw, IXGBE_TXBUFDATA2);
	sbuf_printf(sb, "\tIXGBE_TXBUFDATA2\t %08x\n", regs_buff[1104]);
	regs_buff[1105] = IXGBE_READ_REG(hw, IXGBE_TXBUFDATA3);
	sbuf_printf(sb, "\tIXGBE_TXBUFDATA3\t %08x\n", regs_buff[1105]);
	regs_buff[1106] = IXGBE_READ_REG(hw, IXGBE_RXBUFCTRL);
	sbuf_printf(sb, "\tIXGBE_RXBUFCTRL\t %08x\n", regs_buff[1106]);
	regs_buff[1107] = IXGBE_READ_REG(hw, IXGBE_RXBUFDATA0);
	sbuf_printf(sb, "\tIXGBE_RXBIFDATA0\t %08x\n", regs_buff[1107]);
	regs_buff[1108] = IXGBE_READ_REG(hw, IXGBE_RXBUFDATA1);
	sbuf_printf(sb, "\tIXGBE_RXBUFDATA1\t %08x\n", regs_buff[1108]);
	regs_buff[1109] = IXGBE_READ_REG(hw, IXGBE_RXBUFDATA2);
	sbuf_printf(sb, "\tIXGBE_RXBUFDATA2\t %08x\n", regs_buff[1109]);
	regs_buff[1110] = IXGBE_READ_REG(hw, IXGBE_RXBUFDATA3);
	sbuf_printf(sb, "\tIXGBE_RXBUFDATA3\t %08x\n", regs_buff[1110]);
	for (i = 0; i < adapter->num_rx_queues; i++) {
	  regs_buff[1111 + i] = IXGBE_READ_REG(hw, IXGBE_PCIE_DIAG(i));
	  sbuf_printf(sb, "\tIXGBE_PCIE_DIAG(%2d)\t %08x\n", i, regs_buff[1111+i]);
	}
	regs_buff[1119] = IXGBE_READ_REG(hw, IXGBE_RFVAL);
	sbuf_printf(sb, "\tIXGBE_RFVAL\t %08x\n", regs_buff[1119]);
	regs_buff[1120] = IXGBE_READ_REG(hw, IXGBE_MDFTC1);
	sbuf_printf(sb, "\tIXGBE_MDFTC1\t %08x\n", regs_buff[1120]);
	regs_buff[1121] = IXGBE_READ_REG(hw, IXGBE_MDFTC2);
	sbuf_printf(sb, "\tIXGBE_MDFTC2\t %08x\n", regs_buff[1121]);
	regs_buff[1122] = IXGBE_READ_REG(hw, IXGBE_MDFTFIFO1);
	sbuf_printf(sb, "\tIXGBE_MDFTFIF01\t %08x\n", regs_buff[1122]);
	regs_buff[1123] = IXGBE_READ_REG(hw, IXGBE_MDFTFIFO2);
	sbuf_printf(sb, "\tIXGBE_MDFTFIF02\t %08x\n", regs_buff[1123]);
	regs_buff[1124] = IXGBE_READ_REG(hw, IXGBE_MDFTS);
	sbuf_printf(sb, "\tIXGBE_MDFTS\t %08x\n", regs_buff[1124]);
	regs_buff[1125] = IXGBE_READ_REG(hw, IXGBE_PCIEECCCTL);
	sbuf_printf(sb, "\tIXGBE_PCIEECCCTL\t %08x\n", regs_buff[1125]);
	regs_buff[1126] = IXGBE_READ_REG(hw, IXGBE_PBTXECC);
	sbuf_printf(sb, "\tIXGBE_PBTXECC\t %08x\n", regs_buff[1126]);
	regs_buff[1127] = IXGBE_READ_REG(hw, IXGBE_PBRXECC);
	sbuf_printf(sb, "\tIXGBE_PBRXECC\t %08x\n", regs_buff[1127]);

	/* 82599 X540 specific registers  */
	regs_buff[1128] = IXGBE_READ_REG(hw, IXGBE_MFLCN);

	/* 82599 X540 specific DCB registers  */
	regs_buff[1129] = IXGBE_READ_REG(hw, IXGBE_RTRUP2TC);
	regs_buff[1130] = IXGBE_READ_REG(hw, IXGBE_RTTUP2TC);
	for (i = 0; i < 4; i++) {
		regs_buff[1131 + i] = IXGBE_READ_REG(hw, IXGBE_TXLLQ(i));
		sbuf_printf(sb, "\tIXGBE_TXLLQ(%2d)\t %08x\n", i, regs_buff[1131+i]);
	}
	regs_buff[1135] = IXGBE_READ_REG(hw, IXGBE_RTTBCNRM);
	/* same as RTTQCNRM */
	sbuf_printf(sb, "\tIXGBE_RTTBCNRM\t %08x\n", regs_buff[1135]);
	regs_buff[1136] = IXGBE_READ_REG(hw, IXGBE_RTTBCNRD);
	/* same as RTTQCNRR */
	sbuf_printf(sb, "\tIXGBE_RTTBCNRD\t %08x\n", regs_buff[1136]);
#ifdef PRINT_QSET
	for (j = 0; j < nrxd; j++) {
		u32 staterr = le32toh(rxr->rx_base[j].wb.upper.status_error);
		u32 length =  le32toh(rxr->rx_base[j].wb.upper.length);
		sbuf_printf(sb, "\tReceive Descriptor Address %d: %08lx  Error:%d  Length:%d\n", j, rxr->rx_base[j].read.pkt_addr, staterr, length);
	}

	for (j = 0; j < min(ntxd, 256); j++) {
		struct ixgbe_tx_buf *buf = &txr->tx_buffers[j];
		unsigned int *ptr = (unsigned int *)&txr->tx_base[j].read;

		sbuf_printf(sb, "\tTXD[%03d] [0]: %08x [1]: %08x [2]: %08x [3]: %08x  eop: %d DD=%d\n",
			    j, ptr[0], ptr[1], ptr[2], ptr[3], buf->eop,
			    buf->eop != -1 ? txr->tx_base[buf->eop].wb.status & IXGBE_TXD_STAT_DD : 0);

	}
#endif
	/* X540 specific DCB registers
	regs_buff[1137] = IXGBE_READ_REG(hw, IXGBE_RTTQCNCR);
	regs_buff[1138] = IXGBE_READ_REG(hw, IXGBE_RTTQCNTG); */

	rc = sbuf_finish(sb);
	sbuf_delete(sb);
        return(rc);
}
#endif
