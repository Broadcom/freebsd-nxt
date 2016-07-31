#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#ifndef IXL_STANDALONE_BUILD
#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"
#endif

#include "ixl.h"
#include "ixl_pf.h"

#ifdef RSS
#include <net/rss_config.h>
#endif

#include "ifdi_if.h"


/*********************************************************************
 *
 *  Media Ioctl callback
 *
 *  This routine is called when the user changes speed/duplex using
 *  media/mediopt option with ifconfig.
 *
 **********************************************************************/
int
ixl_if_media_change(if_ctx_t ctx)
{
	struct ifmedia *ifm = iflib_get_media(ctx);

	INIT_DEBUGOUT("ixl_media_change: begin");

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	if_printf(iflib_get_ifp(ctx), "Media change is currently not supported.\n");
	return (ENODEV);
}

int
ixl_if_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs, int nqs, int nqsets)
{
	struct ixl_vsi *vsi = iflib_get_softc(ctx);
	struct ixl_tx_queue *que;
	if_shared_ctx_t sctx;
	int i;

	MPASS(vsi->num_tx_queues > 0);
	MPASS(nqs == 1);
	MPASS(vsi->num_tx_queues == nqsets);
	/* Allocate queue structure memory */
	sctx = iflib_get_sctx(ctx);
	if (!(vsi->tx_queues =
	    (struct ixl_tx_queue *) malloc(sizeof(struct ixl_tx_queue) *nqsets, M_IXL, M_NOWAIT | M_ZERO))) {
		device_printf(iflib_get_dev(ctx), "Unable to allocate TX ring memory\n");
		return (ENOMEM);
	}
	
	for (i = 0, que = vsi->tx_queues; i < nqsets; i++, que++) {
		struct tx_ring		*txr = &que->txr;

		txr->me = i;
		que->vsi = vsi;

		/* get the virtual and physical address of the hardware queues */
		txr->tail = I40E_QTX_TAIL(txr->me);
		txr->tx_base = (struct i40e_tx_desc *)vaddrs[i];
		txr->tx_paddr = paddrs[i];
		txr->que = que;
	}

	device_printf(iflib_get_dev(ctx), "allocated for %d queues\n", vsi->num_tx_queues);
	return (0);
}

int
ixl_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs, int nqs, int nqsets)
{
	struct ixl_vsi *vsi = iflib_get_softc(ctx);
	struct ixl_rx_queue *que;
	if_shared_ctx_t sctx;
	int i;

	MPASS(vsi->num_rx_queues > 0);
	MPASS(nqs == 1);
	MPASS(vsi->num_rx_queues == nqsets);
	/* Allocate queue structure memory */
	sctx = iflib_get_sctx(ctx);
	if (!(vsi->rx_queues =
	    (struct ixl_rx_queue *) malloc(sizeof(struct ixl_rx_queue) *
	    nqsets, M_IXL, M_NOWAIT | M_ZERO))) {
		device_printf(iflib_get_dev(ctx), "Unable to allocate TX ring memory\n");
		return (ENOMEM);
	}

	for (i = 0, que = vsi->rx_queues; i < nqsets; i++, que++) {
		struct rx_ring 		*rxr = &que->rxr;

		rxr->me = i;
		que->vsi = vsi;

		/* get the virtual and physical address of the hardware queues */
		rxr->tail = I40E_QRX_TAIL(rxr->me);
		rxr->rx_base = (union i40e_rx_desc *)vaddrs[i];
		rxr->rx_paddr = paddrs[i];
		rxr->que = que;
	}

	device_printf(iflib_get_dev(ctx), "allocated for %d queues\n", vsi->num_rx_queues);
	return (0);
}

void
ixl_if_queues_free(if_ctx_t ctx)
{
	struct ixl_vsi *vsi = iflib_get_softc(ctx);

	if (vsi->tx_queues != NULL) {
		free(vsi->tx_queues, M_IXL);
		vsi->tx_queues = NULL;
	}
	if (vsi->rx_queues != NULL) {
		free(vsi->rx_queues, M_IXL);
		vsi->rx_queues = NULL;
	}
}
#endif /* IFLIB */
