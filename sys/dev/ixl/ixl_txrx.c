/* $FreeBSD */
#ifndef IFLIB
#include "opt_iflib.h"
#endif


#ifdef IFLIB
#include "iflib_ixl_txrx.c"
#else
#include "legacy_ixl_txrx.c"
#endif
