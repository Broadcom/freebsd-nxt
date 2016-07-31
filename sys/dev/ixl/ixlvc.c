/* $FreeBSD */
#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include "iflib_ixlvc.c"
#else
#include "legacy_ixlvc.c"
#endif
