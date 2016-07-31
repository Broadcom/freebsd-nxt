/*$FreeBSD */
#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include "iflib_if_ixlv.c"
#else
#include "legacy_if_ixlv.c"
#endif
