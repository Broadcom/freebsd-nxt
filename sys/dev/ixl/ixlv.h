/* $FreeBSD*/
#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include "iflib_ixlv.h"
#else
#include "legacy_ixlv.h"
#endif
