/* $FreeBSD */
#ifndef IFLIB
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include "iflib_ixl_pf.h"
#else
#include "legacy_ixl_pf.h"
#endif
