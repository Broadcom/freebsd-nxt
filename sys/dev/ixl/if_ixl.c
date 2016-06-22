/* $FreeBSD$ */
#ifndef IFLIB
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include "iflib_if_ixl.c"
#else
#include "legacy_if_ixl.c"
#endif
