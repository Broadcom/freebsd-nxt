#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/ixgbe/iflib_if_ixv.c>
#else
#include <dev/ixgbe/legacy_if_ixv.c>
#endif
