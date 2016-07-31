#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/ixgbe/iflib_ixgbe.h>
#else
#include <dev/ixgbe/legacy_ixgbe.h>
#endif
