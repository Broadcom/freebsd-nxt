#ifndef IFLIB
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/ixgbe/iflib_ix_txrx.c>
#else
#include <dev/ixgbe/legacy_ix_txrx.c>
#endif
