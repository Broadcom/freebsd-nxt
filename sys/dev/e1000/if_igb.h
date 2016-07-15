#ifndef IFLIB
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/e1000/iflib_if_igb.h>
#else
#include <dev/e1000/legacy_if_igb.h>
#endif
