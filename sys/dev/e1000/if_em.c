#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/e1000/iflib_if_em.c>
#else
#include <dev/e1000/legacy_if_em.c>
#endif
