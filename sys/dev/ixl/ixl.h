/* $FreeBSD */
#ifndef KLD_MODULE
#include "opt_iflib.h"
#endif

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/malloc.h>


#ifdef IFLIB
#include "iflib_ixl.h"
#else
#include "legacy_ixl.h"
#endif
