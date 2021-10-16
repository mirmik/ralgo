#!/usr/bin/env python3

import licant

licant.module("f2clib",
	sources = [
		"F2CLIBS/libf2c/c_*.c",
		"F2CLIBS/libf2c/d_*.c",
		"F2CLIBS/libf2c/r_*.c",
		"F2CLIBS/libf2c/z_*.c",
		"F2CLIBS/libf2c/i_*.c",
		"F2CLIBS/libf2c/s_*.c",
		"F2CLIBS/libf2c/cabs.c",
		"F2CLIBS/libf2c/f77_aloc.c",
		"F2CLIBS/libf2c/pow_ri.c",
		"F2CLIBS/libf2c/pow_di.c",
		"F2CLIBS/libf2c/pow_ii.c",
		"F2CLIBS/libf2c/pow_ci.c",
		"F2CLIBS/libf2c/pow_dd.c",
		"F2CLIBS/libf2c/pow_zi.c",
		"F2CLIBS/libf2c/fmt.c",
		"F2CLIBS/libf2c/sfe.c",
		"F2CLIBS/libf2c/err.c",
		"F2CLIBS/libf2c/open.c",
		"F2CLIBS/libf2c/close.c",
		"F2CLIBS/libf2c/util.c",
		"F2CLIBS/libf2c/wsfe.c",
		"F2CLIBS/libf2c/wrtfmt.c",
		"F2CLIBS/libf2c/wref.c",
		"F2CLIBS/libf2c/fmtlib.c",
		"F2CLIBS/libf2c/ctype.c",
		"F2CLIBS/libf2c/sig_die.c"
	]
)
