#!/usr/bin/env python3

import licant
from licant.cxx_modules import application
from licant.modules import submodule, module
from licant.libs import include

licant.execute("../ralgo.g.py")

application("runtests",
	sources = [
		"imu/*",
		"*.cpp",
		"heimer/*.cpp",
		"math/*.cpp",
		"lp/*.cpp",
	],

	cxx_flags = "-g -pedantic -Wno-vla -Werror=extra -Werror=all",
	cc_flags = "-g -Werror=incompatible-pointer-types -pedantic -Werror=extra -Werror=all",
	ld_flags = "-L/usr/local/lib/",

	include_paths = ["."],
	mdepends = ["ralgo"],
	libs = ["igris", "nos"],
)

licant.ex("runtests")