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
	],

	cxx_flags = "",
	cc_flags = "",
	ld_flags = "-L/usr/local/lib/",

	include_paths = ["."],
	mdepends = ["ralgo"],
	libs = ["igris", "nos"],
)

licant.ex("runtests")