#!/usr/bin/env python3

import licant
from licant.cxx_modules import application
from licant.modules import submodule, module
from licant.libs import include

licant.include("ralgo")
licant.include("linalg")
licant.include("igris")

tests = [
	"virtdevs"
]

tests_c = [
]

application("runtests",
	sources = [
		"main.cpp",
		"heimer.cpp",
		"heimer2.cpp",
		"traj.cpp",
		"geom.cpp",
		"matops.cpp",
		"backpack.cpp",
		"sliding_array.cpp",
		"vecops.cpp"
	],

	ld_flags = "-L/usr/local/lib/",

	include_paths = ["."],
	mdepends = ["ralgo", "linalg"],
	libs = ["igris", "nos"],
)

licant.ex("runtests")