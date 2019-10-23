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
	sources = (
		["main.cpp"] 
		+ [t+".cpp" for t in tests] 
		+ [t+".c" for t in tests_c]),

	ld_flags = "-L/usr/local/lib/",

	include_paths = ["."],
	mdepends = ["ralgo", "linalg", "igris"],

	libs = ["gtest", "pthread"]
)

licant.ex("runtests")