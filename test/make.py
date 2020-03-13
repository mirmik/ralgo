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
	sources = (["main.cpp"]),

	ld_flags = "-L/usr/local/lib/",

	include_paths = ["."],
	mdepends = ["ralgo", "igris"],
	#libs = ["gtest", "pthread", "igris", "nos"],
)

licant.ex("runtests")