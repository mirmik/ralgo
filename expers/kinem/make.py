#!/usr/bin/env python3

import licant

licant.include("linalg")
licant.include("ralgo")

licant.cxx_application("target",
	sources = ["main.cpp"],
	mdepends = ["ralgo"],
	libs = ["nos", "igris"]
)

licant.ex("target")
