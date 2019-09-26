#!/usr/bin/env python3

import licant

licant.include("linalg")
licant.include("ralgo")
licant.include("malgo")
licant.include("rabbit")

licant.cxx_application("target",
	sources = ["main.cpp"],
	mdepends = ["ralgo", "malgo", "rabbit"],
	libs = ["nos", "igris"]
)

licant.ex("target")