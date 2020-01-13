#!/usr/bin/env python3

import licant

licant.include("linalg")
licant.include("ralgo")
licant.include("malgo")

licant.cxx_application("target",
	sources = ["main.cpp"],
	mdepends = ["ralgo", "malgo", "linalg"],
	libs = ["nos", "igris", "pthread"]
)

licant.ex("target")