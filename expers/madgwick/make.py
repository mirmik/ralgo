#!/usr/bin/env python3

import licant

licant.include("ralgo")
licant.include("malgo")
licant.include("linalg")
licant.include("nos")
licant.include("igris")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=[
		"linalg",
		"ralgo"
	]
)

licant.ex("target")
