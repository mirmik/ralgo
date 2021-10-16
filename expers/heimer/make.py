#!/usr/bin/env python3

import licant

licant.include("linalg")
licant.include("ralgo")
licant.include("rabbit")
licant.include("malgo")
licant.include("nos")
licant.include("igris")

licant.cxx_application("target",
	sources = ["main.cpp"],
	mdepends = [
		"ralgo", 
		"ralgo.heimer", 
		"malgo", 
		"linalg", 
		"rabbit", 
		"nos", 
		"igris"],
	libs = ["pthread"],
	cxx_flags="-g"
)

licant.ex("target")
