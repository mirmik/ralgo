#!/usr/bin/env python3
#coding:utf-8

import licant

licant.include("igris")
licant.include("nos")
licant.include("linalg")
licant.include("malgo")
licant.include("ralgo")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=[
		"ralgo", 
		"nos", 
		"linalg",
		"igris.dprint"
	]
)

licant.ex("target")