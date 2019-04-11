#!/usr/bin/env python3
#coding:utf-8

import licant

licant.include("igris")
licant.include("nos")
licant.include("linalg-v3")
licant.include("malgo")
licant.include("ralgo", "ralgo.g.py")

licant.cxx_shared_library("ralgo.so",
	mdepends=[
		"ralgo"
	],
	cxx_flags="-fPIC",
	cc_flags="-fPIC"
)

licant.ex("ralgo.so")