#!/usr/bin/env python3
#coding:utf-8

import licant

licant.include("nos")
licant.include("ralgo")
licant.include("linalg-v3")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=["ralgo", "nos", "linalg-v3"]
)

licant.ex("target")