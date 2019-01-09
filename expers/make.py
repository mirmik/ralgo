#!/usr/bin/env python3
#coding:utf-8

import licant

licant.include("nos")
licant.include("autocontrol")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=["autocontrol", "nos"]
)

licant.ex("target")