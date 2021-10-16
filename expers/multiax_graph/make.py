#!/usr/bin/env python3

import licant

licant.include("ralgraph")
licant.include("rabbit")
licant.include("ralgo")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=["ralgo", "rabbit", "ralgraph"],
	include_paths=
	[
		"/usr/include/x86_64-linux-gnu/qt5/", 
		"/usr/include/x86_64-linux-gnu/qt5/QtCore",
		"/usr/include/x86_64-linux-gnu/qt5/QtWidgets",
		"/usr/include/x86_64-linux-gnu/qt5/QtCharts"
	],
	cxx_flags="-fPIC",
	libs=["Qt5Core", "Qt5Widgets", "Qt5Charts", "nos", "igris"]
)

licant.ex("target")
