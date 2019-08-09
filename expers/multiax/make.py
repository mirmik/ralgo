#!/usr/bin/env python3

import licant

licant.include("genos")
licant.include("ralgo")
licant.include("rabbit")
licant.include("nos")
licant.include("igris")
licant.include("linalg-v3")

licant.glbfunc.genos_firmware(
	sources=["main.cpp"],
	mdepends=["ralgo", "linalg-v3", "rabbit", "nos", "igris"]
)

licant.ex("firmware")