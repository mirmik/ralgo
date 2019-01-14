import licant

licant.include("linalg-v3")
licant.include("nos")
licant.include("gxx")

licant.module("ralgo",
	sources=["ralgo/util/testheaders.cpp"],
	include_paths=["."],
	mdepends=[
		"linalg-v3", 
		"nos", 
		"gxx", 
		("gxx.dprint", "__none__")
	],
)