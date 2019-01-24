import licant

licant.include("linalg-v3")
licant.include("malgo")
licant.include("nos")
licant.include("gxx")

licant.module("ralgo",
	sources=[
	#	"ralgo/util/testheaders.cpp"
	],
	include_paths=["."],
	mdepends=[
		"linalg-v3", 
		#"nos", 
		#"gxx", 
		"malgo", 
		#("gxx.dprint", "__none__")
	],
)