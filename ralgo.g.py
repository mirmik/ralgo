import licant

licant.module("ralgo",
	sources=[
	#	"ralgo/util/testheaders.cpp"

		"ralgo/madgwick.cpp"

	],
	include_paths=["."],
	mdepends=[
		"linalg-v3", 
		#"nos", 
		"igris.dprint", 
		#"malgo", 
		#("gxx.dprint", "__none__")
	],
)