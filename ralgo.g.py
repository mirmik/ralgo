import licant

licant.module("ralgo.include",
	include_paths=["."])


licant.module("ralgo",
	sources=[
	#	"ralgo/util/testheaders.cpp"

		"ralgo/madgwick.cpp",
		"ralgo/signal/fft.cpp"

	],
	include_paths=["."],
	mdepends=[
		#"linalg-v3", 
		#"nos", 
		#"igris.dprint", 
		#"malgo", 
		#("gxx.dprint", "__none__")
	],
)