import licant

licant.module("ralgo.include",
	include_paths=["."])

licant.module("ralgo.fault", "exception",
	sources=["ralgo/fault-exception.cpp"], 
	default=True)

licant.module("ralgo.fault", "abort",
	sources=["ralgo/fault-abort.cpp"], 
	default=True)

licant.module("ralgo",
	sources=[
	#	"ralgo/util/testheaders.cpp"

		"ralgo/madgwick.cpp",
		"ralgo/signal/fft.cpp"

	],
	include_paths=["."],
	mdepends=[
		#"linalg", 
		#"nos", 
		#"igris.dprint", 
		#"malgo", 
		#("gxx.dprint", "__none__")

		"ralgo.fault"
	],
)

licant.module("ralgo.virtdevs", 
	mdepends=["ralgo.include"],
	sources=["ralgo/virtdevs/device.cpp"]
)

licant.module("ralgo.disctime", "chrono-millis",
	sources = ["ralgo/disctime-chrono-millis.cpp"])