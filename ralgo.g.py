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
		"ralgo/madgwick.cpp",
		"ralgo/signal/fft.cpp"

	],
	include_paths=["."],
	mdepends=[
		"ralgo.fault",
		"ralgo.disctime"
	],
)

#licant.module("ralgo.virtdevs", 
#	mdepends=["ralgo.include"],
#	sources=["ralgo/virtdevs/device.cpp"]
#)

licant.module("ralgo.disctime", "chrono-millis",
	sources = ["ralgo/disctime-chrono-millis.cpp"], default=True)