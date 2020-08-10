import licant

licant.module("ralgo.include",
	include_paths=["."])

licant.module("ralgo.fault", "exception",
	sources=["ralgo/log/fault-exception.cpp"])

licant.module("ralgo.fault", "abort",
	sources=["ralgo/log/fault-abort.cpp"], 
	default=True)

licant.module("ralgo.warn", "dprint",
	sources=["ralgo/log/warn.cpp"], 
	default=True)

licant.module("ralgo.log",
	mdepends=[
	"ralgo.fault", 
	"ralgo.warn"
])

licant.module("ralgo",
	sources=[
		"ralgo/madgwick.cpp",
		"ralgo/signal/fft.cpp",
		#"ralgo/heimer/control.cpp",
		#"ralgo/heimer/command_center.cpp"
	],
	include_paths=["."],
	mdepends=[
		"ralgo.include",
		"ralgo.log",
		"ralgo.disctime",
		"ralgo.heimer",
	],
)

licant.module("ralgo.heimer",
	sources=[
		"ralgo/heimer/control.cpp"
	])

#licant.module("ralgo.virtdevs", 
#	mdepends=["ralgo.include"],
#	sources=["ralgo/virtdevs/device.cpp"]
#)

licant.module("ralgo.disctime", "chrono-millis",
	sources = ["ralgo/disctime-chrono-millis.cpp"], default=True)