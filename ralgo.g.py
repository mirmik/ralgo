import licant

licant.module("ralgo.include",
	include_paths=["."])

licant.module("ralgo.log", "console",
	sources=["ralgo/log/log-console.cpp"],
	default=True
)

licant.module("ralgo",
	sources=[
		"ralgo/imu/madgwick.cpp",
		#"ralgo/signal/fft.cpp",
		"ralgo/log/log.cpp",
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
		"ralgo/heimer/protect.cpp",
		"ralgo/heimer/control.cpp",
		"ralgo/heimer/node.cpp",
		"ralgo/heimer/command_center.cpp",
		"ralgo/heimer/command_center_2.cpp",
		"ralgo/heimer/servo_wishfeed_node.cpp",
		"ralgo/heimer/signal.cpp",
		"ralgo/heimer/command_center_console.cpp"
	])

#licant.module("ralgo.virtdevs", 
#	mdepends=["ralgo.include"],
#	sources=["ralgo/virtdevs/device.cpp"]
#)

licant.module("ralgo.disctime", "chrono-millis",
	sources = ["ralgo/disctime-chrono-millis.cpp"], default=True)

licant.module("ralgo.disctime", "manual",
	sources = [])
