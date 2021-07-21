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
		"ralgo/trajectory/tsdeform.c",
		"ralgo/trajectory/trajectory.c",
		"ralgo/trajectory/linetraj.c",
		"ralgo/heimer2/axisctr.c",
		"ralgo/heimer2/axisctr_command.c",
		"ralgo/heimer2/stepctr.c",
		"ralgo/heimer2/stepctr_applier.c",
		"ralgo/heimer2/signal_processor.c",
		"ralgo/heimer2/signal.c",
		"ralgo/heimer2/command.c",
		"ralgo/heimer2/axis_state.c",
		"ralgo/heimer2/axstate_linear_processor.c",
		"ralgo/robo/quadgen4_arduino.c",
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
