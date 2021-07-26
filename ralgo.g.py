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
		"ralgo/log/log.cpp",
		"ralgo/trajectory/tsdeform.c",
		"ralgo/trajectory/trajectory.c",
		"ralgo/trajectory/linetraj.c",
		"ralgo/robo/quadgen4_arduino.c",

		"ralgo/heimer2/axisctr.c",
		"ralgo/heimer2/axisctr_command.c",
		"ralgo/heimer2/stepctr.c",
		"ralgo/heimer2/stepctr_applier.c",
		"ralgo/heimer2/signal_processor.c",
		"ralgo/heimer2/signal.c",
		"ralgo/heimer2/command.c",
		"ralgo/heimer2/axis_state.c",
		"ralgo/heimer2/axstate_linear_processor.c",
		"ralgo/heimer2/axstate_sincos_processor.c",
		"ralgo/heimer2/convex_zone_approval.c",

		"ralgo/clinalg/*.cpp",
		"ralgo/clinalg/*.c",

		"ralgo/lp/*.c",
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


licant.module("f2clib.internal",
	sources = [
		"ralgo/compat/f2clib/c_*.c",
		"ralgo/compat/f2clib/d_*.c",
		"ralgo/compat/f2clib/r_*.c",
		"ralgo/compat/f2clib/z_*.c",
		"ralgo/compat/f2clib/i_*.c",
		"ralgo/compat/f2clib/s_*.c",
		"ralgo/compat/f2clib/cabs.c",
		"ralgo/compat/f2clib/f77_aloc.c",
		"ralgo/compat/f2clib/pow_ri.c",
		"ralgo/compat/f2clib/pow_di.c",
		"ralgo/compat/f2clib/pow_ii.c",
		"ralgo/compat/f2clib/pow_ci.c",
		"ralgo/compat/f2clib/pow_dd.c",
		"ralgo/compat/f2clib/pow_zi.c",
		"ralgo/compat/f2clib/fmt.c",
		"ralgo/compat/f2clib/sfe.c",
		"ralgo/compat/f2clib/err.c",
		"ralgo/compat/f2clib/open.c",
		"ralgo/compat/f2clib/close.c",
		"ralgo/compat/f2clib/util.c",
		"ralgo/compat/f2clib/wsfe.c",
		"ralgo/compat/f2clib/wrtfmt.c",
		"ralgo/compat/f2clib/wref.c",
		"ralgo/compat/f2clib/fmtlib.c",
		"ralgo/compat/f2clib/ctype.c",
		"ralgo/compat/f2clib/sig_die.c",
		"ralgo/compat/lapack/*.c",
		"ralgo/compat/blas/*.c",
		"ralgo/compat/install/slamch.c",
		"ralgo/compat/install/dlamch.c",
	
	],
	defines = ["IEEE_COMPLEX_DIVIDE=1"],
	include_paths=["ralgo/compat/include"]
)

licant.module("ralgo.disctime", "chrono-millis",
	sources = ["ralgo/disctime-chrono-millis.cpp"], default=True)

licant.module("ralgo.disctime", "manual",
	sources = [])
