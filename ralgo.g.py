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

		"ralgo/heimer/axisctr.cpp",
		"ralgo/heimer/axisctr_command.cpp",
		"ralgo/heimer/stepctr.cpp",
		"ralgo/heimer/stepctr_applier.cpp",
		"ralgo/heimer/signal_processor.cpp",
		"ralgo/heimer/signal.cpp",
		"ralgo/heimer/dof6_signal.cpp",
		"ralgo/heimer/dof6_controller.cpp",
		"ralgo/heimer/command.cpp",
		"ralgo/heimer/scalar_signal.cpp",
		#"ralgo/heimer/axis_state.cpp",
		"ralgo/heimer/axis_stub_processor.cpp",
		"ralgo/heimer/axstate_pid_processor.cpp",
		"ralgo/heimer/axstate_signal_processor.cpp",
		"ralgo/heimer/axstate_linear_processor.cpp",
		"ralgo/heimer/axstate_sincos_processor.cpp",
		"ralgo/heimer/axstate_pose3_chain_processor.cpp",
		"ralgo/heimer/convex_zone_approval.cpp",
		"ralgo/heimer/executor.cpp",

		"ralgo/clinalg/matops_square_inverse.cpp",  
		"ralgo/clinalg/solve.cpp",

		"ralgo/lp/gradient.c",
		"ralgo/lp/point_in_hexagon.c",

		"ralgo/util/index_brute_force.c",
		"ralgo/kinematic/kinchain.cpp",
	],
	include_paths=["."],
	mdepends=[
		"ralgo.include",
		"ralgo.log",
		"ralgo.disctime"
	],
)

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
