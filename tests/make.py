#!/usr/bin/env python3

import licant
from licant.cxx_modules import application
from licant.modules import submodule, module
from licant.libs import include

licant.execute("../ralgo.g.py")

application("runtests",
            sources=[
                "signal/*.cpp",
                "imu/*",
                "*.cpp",
                "robo/*.cpp",
                "heimer/*.cpp",
                "space/*.cpp",
                "math/*.cpp",
                "lp/*.cpp",
                #		"cnc/*.cpp",
                "filter/*.cpp",
                "physics/*.cpp",
                "rxsignal/*.cpp",
            ],

            cxx_flags="-O0 -g -pedantic -Wno-vla -Werror=extra -Werror=all -Werror=reorder -Weffc++",
            cc_flags="-O0 -g -Werror=incompatible-pointer-types -pedantic -Werror=extra -Werror=all",
            ld_flags="-O0 -L/usr/local/lib/",
            cxxstd="c++20",

            include_paths=["."],
            mdepends=["ralgo", ("ralgo.log", "silent")],
            libs=["igris", "nos", "reactivex"],
            )

licant.ex("runtests")
