#!/usr/bin/env python3
# coding:utf-8

import os
import shutil
import licant
import licant.install

licant.include("ralgo", "ralgo.g.py")

licant.execute("apps/sigtrans/make.py")
licant.execute("apps/cncsim/make.py")

target = "libralgo.so"

install_include_path = '/usr/local/include/ralgo'
install_directory_path = '/usr/lib/'
install_library_path = os.path.join(install_directory_path, target)


licant.cxx_static_and_shared("libs",
                             shared_lib="libralgo.so",
                             static_lib="libralgo.a",
                             mdepends=[
                                 "ralgo"
                             ],
                             cxx_flags="-fPIC -g -pedantic -Werror=extra -Werror=all -Weffc++",
                             cc_flags="-fPIC -Werror=implicit-function-declaration -g -Werror=incompatible-pointer-types -pedantic -Werror=extra -Werror=all",
                             libs=["igris", "nos"]
                             )

licant.install.install_library(
    tgt="install_library",
    uninstall="uninstall",
    libtgt=["libralgo.so", "libralgo.a"],
    hroot="ralgo",
    headers="ralgo")

licant.fileset("apps", targets=[
    "sigtrans",
    "cncsim"
], deps=["libralgo.so"])


@licant.routine(deps=["apps"])
def install_apps():
    licant.do(["install_sigtrans", "makefile"])


licant.cxx_application("runtests",
                       sources=[
                           "tests/signal/*.cpp",
                           "tests/imu/*",
                           "tests/*.cpp",
                           "tests/robo/*.cpp",
                           "tests/heimer/*.cpp",
                           "tests/space/*.cpp",
                           "tests/math/*.cpp",
                           "tests/lp/*.cpp",
                           #		"cnc/*.cpp",
                           "tests/filter/*.cpp",
                           "tests/physics/*.cpp",
                           "tests/rxsignal/*.cpp",
                       ],
                       objects=["libralgo.a"],
                       cxx_flags="-fPIC -O0 -g -pedantic -Wno-vla -Werror=extra -Werror=all -Werror=reorder -Weffc++",
                       cc_flags="-fPIC -O0 -g -Werror=incompatible-pointer-types -pedantic -Werror=extra -Werror=all",
                       ld_flags="-fPIC -O0 -L/usr/local/lib/",
                       cxxstd="c++20",
                       include_paths=[".", "tests"],
                       libs=["igris", "nos"],
                       )

licant.fileset("all", targets=["apps", "libs", "runtests"])
licant.fileset("install", targets=["install_apps", "install_library"])

licant.ex("all")
