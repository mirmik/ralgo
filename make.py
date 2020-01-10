#!/usr/bin/env python3
#coding:utf-8

import os
import shutil
import licant
import licant.install

licant.include("igris")
licant.include("nos")
licant.include("linalg")
licant.include("malgo")
licant.include("ralgo", "ralgo.g.py")

target = "libralgo.so"

install_include_path = '/usr/local/include/ralgo' 
install_directory_path = '/usr/lib/'
install_library_path = os.path.join(install_directory_path, target)


licant.cxx_shared_library("libralgo.so",
	mdepends=[
		"ralgo",
		"linalg"
	],
	cxx_flags="-fPIC",
	cc_flags="-fPIC"
)

licant.install.install_library(
	tgt="install",
	uninstall="uninstall",
	libtgt="libralgo.so",
	hroot="ralgo",
	headers="ralgo")

licant.ex("libralgo.so")