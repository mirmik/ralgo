#!/usr/bin/env python3
#coding:utf-8

import os
import shutil
import licant

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

@licant.routine(deps=["libralgo.so"])
def install():
	os.system("cp {0} {1}".format(target, install_directory_path))
	
	shutil.rmtree(install_include_path, True)
	shutil.copytree("ralgo", install_include_path, 
		symlinks=False, ignore=shutil.ignore_patterns('*.cpp', '*.c'))
	
	print("successfully installed")

licant.ex("libralgo.so")