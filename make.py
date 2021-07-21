#!/usr/bin/env python3
#coding:utf-8

import os
import shutil
import licant
import licant.install

licant.include("ralgo", "ralgo.g.py")

licant.execute("apps/sigtrans/make.py")

target = "libralgo.so"

install_include_path = '/usr/local/include/ralgo' 
install_directory_path = '/usr/lib/'
install_library_path = os.path.join(install_directory_path, target)


licant.cxx_shared_library("libralgo.so",
	mdepends=[
		"ralgo"
	],
	cxx_flags="-fPIC",
	cc_flags="-fPIC -Werror=implicit-function-declaration",
	libs=["igris", "nos"]
)

licant.install.install_library(
	tgt="install_library",
	uninstall="uninstall",
	libtgt="libralgo.so",
	hroot="ralgo",
	headers="ralgo")

licant.fileset("apps", targets=[
	"sigtrans"
], deps=["libralgo.so"])

licant.fileset("all", targets=["apps", target])

@licant.routine(deps=["apps"])
def install_apps():
	licant.do(["install_sigtrans", "makefile"])

@licant.routine(deps=["install_apps", "install_library"])
def install():
	pass

licant.ex("all")