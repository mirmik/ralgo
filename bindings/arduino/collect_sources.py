#!/usr/bin/env python3

import os
import shutil
import glob

shutil.rmtree("src")
os.makedirs("src/ralgo", exist_ok=True)

PREFIX = "../../ralgo"

SOURCES = [
]

for path, subdirs, files in os.walk(PREFIX):
    for f in [ f for f in files if os.path.splitext(f)[1] == ".h" ]:
    	difpath = os.path.relpath(path, PREFIX)
    	os.makedirs(os.path.join("src/ralgo", difpath), exist_ok=True)
    	shutil.copyfile(src=os.path.join(PREFIX, difpath, f), dst=os.path.join("src/ralgo", difpath, f))

for s in SOURCES:
	src = os.path.join(PREFIX, s)
	name = os.path.basename(s)

	shutil.copyfile(src, os.path.join("src", name))

#shutil.copytree(src="addon", dst="src", dirs_exist_ok=True)

with open("src/ralgo.h", "w") as f:
	f.write("//PLACEHOLDER\r\n")