#!/usr/bin/env python3
#coding: utf-8

import pyralgo
import pyralgo.libralgo

kp 			= 0.5
ki_discr 	= pyralgo.libralgo.ki_discr(kp=kp, kip=0.5, delta=0.01)
reg 		= pyralgo.libralgo.pi_regulator_const_delta(kp, ki_discr)

r=0
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)
r = reg(r, 1); print(r)