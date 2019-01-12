#!/usr/bin/env python3
#coding: utf-8

import pyralgo
import matplotlib.pyplot as plt  
import pyralgo as ralgo
import numpy as np
import cmath
import sympy
import sys

import scipy

#regulator = ralgo.regulator_d(0.01)
#regulator = ralgo.pd.by_attrs(K=0.001,T=3)
#regulator = ralgo.pd.by_attrs(K=0.01, T=10)
regulator = ralgo.pid.by_attrs(K=1, T1=100, T2=1, delta=0.01)
#model = ralgo.oscilator.by_attrs(T=0.1, ksi=0.1, delta=0.01)
#tf = model.to_tf()
#st = scipy.signal.step(tf)

#print(regulator.trfunc())

#w = model.trfunc() * regulator.trfunc()

#ralgo.plot_bode(model.trfunc())
#ralgo.plot_bode(regulator.trfunc())
#print(ralgo.zfunc(w, -1))
print(ralgo.zroots(regulator.trfunc()))

#ralgo.plot_step_responce_tf(ralgo.zfunc(w, -1), 20, points = 100)
ralgo.plot_bode(regulator.trfunc())
#ralgo.plot_bode(ralgo.zfunc(w, -1))

plt.show()