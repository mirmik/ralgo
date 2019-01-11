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

#pid = ralgo.pd.by_attrs(K=0.01,T=10)
regulator = ralgo.pd.by_attrs(K=0.01, T=10)
model = ralgo.oscilator.by_attrs(T=0.1, ksi=0, delta=0.01)
#tf = model.to_tf()
#st = scipy.signal.step(tf)

w = model.trfunc() * regulator.trfunc()

#ralgo.plot_bode(w)
#ralgo.plot_bode(model.trfunc())
#ralgo.plot_bode(regulator.trfunc())
print(ralgo.zfunc(w, -1))
print(ralgo.zroots(w))

ralgo.plot_step_responce_tf(model.trfunc(), 2, points = 100)

plt.show()