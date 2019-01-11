#!/usr/bin/env python3
#coding: utf-8

import pyralgo
import matplotlib.pyplot as plt  
import pyralgo as ralgo
import numpy as np
import cmath
import sys

Tcor = 0.5
Kcor = 0.005

tau = 0.001
time = 100
N = int(time / tau)
model = ralgo.zv2(0.1, 0.5, tau)
regulator = ralgo.pi(Kcor/Tcor, 1/Tcor, tau)

print((model.trfunc() * regulator.trfunc()).factor())

#ralgo.plot_bode(regulator.trfunc(), -5, 5, 200)
#plt.show()
#sys.exit()

values = []
sigs = []
for i in range(0,N):
	target = 20
	sig = regulator(target - model.output())
	values.append(model(sig))
	sigs.append(sig)
	
plt.subplot(2, 1, 1)
plt.plot(np.arange(0,N) * tau, values)

plt.subplot(2, 1, 2)
plt.plot(np.arange(0,N) * tau, sigs)

w = model.trfunc() * regulator.trfunc()
ralgo.plot_bode(w, -5, 5, 200)


plt.show()
