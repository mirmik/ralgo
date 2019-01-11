#!/usr/bin/env python3
#coding: utf-8

import pyralgo
import matplotlib.pyplot as plt  
import pyralgo as ralgo
import numpy as np
import cmath
import sympy
import sys

tau = 0.01
Tcor = 10
Kcor = 0.1
time = 200
N = int(time / tau)

regulator = ralgo.pi(Kcor/Tcor, 1/Tcor, tau)
#regulator2 = ralgo.pi(Kcor/Tcor, 1/Tcor, tau)
model = ralgo.oscilator(T=0.1, ksi=0.5, delta=tau)

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


pyralgo.plot_bode(regulator.trfunc() * model.trfunc())
print((regulator.trfunc() * model.trfunc()).factor())


plt.show()