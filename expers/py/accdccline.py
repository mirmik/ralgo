#!/usr/bin/env python3
#coding: utf-8

import pyralgo
import numpy as np
import matplotlib.pyplot as plt

acctime = 10
lintime = 20
dcctime = 10

traj = pyralgo.accdcc_by_time_trajectory(x0=0,x1=100,tacc=acctime,tlin=lintime,tdcc=dcctime)

print(traj)

ls = np.linspace(0, acctime + lintime + dcctime, 100)
posarr = []
velarr = []
for i in ls:
	posarr.append(traj.inloctime(i)[0].d0)
	velarr.append(traj.inloctime(i)[0].d1)

plt.plot(ls, posarr, ls, velarr)
plt.show()