#!/usr/bin/env python3
#coding: utf-8

import pyralgo
import numpy as np
import matplotlib.pyplot as plt

traj = pyralgo.accdcc_by_time_trajectory(x0=0,x1=100,tacc=10,tlin=20,tdcc=10)

print(traj)

ls = np.linspace(0, 40, 100)
arr = []
for i in ls:
	arr.append(traj.inloctime(i)[0].d0)

plt.plot(ls,arr)
plt.show()