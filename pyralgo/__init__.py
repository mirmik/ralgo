import matplotlib.pyplot as plt   
import pyralgo.libralgo as lib
import sympy
import numpy as np
import cmath
import math

s = sympy.symbols("s")

class pi(lib.pi):
	def trfunc(self):
		return self.kp() + self.ki() * 1/s 

class zv1(lib.zv1):
	def trfunc(self):
		return 1/(self.a()*s+1) 

class zv2(lib.zv2):
	def trfunc(self):
		return 1/(self.a()*s*s+self.b()*s+1) 


def plot_bode(w, start=-5, stop=5, num=100):
	w = w.factor()
	pnts = np.logspace(start, stop, num, endpoint=True)
	
	phs=[]
	amp=[]
	
	for omega in pnts:
		c = w.subs({s:omega*1j})
		am, ph = cmath.polar(c)
		phs.append(ph)
		amp.append(am)
	
	fig = plt.figure()
	
	ax = fig.add_subplot(2, 1, 1)
	line, = ax.plot(pnts, amp)
	ax.plot(pnts, np.full(len(pnts), 1))
	ax.set_xscale('log')
	ax.set_yscale('log')
	
	ax = fig.add_subplot(2, 1, 2)
	line, = ax.plot(pnts, np.array(phs) * 180 / math.pi)
	ax.set_xscale('log')
	ax.set_ylim((-180 - 0.1, 180 + 0.1))

def plot_step_responce(model, time, step):
	space = np.r_[0:time:step]

	sig = 1
	values = []
	for i in range(0,len(space)):
		values.append(model(sig))
	
	plt.plot(space, values)

	