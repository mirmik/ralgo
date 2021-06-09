import matplotlib.pyplot as plt   
from ralgo.libralgo import * 
import sympy
import numpy as np
import cmath
import math

import scipy
import scipy.signal
import scipy.linalg

from sympy import *
from numpy import *

#from ralgo.libralgo import keep_trajectory
#from ralgo.libralgo import accdcc_by_time_trajectory

s, t = sympy.symbols("s t")

#class pi(lib.pi):
#	def trfunc(self):
#		return self.kp() + self.ki() * 1/s 
#	@staticmethod
#	def by_attrs(K, T, delta): return pi(K/T, 1/T, delta)
#
#class pid(lib.pi):
#	def __init__(self, kp, ki, kd, delta): self._kp = kp; self._ki = ki; self._kd = kd;  
#	def trfunc(self):
#		return self.kp() + self.ki() * 1/s + self.kd() * s
#	def kp(self): return self._kp
#	def ki(self): return self._ki
#	def kd(self): return self._kd
#	def to_tf(self):
#		return scipy.signal.TransferFunction([self.kd(), self.kp(), self.ki()], [1,0])
#	@staticmethod
#	def by_attrs(K, T1, T2, delta): return pid(K*T1, K, K*T1*T2, delta)
#
#
#class pd(lib.pi):
#	def __init__(self, kp, kd, delta): self._kp = kp; self._kd = kd;  
#	def trfunc(self):
#		return self.kp() + self.kd() * s
#	def kp(self): return self._kp
#	def kd(self): return self._kd
#	@staticmethod
#	def by_attrs(K, T, delta): return pd(K, T*K, delta)

class regulator_p:
	def __init__(self, kp): self._kp = kp;  
	def trfunc(self):
		return self.kp()
	def kp(self): return self._kp

class regulator_d:
	def __init__(self, kd): self._kd = kd;  
	def trfunc(self):
		return self.kd() * s
	def kd(self): return self._kd
	
#class aperiodic(lib.aperiodic):
#	def trfunc(self):
#		return 1/(self.a()*s+1) 
#
#class oscilator(lib.oscilator):
#	def trfunc(self):
#		return 1/(self.a()*s*s+self.b()*s+1) 
#
#	@staticmethod	
#	def by_attrs(T, ksi, delta): return oscilator(T**2, 2*ksi*T, delta)
#	#def to_tf(self):
#	#	return scipy.signal.TransferFunction([1], [self.a(), self.b(), 1])


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
	ax.set_ylim((-180 - 5, 180 + 5))

def plot_step_responce(model, time, step):
	space = np.r_[0:time:step]

	sig = 1
	values = []
	for i in range(0,len(space)):
		values.append(model(sig))
	
	plt.plot(space, values)

def zfunc(w, wo):
	return (w / (1 + w*wo)).factor()

def numer_denum_poly(w):
	w=w.factor()
	numer, denum = sympy.fraction(w)
	npoly = sympy.Poly(numer, s)
	dpoly = sympy.Poly(denum, s)
	return npoly.all_coeffs(), dpoly.all_coeffs()

def roots(w):
	zf=w.factor()
	denum = sympy.fraction(zf)[1]
	poly = sympy.Poly(denum, s)
	return np.roots(poly.all_coeffs())

def zroots(w):
	zf=zfunc(w, -1).factor()
	denum = sympy.fraction(zf)[1]
	pl = sympy.poly(denum)
	return np.roots(pl.all_coeffs())

def heaviside(t):
	def f(t):
		return 1 if t>=0 else 0
	return np.vectorize(f)(t)

def plot_step_responce_il(w, time, points):
	w = w * 1/s
	w = w.factor()
	modules = [{'Heaviside': lambda x: np.heaviside(x, 1)}, 'numpy']
	step = time / points
	N = int(time / step)
	F = sympy.inverse_laplace_transform(w, s, t)
	F = sympy.lambdify(t, F, modules=modules)
	time = np.r_[0:time:N*1j]
	plt.plot(time, F(time))

def check_dimmension(A,B,C,D) :
	g_dim = B.shape[-1]
	x_dim = B.shape[-2]
	y_dim = C.shape[-2]

	assert(A.shape[-1] == x_dim)
	assert(A.shape[-2] == x_dim)
	assert(B.shape[-1] == g_dim)
	assert(B.shape[-2] == x_dim)
	assert(C.shape[-1] == x_dim)
	assert(C.shape[-2] == y_dim)
	assert(D.shape[-1] == g_dim)
	assert(D.shape[-2] == y_dim)

	return (g_dim, x_dim, y_dim)

def matrix_discretization(A,B,C,D,Step):
	_A = scipy.linalg.expm(A * Step)
	_B = scipy.linalg.inv(A).dot((np.identity(A.shape[0]) - _A).dot(B))
	_C = C
	_D = D
	return (_A,_B,_C,_D)




def plot_step_responce_tf(w, time, points):
	step = time / points
	numer, denum = numer_denum_poly(w)
	numer, denum = np.array(numer), np.array(denum)
	numer = numer / denum[0]
	denum = denum / denum[0]

	print("numer", numer)
	print("denum", denum)

	dim = len(denum) - 1
	
	A = np.zeros((dim, dim))
	for i in range(dim - 1):
		for j in range(dim):
			A[i,j] = 1 if i + 1 == j else 0
	for j in range(dim):
		A[dim - 1, j] = -denum[dim - j]

	B = np.zeros((dim, 1))
	B[dim - 1][0] = 1

	C = np.zeros((1, dim))
	for j in range(min(dim, len(numer))):
		C[0][len(numer) - j - 1] = numer[j]

	D = np.zeros((1, 1))

	#print(w)
	#print(numer, denum)
	print("A:",A)
	print("B:",B)
	print("C:",C)
	print("D:",D)
	#A,B,C,D = matrix_discretization(A,B,C,D,step)

	#print(A)
	#print(B)
	#print(C)
	#print(D)

	x=np.zeros((dim, 1))

	N = int(time / step)

	arr = []
	for i in range(N):
		x = x + (A.dot(x) + B) * step
		arr.append(C.dot(x)[0,0])

	plt.plot(np.r_[0:time:N*1j], arr)


#import ralgo.libralgo
#from ralgo.libralgo import htrans, vec3, mat33, quat
#from ralgo.libralgo import rotation_quat
#from ralgo.libralgo import deg

def to_quat(a):
	if isinstance(a, tuple):
		return quat(*a)
	else: 
		return quat(a)

def to_vec3(a):
	if isinstance(a, tuple):
		return vec3(*a)
	else: 
		return vec3(a)

#class htrans(ralgo.libralgo.htrans):
#	def __init__(self,rot, mov):
#		super().__init__(to_quat(rot), to_vec3(mov))

def rotation_quat(ax, angle):
	return ralgo.libralgo.rotation_quat(to_vec3(ax), angle)