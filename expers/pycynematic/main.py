#!/usr/bin/env python3

import ralgo
import ralgo.cynematic

from functools import wraps

import time

def timing(f):
    @wraps(f)
    def wrap(*args, **kw):
        ts = time.time()
        result = f(*args, **kw)
        te = time.time()
        print ('func:%r args:[%r, %r] took: %2.4f sec' % (f.__name__, args, kw, te-ts))
        return result
    return wrap

a=ralgo.htrans(ralgo.rotation_quat((0,0,1),ralgo.deg(1)),(0,0,0))
b=ralgo.htrans(ralgo.rotation_quat((1,0,0),ralgo.deg(1)),(0,0,0))
#c=ralgo.htrans(ralgo.rotation_quat((0,1,0),ralgo.deg(33)),(0,0,0))

m = a*b#*c

alink = ralgo.cynematic.rotation_link((0,0,1))
blink = ralgo.cynematic.rotation_link((1,0,0))
#clink = ralgo.cynematic.rotation_link((0,1,0))

chain = ralgo.cynematic.chain(
	[
		alink, 
		blink, 
		#clink
	]
)

ret = None

@timing
def do():
	global ret

	for i in range(100):
		ret = chain.solve_inverse_cynematic(m, [0,0], 1)

do()

print(m)
print(ret)