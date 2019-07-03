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

q=ralgo.htrans(ralgo.quat(0,0,0,1),(8,0,0))
a=ralgo.htrans(ralgo.rotation_quat((0,0,1),ralgo.deg(30)),(0,0,0))
b=ralgo.htrans(ralgo.rotation_quat((1,0,0),ralgo.deg(44)),(0,0,0))
#c=ralgo.htrans(ralgo.rotation_quat((0,1,0),ralgo.deg(33)),(0,0,0))

m = q*a*b

qlink = ralgo.cynematic.translation_link((1,0,0))
alink = ralgo.cynematic.rotation_link((0,0,1))
blink = ralgo.cynematic.rotation_link((1,0,0))
#clink = ralgo.cynematic.rotation_link((0,1,0))

chain = ralgo.cynematic.chain(
	[
		qlink,
		alink, 
		blink, 
		#clink
	]
)

ret = None

@timing
def do():
	global ret

	for i in range(1):
		ret = chain.solve_inverse_cynematic(m, [0,0,0], 1)

do()

print(chain.get(ret[0]))

print(m)
print(ret)