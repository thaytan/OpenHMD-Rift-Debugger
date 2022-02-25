#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots observed poses against priors

import sys
import numpy as np
import matplotlib.pyplot as plt

# Orig distortion
compare_dists=[
        ("Original", 0.098, .324, -0.241, 0.819),
        # ("take 1", 0.269, -0.287, +0.178, 0.84),
        # ("Original reversed", 0.819, -0.241, 0.324, 0.098),
        ("None", 0.0, 0.0, 0.0, 1.0),
        ]


if len(sys.argv) < 4:
    print("Usage: {} a b c".format(sys.argv[0]))
    sys.exit(1)

(a,b,c) = map(lambda x: float(x), sys.argv[1:5])
d=1.0 - (a+b+c)

print("Calculating distortion for {} {} {} {}".format(a,b,c,d))

dist_params=(a,b,c,d)

step=0.05
r=np.arange(0., 1.+step, step)

def f(r, params):
    (a,b,c,d) = params
    return ((((r*a + b) * r) + c) * r + d) * r

plt.plot(r, f(r, dist_params), 'bo-', label='New')
for d in compare_dists:
    plt.plot(r, f(r, d[1:]), label=d[0])

plt.legend(loc='lower right')
plt.gca().set_aspect('equal')
plt.show()
