#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots observed poses against priors

import sys
import matplotlib.pyplot as plt
import json

import math

relative_times = False
# relative_times = True

def euler_from_quaternion(in_orient):
    if in_orient[3] < 0:
        in_orient = [v * -1 for v in in_orient]

    orient = np.quaternion(in_orient[3], *in_orient[0:3])
    a = quaternion.as_euler_angles(orient)
    return [((v+math.pi) % (2*math.pi)) - math.pi for v in a]

if len(sys.argv) < 2:
    print("Usage: {} openhmd-device-trace".format(sys.argv[0]))
    sys.exit(1)

f = open(sys.argv[1])

events = []
events_priors = []
for i, l in enumerate(f):
    try:
        js = json.loads(l)
    except:
        print("Error parsing JSON at line {}".format(i))

    if js['type'] == 'output-pose':
        events.append(js)
    elif js['type'] == 'exposure':
        events_priors.append(js)

print("Loaded {} JSON pose events".format(len(events)))
if len(events) < 1:
    sys.exit(0)

t_priors = list(map(lambda e: e['local-ts'] / 1000000000.0, events_priors))
t = list(map(lambda e: e['local-ts'] / 1000000000.0, events))

if relative_times:
    base_time = t_priors[0]
    if len(t) > 0 and t[0] < base_time:
        base_time = t[0]
    t_priors = list(map(lambda t: t - base_time, t_priors))
    t = list(map(lambda t: t - base_time, t))

fig, axs = plt.subplots(3, 5, sharex=True)

axis_names = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z',
    'Ang Vel X', 'Ang Vel Y', 'Ang Vel Z',
    'Lin Vel X', 'Lin Vel Y', 'Lin Vel Z',
    'Lin Accel X', 'Lin Accel Y', 'Lin Accel Z'
]

for col in range(5):
    if col == 0:
        priors = list(map(lambda e: euler_from_quaternion(e['capture-orient']), events_priors))
        obs = list(map(lambda e: euler_from_quaternion(e['orient']), events))
    elif col == 1:
        priors = list(map(lambda e: e['capture-pos'], events_priors))
        obs = list(map(lambda e: e['pos'], events))
    elif col == 2:
        priors = None
        obs = list(map(lambda e: e['ang-vel'], events))
    elif col == 3:
        priors = None
        obs = list(map(lambda e: e['lin-vel'], events))
    else:
        priors = None
        obs = list(map(lambda e: e['lin-accel'], events))

    for i in range(3):
        axs[i][col].plot(t, list(map(lambda p: p[i], obs)), '.-y', label='obs')
        if priors is not None:
            axs[i][col].plot(t_priors, list(map(lambda p: p[i], priors)), '.-b', label='prior')
        axs[i][col].set_xlabel('Time')
        axs[i][col].set_ylabel(axis_names[i + 3*col])
        axs[i][col].grid(True)
        axs[i][col].legend(loc='lower right')

fig.tight_layout(pad=0.1)
plt.subplots_adjust(wspace=.25, hspace=.25)
plt.show()
