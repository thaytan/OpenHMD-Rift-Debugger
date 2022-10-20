#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots observed poses against priors

import sys
import matplotlib.pyplot as plt
import json

import math

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
events_ignored = []
for l in f:
    js = json.loads(l)
    if js['type'] == 'pose':
        if js['update-position'] == 0:
            events_ignored.append(js)
        else:
            events.append(js)
    elif js['type'] == 'exposure':
        events_priors.append(js)

print("Loaded {} JSON orientation events".format(len(events)))
if len(events) < 1:
    sys.exit(0)

t_priors = list(map(lambda e: e['local-ts'] / 1000000000.0, events_priors))
priors = list(map(lambda e: euler_from_quaternion(e['capture-orient']), events_priors))

t = list(map(lambda e: e['local-ts'] / 1000000000.0, events))
obs = list(map(lambda e: euler_from_quaternion(e['orient']), events))

t_ignored = list(map(lambda e: e['local-ts'] / 1000000000.0, events_ignored))
obs_ignored = list(map(lambda e: euler_from_quaternion(e['orient']), events_ignored))

fig, axs = plt.subplots(3, 1, sharex=True)

axis_names = ['Roll', 'Pitch', 'Yaw']

for i in range(3):
    axs[i].plot(t_priors, list(map(lambda p: p[i], priors)), '.-b', label='prior')
    axs[i].plot(t, list(map(lambda p: p[i], obs)), '.-r', label='obs')
    axs[i].plot(t_ignored, list(map(lambda p: p[i], obs_ignored)), '.g', label='ignored')
    axs[i].set_xlabel('Time')
    axs[i].set_ylabel(axis_names[i])
    axs[i].grid(True)
    axs[i].legend(loc='lower right')

fig.tight_layout()
plt.show()
