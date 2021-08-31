#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots observed poses against priors

import sys
import matplotlib.pyplot as plt
import json

import math

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    (x, y, z, w) = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    # Manual adjustments to avoid unecessary
    # wraparounds in the typical range of our data
    if roll_x < 0:
        roll_x += 2 * math.pi
    if pitch_y < -math.pi/2:
        pitch_y += 2 * math.pi
    if yaw_z < 0:
        yaw_z += 2 * math.pi

    return roll_x, pitch_y, yaw_z

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
t_ignored = list(map(lambda e: e['local-ts'] / 1000000000.0, events_ignored))
t = list(map(lambda e: e['local-ts'] / 1000000000.0, events))

fig, axs = plt.subplots(3, 2, sharex=True)

axis_names = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z']

for col in range(2):
    if col == 0:
        priors = list(map(lambda e: euler_from_quaternion(e['capture-orient']), events_priors))
        obs_ignored = list(map(lambda e: euler_from_quaternion(e['orient']), events_ignored))
        obs = list(map(lambda e: euler_from_quaternion(e['orient']), events))
    else:
        priors = list(map(lambda e: e['capture-pos'], events_priors))
        obs_ignored = list(map(lambda e: e['pos'], events_ignored))
        obs = list(map(lambda e: e['pos'], events))

    for i in range(3):
        axs[i][col].plot(t_priors, list(map(lambda p: p[i], priors)), '.-b', label='prior')
        axs[i][col].plot(t_ignored, list(map(lambda p: p[i], obs_ignored)), '.g', label='ignored')
        axs[i][col].plot(t, list(map(lambda p: p[i], obs)), '.-r', label='obs')
        axs[i][col].set_xlabel('Time')
        axs[i][col].set_ylabel(axis_names[i + 3*col])
        axs[i][col].grid(True)
        axs[i][col].legend(loc='lower right')

fig.tight_layout(pad=1.0)
plt.show()
