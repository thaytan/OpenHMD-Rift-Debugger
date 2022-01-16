#!/usr/bin/python3
# Utility which parses OpenHMD JSON sensor traces
# and plots observed poses

import sys
import matplotlib.pyplot as plt
import json

import math

relative_times = True
filter_device_blobs = True

def filter_blobs(device_id, blobs):
    return list(filter(lambda b: (b['device'] == device_id), blobs))

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

if len(sys.argv) < 3:
    print("Usage: {} openhmd-sensor-trace device-id".format(sys.argv[0]))
    sys.exit(1)

f = open(sys.argv[1])
device_id = int(sys.argv[2])

print("Loading poses for device {}".format(device_id))

poses = []
times = []
obs = []
for l in f:
    js = json.loads(l)
    for p in js['poses']:
        if p['device'] != device_id:
            continue
        times.append(js['local-ts'] / 1000000000.0)
        poses.append(p['pose'])
    obs.append(js)

print("Loaded {} JSON pose events".format(len(poses)))
if len(poses) < 1:
    sys.exit(0)

base_time = times[0]
if relative_times:
    times = list(map(lambda t: t - base_time, times))

for o in obs:
    if filter_device_blobs:
        fb = filter_blobs(device_id, o['blobs'])
    else:
        fb = o['blobs']

    if len(fb) < 1:
        continue

    # print("time {} - {} blobs".format(o['local-ts'] / 1000000000.0 - base_time, len(fb)))
    # for b in fb:
        # print("\t{}".format(b))

fig, axs = plt.subplots(3, 2, sharex=True)

axis_names = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z']

for col in range(2):
    if col == 0:
        vals = list(map(lambda e: euler_from_quaternion(e['orient']), poses))
    else:
        vals = list(map(lambda e: e['pos'], poses))

    for i in range(3):
        axs[i][col].plot(times, list(map(lambda p: p[i], vals)), '.-r', label='poses')
        axs[i][col].set_xlabel('Time')
        axs[i][col].set_ylabel(axis_names[i + 3*col])
        axs[i][col].grid(True)
        axs[i][col].legend(loc='lower right')

fig.tight_layout(pad=1.0)
plt.show()
