#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots observed poses against priors

import sys
import matplotlib.pyplot as plt
import json
import numpy as np

import math

relative_times = True

# For observations from the camera, use the frame timestamp (True)
# or the time the pose was reported (False)
# display_frame_time = True
display_frame_time = False

# Plot linear velocity
plot_lin_vel = False
plot_lin_vel = True

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

events_a = []
events_b = []
events_outputs = []
events_priors = []
events_ignored = []
events_poses = []

for i, l in enumerate(f):
    try:
        js = json.loads(l)
    except:
        print("Error parsing JSON at line {}".format(i))

    if js['type'] == 'pose':
        if js['update-position'] == 0:
            events_ignored.append(js)
        elif js['source'] == 'WMTD307G100H61':
            events_a.append(js)
        else:
            events_b.append(js)
        events_poses.append(js)
    elif js['type'] == 'exposure':
        events_priors.append(js)
    elif js['type'] == 'output-pose':
        events_outputs.append(js)

print("Loaded {} + {} JSON pose events + {} output poses".format(len(events_a), len(events_b), len(events_outputs)))
if len(events_a) + len(events_b) + len(events_outputs) < 1:
    sys.exit(0)

t_priors = list(map(lambda e: e['local-ts'] / 1000000000.0, events_priors))
t_ignored = list(map(lambda e: e['local-ts'] / 1000000000.0, events_ignored))
if display_frame_time:
    t_a = list(map(lambda e: e['frame-local-ts'] / 1000000000.0, events_a))
    t_b = list(map(lambda e: e['frame-local-ts'] / 1000000000.0, events_b))
else:
    t_a = list(map(lambda e: e['local-ts'] / 1000000000.0, events_a))
    t_b = list(map(lambda e: e['local-ts'] / 1000000000.0, events_b))

t_poses = list(map(lambda e: e['local-ts'] / 1000000000.0, events_poses))
t_outputs = list(map(lambda e: e['local-ts'] / 1000000000.0, events_outputs))

if relative_times:
    base_time = t_priors[0]
    for t in (t_ignored, t_a, t_b, t_outputs, t_poses):
        if len(t) > 0 and t[0] < base_time:
            base_time = t[0]

    base_time = 866957.242079152
    print("Base time: {}".format(base_time))
    t_priors = list(map(lambda t: t - base_time, t_priors))
    t_poses = list(map(lambda t: t - base_time, t_poses))
    t_ignored = list(map(lambda t: t - base_time, t_ignored))
    t_a = list(map(lambda t: t - base_time, t_a))
    t_b = list(map(lambda t: t - base_time, t_b))
    t_outputs = list(map(lambda t: t - base_time, t_outputs))


vel_averages = np.mean(np.array(list(map(lambda e: e['lin-vel'], events_outputs))), axis=0)
accel_averages = np.mean(np.array(list(map(lambda e: e['lin-accel'], events_outputs))), axis=0)
print("Average velocity: {}".format(vel_averages))
print("Average acceleration: {}".format(accel_averages))

if plot_lin_vel:
    num_cols = 3
else:
    num_cols = 2

fig, axs = plt.subplots(3, num_cols, sharex=True)
fig.canvas.set_window_title('Pose plot ' + sys.argv[1])

axis_names = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z', 'vel X', 'vel Y', 'vel Z' ]

for col in range(num_cols):
    posteriors = None

    if col == 0:
        priors = list(map(lambda e: euler_from_quaternion(e['capture-orient']), events_priors))
        posteriors = list(map(lambda e: euler_from_quaternion(e['posterior-orient']), events_poses))
        obs_ignored = list(map(lambda e: euler_from_quaternion(e['orient']), events_ignored))
        obs_a = list(map(lambda e: euler_from_quaternion(e['orient']), events_a))
        obs_b = list(map(lambda e: euler_from_quaternion(e['orient']), events_b))
        obs_outputs = list(map(lambda e: euler_from_quaternion(e['orient']), events_outputs))
    elif col == 1:
        priors = list(map(lambda e: e['capture-pos'], events_priors))
        posteriors = list(map(lambda e: e['posterior-pos'], events_poses))
        obs_ignored = list(map(lambda e: e['pos'], events_ignored))
        obs_a = list(map(lambda e: e['pos'], events_a))
        obs_b = list(map(lambda e: e['pos'], events_b))
        obs_outputs = list(map(lambda e: e['pos'], events_outputs))
    else:
        obs_outputs = list(map(lambda e: e['lin-vel'], events_outputs))

    for i in range(3):
        if col != 2:
            axs[i][col].plot(t_a, list(map(lambda p: p[i], obs_a)), '.-r', label='Cam WMTD307G100H61')
            axs[i][col].plot(t_b, list(map(lambda p: p[i], obs_b)), '.-y', label='Cam WMTD30333009TJ')
            axs[i][col].plot(t_outputs, list(map(lambda p: p[i], obs_outputs)), '*-m', label='Output poses')
            axs[i][col].plot(t_priors, list(map(lambda p: p[i], priors)), '.-b', label='prior')
            if posteriors is not None:
                axs[i][col].plot(t_poses, list(map(lambda p: p[i], posteriors)), '.-c', label='posterior')
            axs[i][col].plot(t_ignored, list(map(lambda p: p[i], obs_ignored)), '.g', label='ignored')
        else:
            axs[i][col].plot(t_outputs, list(map(lambda p: p[i], obs_outputs)), '*-c', label='Output velocity (m/s)')
        axs[i][col].set_xlabel('Time')
        axs[i][col].set_ylabel(axis_names[i + 3*col])
        axs[i][col].grid(True)
        axs[i][col].legend(loc='lower right')

fig.tight_layout(pad=0.1)
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.1, hspace=0.1)
plt.show()
