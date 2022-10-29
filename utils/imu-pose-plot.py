#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots poses around IMU updates

import sys
import matplotlib.pyplot as plt
import json
import numpy as np

import quaternion
import math

# For observations from the camera, use the frame timestamp (True)
# or the time the pose was reported (False)
# display_frame_time = True
display_frame_time = False

relative_times = True

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

events_imu = []
events_poses = []
events_outputs = []

for i, l in enumerate(f):
    try:
        js = json.loads(l)
    except:
        print("Error parsing JSON at line {}".format(i))

    if js['type'] == 'imu':
        events_imu.append(js)
    elif js['type'] == 'output-pose':
        events_outputs.append(js)
    elif js['type'] == 'pose':
        events_poses.append(js)

print("Loaded {} IMU events + {} pose events + {} output poses".format(len(events_imu), len(events_poses), len(events_outputs)))
if len(events_poses) < 1 or len(events_outputs) < 1 or len(events_imu) < 1:
    sys.exit(0)

t_imu = list(map(lambda e: e['local-ts'] / 1000000000.0, events_imu))
if display_frame_time:
    t_poses = list(map(lambda e: e['frame-local-ts'] / 1000000000.0, events_poses))
else:
    t_poses = list(map(lambda e: e['local-ts'] / 1000000000.0, events_poses))
t_outputs = list(map(lambda e: e['local-ts'] / 1000000000.0, events_outputs))

if relative_times:
    base_time = t_poses[0]
    for t in (t_outputs, t_imu):
        if len(t) > 0 and t[0] < base_time:
            base_time = t[0]

    print("Base time: {}".format(base_time))
    t_imu = list(map(lambda t: t - base_time, t_imu))
    t_poses = list(map(lambda t: t - base_time, t_poses))
    t_outputs = list(map(lambda t: t - base_time, t_outputs))


vel_averages = np.mean(np.array(list(map(lambda e: e['lin-vel'], events_outputs))), axis=0)
accel_averages = np.mean(np.array(list(map(lambda e: e['lin-accel'], events_outputs))), axis=0)
max_jerk = np.amax(np.diff(np.array(list(map(lambda e: e['lin-accel'], events_outputs))), axis=0), axis=0)
print("Average velocity: {}".format(vel_averages))
print("Average acceleration: {}".format(accel_averages))
print("Maximum jerk (m/s^3): {}".format(max_jerk))

num_cols = 5

fig, axs = plt.subplots(3, num_cols, sharex=True)

axis_names = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z', 'vel X', 'vel Y', 'vel Z', 'accel X', 'accel Y', 'accel Z', 'ang vel X', 'ang vel Y', 'ang vel Z' ]

for col in range(num_cols):
    obs_poses = None

    if col == 0:
        priors = list(map(lambda e: euler_from_quaternion(e['pose-before']['orient']), events_imu))
        posteriors = list(map(lambda e: euler_from_quaternion(e['pose-after']['orient']), events_imu))
        obs_outputs = list(map(lambda e: euler_from_quaternion(e['orient']), events_outputs))
        obs_poses = list(map(lambda e: euler_from_quaternion(e['orient']), events_poses))
    elif col == 1:
        priors = list(map(lambda e: e['pose-before']['pos'], events_imu))
        posteriors = list(map(lambda e: e['pose-after']['pos'], events_imu))
        obs_outputs = list(map(lambda e: e['pos'], events_outputs))
        obs_poses = list(map(lambda e: e['pos'], events_poses))
    elif col == 2:
        priors = list(map(lambda e: e['lin-vel-before'], events_imu))
        posteriors = list(map(lambda e: e['lin-vel-after'], events_imu))
        obs_outputs = list(map(lambda e: e['lin-vel'], events_outputs))
    elif col == 3:
        priors = list(map(lambda e: e['lin-accel-before'], events_imu))
        posteriors = list(map(lambda e: e['lin-accel-after'], events_imu))
        obs_outputs = list(map(lambda e: e['lin-accel'], events_outputs))
    else:
        priors = list(map(lambda e: e['ang_vel'], events_imu))
        posteriors = list(map(lambda e: e['ang_vel'], events_imu))
        obs_outputs = list(map(lambda e: e['ang-vel'], events_outputs))

    for i in range(3):
        axs[i][col].plot(t_outputs, list(map(lambda p: p[i], obs_outputs)), '*-m', label='Output poses')
        axs[i][col].plot(t_imu, list(map(lambda p: p[i], priors)), '.-b', label='prior')
        axs[i][col].plot(t_imu, list(map(lambda p: p[i], posteriors)), '.-g', label='posterior')
        if obs_poses is not None:
            axs[i][col].plot(t_poses, list(map(lambda p: p[i], obs_poses)), '+c', label='Camera poses')
        axs[i][col].set_xlabel('Time')
        axs[i][col].set_ylabel(axis_names[i + 3*col])
        axs[i][col].grid(True)
        axs[i][col].legend(loc='lower right')

fig.tight_layout(pad=0.1)
fig.canvas.set_window_title('IMU Pose plot ' + sys.argv[1])
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.1, hspace=0.1)
plt.show()
