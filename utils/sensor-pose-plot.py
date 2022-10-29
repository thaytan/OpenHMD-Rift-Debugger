#!/usr/bin/python3
# Utility which parses OpenHMD JSON sensor traces
# and plots observed poses

import sys
import traceback
import matplotlib.pyplot as plt
import numpy as np
import quaternion
import json
import re

import math

import models
import cvutil

relative_times = True
sort_times = True
filter_device_blobs = False
reproj_thresh=0.5

# Use the frame timestamp (True)
# or the time the pose was reported (False)
display_frame_time = True

def correct_pose(o):
    global device_id, sensor_id

    blobs = sorted(o['blobs'], key=lambda x: x.get('device'))
    fb = list(filter(lambda x: x['device'] == device_id, blobs))
    pose = list(filter(lambda x: x['device'] == device_id, o['poses']))
    if len(pose) == 0:
        # print("No device pose for device {}".format (device_id))
        return None, None
    pose = pose[0]['pose']

    # orientation in the file is x,y,z,w
    in_orient = pose['orient']
    in_pos = pose['pos']

    cam = models.cameras[sensor_id]

    orient = np.quaternion(in_orient[3], *in_orient[0:3])

    t, R, error_t, error_R = cvutil.refine_pose(models.ledmodels[device_id], fb, in_pos, orient, cam['M'], cam['K'], reproj_thresh)

    q = quaternion.from_rotation_matrix(R)

    # Rotate corrected pose 180°
    if device_id == 0:
        try:
            q = quaternion.from_rotation_matrix(quaternion.as_rotation_matrix(q) @ quaternion.as_rotation_matrix(np.quaternion(0.0, 0.0, 1.0, 0.0)))
        except Exception as e:
            print(e)

    return t, (q.x, q.y, q.z, q.w)

def dump_frame(o, verbose=False, plot_points=False):
    print("TS {} Frame time {} fast analysis {} ms long {} ms".format(o['local-ts'], o['frame-local-ts'],
        (o['fast-analysis-finish-local-ts'] - o['fast-analysis-start-local-ts']) / 1000000.0,
        (o['long-analysis-finish-local-ts'] - o['long-analysis-start-local-ts']) / 1000000.0))
    for p in o['poses']:
        print("Device {} pose: {} prior: {}".format(p['device'], p.get('pose'), p.get('pose-prior')))
    print("Blobs:")
    blobs = sorted(o['blobs'], key=lambda x: x.get('device'))
    for b in blobs:
        print("\t{}".format(b))

    global device_id, sensor_id

    cam = models.cameras[sensor_id]

    fb = list(filter(lambda x: x['device'] == device_id, blobs))
    imgpoints = list(map(lambda x: (x['x'], x['y'], x['led']), fb))

    blobs_unknowns = list(filter(lambda x: x['device'] == -1, blobs))
    other_known_blobs = list(filter(lambda x: x['device'] != device_id and x['device'] != -1, blobs))

    pose = list(filter(lambda x: x['device'] == device_id, o['poses']))
    if len(pose) == 0:
        pose = None
    else:
        pose = pose[0].get('pose')

    if pose is None:
        print("No device pose for device {}".format (device_id))

        match_projected = None

        if len(fb) > 3:
            t, R, error_t, error_R = cvutil.refine_pose(models.ledmodels[device_id], fb, None, None, cam['M'], cam['K'], reproj_thresh, verbose)
            if t is None or R is None:
                print ("PnP Failed")
            else:
                print ("PnP provided t={} R={} q={} error_t={} error_R={}".format(t, R, quaternion.from_rotation_matrix(R), error_t, error_R))

            if t is not None and R is not None:
                _, match_projected = cvutil.project_leds(models.ledmodels[device_id], fb, t, quaternion.from_rotation_matrix(R), cam['M'], cam['K'])

                for i, p in enumerate(match_projected):
                    print ("  LED {} ({},{}) -> ({},{}) err ({}, {})".format (imgpoints[i][2], imgpoints[i][0], imgpoints[i][1],
                        p[0][0], p[0][1], imgpoints[i][0] - p[0][0], imgpoints[i][1] - p[0][1]))

        other_known_points = list(map(lambda x: (x['x'], x['y']), other_known_blobs))
        unknown_points = list(map(lambda x: (x['x'], x['y']), blobs_unknowns))
        known_points = list(map(lambda x: (x['x'], x['y']), fb))

        o['local-ts']
        if display_frame_time:
            o_ts = o['frame-local-ts']/1000000000.0 - base_time
        else:
            o_ts = o['local-ts']/1000000000.0 - base_time

        frame_ts = o['frame-local-ts']

        fig, ax = plt.subplots()
        ax.set_title('LEDs - Tracking loss, no pose. TS {} frame {}'.format(o_ts, frame_ts))
        if len(unknown_points) > 0:
            ax.scatter(*zip(*unknown_points), marker='.', label="Unknown blobs")
        if len(other_known_points) > 0:
            ax.scatter(*zip(*other_known_points), marker='.', label="Other device blobs")
        if len(known_points) > 0:
            ax.scatter(*zip(*known_points), marker='.', label="Target device blobs")
        if match_projected is not None:
            ax.scatter(*zip(*match_projected.reshape(-1,2)), marker='+', label="Found pose matched LEDs")
        ax.legend()
        ax.set_xlim([0, 1280])
        ax.set_ylim([0, 960])

        for p in fb:
            ax.annotate(p['led'], (p['x'], p['y']))
        plt.show()

        return

    # orientation in the file is x,y,z,w
    in_orient = pose['orient']
    in_pos = pose['pos']

    orient = np.quaternion(in_orient[3], *in_orient[0:3])
    t, R, error_t, error_R = cvutil.refine_pose(models.ledmodels[device_id], fb, in_pos, orient, cam['M'], cam['K'], reproj_thresh, verbose)

    print ("Original t={} R={} q={}".format(in_pos, quaternion.as_rotation_matrix(orient), orient))
    if t is None or R is None:
        print ("PnP Failed")
    else:
        print ("PnP provided t={} R={} q={} error_t={} error_R={}".format(t, R, quaternion.from_rotation_matrix(R), error_t, error_R))

    print ("Matched Projected points (original pose):")
    all_projected_old, match_projected_old = cvutil.project_leds(models.ledmodels[device_id], fb, in_pos, orient, cam['M'], cam['K'])

    for i, p in enumerate(match_projected_old):
        print ("  LED {} ({},{}) -> ({},{}) err ({}, {})".format (imgpoints[i][2], imgpoints[i][0], imgpoints[i][1],
            p[0][0], p[0][1], imgpoints[i][0] - p[0][0], imgpoints[i][1] - p[0][1]))

    print ("Matched Projected points (new pose):")
    if t is not None and R is not None:
        all_projected, match_projected = cvutil.project_leds(models.ledmodels[device_id], fb, t, quaternion.from_rotation_matrix(R), cam['M'], cam['K'])

        for i, p in enumerate(match_projected):
            print ("  LED {} ({},{}) -> ({},{}) err ({}, {})".format (imgpoints[i][2], imgpoints[i][0], imgpoints[i][1],
                p[0][0], p[0][1], imgpoints[i][0] - p[0][0], imgpoints[i][1] - p[0][1]))

        print ("All Projected points (new pose):")
        for i, p in enumerate(all_projected):
            print ("  LED {} ({},{})".format (i, p[0][0], p[0][1]))
    else:
        match_projected = None

    if plot_points:
        unknown_points = list(map(lambda x: (x['x'], x['y']), blobs_unknowns))
        known_points = list(map(lambda x: (x['x'], x['y']), fb))
        other_known_points = list(map(lambda x: (x['x'], x['y']), other_known_blobs))

        o['local-ts']
        if display_frame_time:
            o_ts = o['frame-local-ts']/1000000000.0 - base_time
        else:
            o_ts = o['local-ts']/1000000000.0 - base_time

        frame_ts = o['frame-local-ts']

        fig, ax = plt.subplots()
        ax.set_title('LEDs - original pose vs corrected. TS {} frame {}'.format(o_ts, frame_ts))

        if len(unknown_points) > 0:
            ax.scatter(*zip(*unknown_points), marker='.', label="Unknown blobs")
        if len(other_known_points) > 0:
            ax.scatter(*zip(*other_known_points), marker='.', label="Other device blobs")
        ax.scatter(*zip(*known_points), marker='.', label="Matched blobs")
        ax.scatter(*zip(*match_projected_old.reshape(-1,2)), marker='^', label="Old pose matched LEDs")
        if match_projected is not None:
            ax.scatter(*zip(*match_projected.reshape(-1,2)), marker='+', label="New pose matched LEDs")
        ax.legend()
        ax.set_xlim([0, 1280])
        ax.set_ylim([0, 960])

        for p in fb:
            ax.annotate(p['led'], (p['x'], p['y']))
        plt.show()

# Find the observation within 1ms of the TS and print blobs for the previous frame and this one.
def dump_obs(obs, ts):
    for i, o in enumerate(obs):
        if display_frame_time:
            o_ts = o['frame-local-ts']
        else:
            o_ts = o['local-ts']
        if o_ts > ts + 1000000 or o_ts < ts - 1000000:
            continue
        if i == 1:
            print("No previous frame")
        else:
            print("--------------\nPrevious frame blobs")
            dump_frame(obs[i-1], True, False)
        print("--------------\nChosen frame blobs")
        dump_frame(o, True, True)
        if i >= len(obs):
            print("No next frame")
        else:
            print("--------------\nNext frame blobs")
            dump_frame(obs[i+1], True, False)

def filter_blobs(device_id, blobs):
    return list(filter(lambda b: (b['device'] == device_id), blobs))

def euler_from_quaternion(in_orient):
    global device_id

    if in_orient[3] < 0:
        in_orient = [v * -1 for v in in_orient]

    orient = np.quaternion(in_orient[3], *in_orient[0:3])
    a = quaternion.as_euler_angles(orient)
    roll_x, pitch_y, yaw_z = [((v+math.pi) % (2*math.pi)) - math.pi for v in a]

    # Fix up orientations for controllers where
    # the model faces backward
    if device_id != 0:
        yaw_z = 2*math.pi - yaw_z
        pitch_y = - pitch_y
        if roll_x > math.pi:
           roll_x = roll_x - math.pi
        else:
            roll_x = roll_x + math.pi

    return roll_x, pitch_y, yaw_z

if len(sys.argv) < 3:
    print("Usage: {} openhmd-sensor-trace device-id".format(sys.argv[0]))
    sys.exit(1)

f = open(sys.argv[1])
sensor_id = re.findall('rift-sensor-(.*)', sys.argv[1])[0]
device_id = int(sys.argv[2])

print("Loading poses for device {} from sensor {}".format(device_id, sensor_id))

correct_poses = []
correct_times = []

poses = []
times = []

pose_priors = []
times_priors = []

obs = []
last_obs = None

for i, l in enumerate(f):
    try:
        js = json.loads(l)
    except:
        print("Error parsing JSON at line {}".format(i))

    obs.append(js)
    for p in js['poses']:
        if p['device'] != device_id:
            continue
        if display_frame_time:
            ts = js['frame-local-ts'] / 1000000000.0
        else:
            ts = js['local-ts'] / 1000000000.0
        if 'pose' in p:
            times.append(ts)
            poses.append(p['pose'])
        if 'pose-prior' in p:
            times_priors.append(ts)
            pose_priors.append(p['pose-prior'])

print("Loaded {} JSON pose events".format(len(poses)))
if len(poses) < 1:
    sys.exit(0)

base_time = min(times[0], times_priors[0])
print("Base time: {}".format(base_time))

if relative_times:
    times = list(map(lambda t: t - base_time, times))
    times_priors = list(map(lambda t: t - base_time, times_priors))
else:
    base_time = 0

for i, o in enumerate(obs):
    try:
        pos, orient = correct_pose(o)
        if pos is not None:
            correct_poses.append({'orient': orient, 'pos': pos})
            if display_frame_time:
                correct_times.append(o['frame-local-ts'] / 1000000000.0 - base_time)
            else:
                correct_times.append(o['local-ts'] / 1000000000.0 - base_time)
    except:
        print("Failed to correct pose @ {}".format(o['local-ts'] / 1000000000.0 - base_time))
        # traceback.print_exception(*sys.exc_info())
        # dump_frame(o)

    if filter_device_blobs:
        fb = filter_blobs(device_id, o['blobs'])
        o['blobs'] = fb
    else:
        fb = o['blobs']

    if len(fb) < 1:
        continue

if sort_times:
    times, poses = zip(*sorted(zip(times, poses), key=lambda x: x[0]))
    times_priors, pose_priors = zip(*sorted(zip(times_priors, pose_priors), key=lambda x: x[0]))
    correct_times, correct_poses = zip(*sorted(zip(correct_times, correct_poses), key=lambda x: x[0]))

print("Corrected times {} poses {}".format(len(correct_poses), len(correct_times)))
print("Loaded {} pose priors".format(len(pose_priors)))

    # print("time {} - {} blobs".format(o['local-ts'] / 1000000000.0 - base_time, len(fb)))
    # for b in fb:
        # print("\t{}".format(b))

fig, axs = plt.subplots(3, 2, sharex=True)

axis_names = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z']

for col in range(2):
    if col == 0:
        vals = list(map(lambda e: euler_from_quaternion(e['orient']), poses))
        correct_vals = list(map(lambda e: euler_from_quaternion(e['orient']), correct_poses))
        prior_vals = list(map(lambda e: euler_from_quaternion(e['orient']), pose_priors))
    else:
        vals = list(map(lambda e: e['pos'], poses))
        correct_vals = list(map(lambda e: e['pos'], correct_poses))
        prior_vals = list(map(lambda e: e['pos'], pose_priors))

    for i in range(3):
        axs[i][col].plot(times_priors, list(map(lambda p: p[i], prior_vals)), '.-b', label='pose priors')
        axs[i][col].plot(times, list(map(lambda p: p[i], vals)), '.-r', label='poses', picker=5)
        axs[i][col].plot(correct_times, list(map(lambda p: p[i], correct_vals)), '.-g', label='corrected poses')
        axs[i][col].set_xlabel('Time')
        axs[i][col].set_ylabel(axis_names[i + 3*col])
        axs[i][col].grid(True)
        axs[i][col].legend(loc='lower right')

fig.tight_layout(pad=0.1)

def on_pick(event):
    line = event.artist
    xdata, ydata = line.get_data()
    ind = event.ind

    print("Picked Times:")
    for d in xdata[ind]:
        if relative_times:
            print("{} (abs {})".format(d, d + base_time))
            d += base_time
        else:
            print("{}".format(d))
        dump_obs(obs, d * 1000000000)

cid = fig.canvas.mpl_connect('pick_event', on_pick)
plt.show()
