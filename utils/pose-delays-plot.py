#!/usr/bin/python3
# Utility which parses OpenHMD JSON traces
# and plots observed poses against priors

import sys
import matplotlib.pyplot as plt
import json

import math

# relative_times = False
relative_times = True

if len(sys.argv) < 2:
    print("Usage: {} openhmd-device-trace".format(sys.argv[0]))
    sys.exit(1)

f = open(sys.argv[1])

events_a = []
events_b = []
events_ignored = []
for l in f:
    js = json.loads(l)
    if js['type'] == 'pose':
        if js['source'] == 'WMTD307G100H61':
            events_a.append(js)
        else:
            events_b.append(js)

print("Loaded {} + {} JSON pose events".format(len(events_a), len(events_b)))
if len(events_a) + len(events_b) < 1:
    sys.exit(0)

t_a = list(map(lambda e: e['local-ts'] / 1000000.0, events_a))
t_b = list(map(lambda e: e['local-ts'] / 1000000.0, events_b))

if relative_times:
    if len(t_a) > 0:
        base_time = t_a[0]
    else:
        base_time = t_b[0]

    if len(t_b) > 0 and t_b[0] < base_time:
        base_time = t_b[0]

    t_a = list(map(lambda t: t - base_time, t_a))
    t_b = list(map(lambda t: t - base_time, t_b))

fig, axs = plt.subplots(2, sharex=True)

axis_names = [ 'pose latency (ms)', 'dt (ms)']

for col in range(2):
    if col == 0:
        # Calculate delay from frame capture to pose arrival
        obs_a = list(map(lambda e: (e['local-ts'] - e['frame-start-local-ts']) / 1000000.0, events_a))
        obs_b = list(map(lambda e: (e['local-ts'] - e['frame-start-local-ts']) / 1000000.0, events_b))
    else:
        # Calculate dt for each (distance between pose arrivals)
        obs_a = [ 0 ]
        for (index, o) in enumerate(events_a[:-1]):
            current, next_ = o, events_a[index + 1]
            obs_a.append((next_['local-ts'] - current['local-ts']) / 1000000.0)

        obs_b = [ 0 ]
        for (index, o) in enumerate(events_b[:-1]):
            current, next_ = o, events_b[index + 1]
            obs_b.append((next_['local-ts'] - current['local-ts']) / 1000000.0)

    axs[col].plot(t_a, list(obs_a), '.-r', label='WMTD307G100H61', markerfacecolor='tab:blue')
    axs[col].plot(t_b, list(obs_b), '.-y', label='WMTD30333009TJ', markerfacecolor='tab:blue')
    axs[col].set_xlabel('Time')
    axs[col].set_ylabel(axis_names[col])
    axs[col].grid(True)
    axs[col].legend(loc='lower right')

fig.tight_layout(pad=1.0)
plt.show()
