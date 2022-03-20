// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Simulated device tracker */

#ifndef __RECORDING_DEVICE_SIMULATOR_H__
#define __RECORDING_DEVICE_SIMULATOR_H__

#include "rift-tracker-common.h"
#include "rift-kalman-6dof.h"
#include "exponential-filter.h"
#include "rift-sensor-pose-helper.h"

/* Number of state slots to use for quat/position updates */
#define NUM_POSE_DELAY_SLOTS 3

typedef struct rift_tracked_device_simulator rift_tracked_device_simulator;
typedef struct rift_tracker_pose_report rift_tracker_pose_report;
typedef struct rift_tracker_pose_delay_slot rift_tracker_pose_delay_slot;

struct rift_tracker_pose_report {
		bool report_used; /* TRUE if this report has been integrated */
		posef pose;
		rift_pose_metrics score;
};

struct rift_tracker_pose_delay_slot {
	int slot_id;		/* Index of the slot */
	bool valid;			/* true if the exposure info was set */
	int use_count;	/* Number of frames using this slot */

	uint64_t device_time_ns; /* Device time this slot is currently tracking */

	/* rift_tracked_device_model_pose_update stores the observed poses here */
	int n_pose_reports;
	rift_tracker_pose_report pose_reports[RIFT_MAX_SENSORS];
	/* Number of reports we used from the supplied ones */
	int n_used_reports;
};

struct rift_tracked_device_simulator {
	rift_tracked_device base;

	int index; /* Index of this entry in the devices array for the tracker and exposures */

	/* 6DOF Kalman Filter */
	rift_kalman_6dof_filter ukf_fusion;

	/* Account keeping for UKF fusion slots */
	int delay_slot_index;
	rift_tracker_pose_delay_slot delay_slots[NUM_POSE_DELAY_SLOTS];

	/* The pose of the device relative to the IMU 3D space */
	posef device_from_fusion;

	/* The pose of the IMU relative to the LED model space */
	posef fusion_from_model;
	posef model_from_fusion;

	uint64_t device_time_ns;
	uint64_t last_imu_local_ts;

	uint64_t last_observed_orient_ts;
	uint64_t last_observed_pose_ts;
	posef last_observed_pose;

	/* Reported view pose (to the user) and model pose (for the tracking) respectively */
	uint64_t last_reported_pose;
	posef reported_pose;
	posef model_pose;

	exp_filter_pose pose_output_filter;
};

rift_tracked_device_simulator *rift_tracked_device_simulator_new(
	int device_id, rift_tracked_device_imu_calibration *imu_calibration,
	posef *imu_pose, posef *model_pose, int num_leds, rift_led *leds);

void rift_tracked_device_simulator_imu_update(rift_tracked_device_simulator *dev,
	uint64_t local_ts, uint64_t device_ts,
	const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);

#endif
