// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Simulated device tracker */

#ifndef __RECORDING_SENSOR_SIMULATOR_H__
#define __RECORDING_SENSOR_SIMULATOR_H__

#include "rift-tracker-common.h"

typedef struct recording_simulator_sensor recording_simulator_sensor;

recording_simulator_sensor *recording_simulator_sensor_new(char *serial_no, rift_sensor_camera_params *calibration,
	posef *pose);
void recording_simulator_sensor_set_pose(recording_simulator_sensor *sensor, posef *camera_pose);

void recording_simulator_sensor_process_frame(recording_simulator_sensor *sensor,
				ohmd_video_frame *frame, rift_tracker_exposure_info *exp_info);

#endif
