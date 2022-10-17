#include <assert.h>
#include <glib.h>
#include <stdio.h>
#include <string.h>

#include "recording-simulator.h"
#include "recording-loader.h"
#include "recording-device-simulator.h"
#include "recording-sensor-simulator.h"

#include "rift-tracker-common.h"
#include "rift-sensor-blobwatch.h"
#include "rift-sensor-pose-search.h"

struct recording_simulator_sensor {
	blobwatch* bw;
	rift_pose_finder pf;
};

/* Callback from the pose search when it finds something */
static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score)
{
	return FALSE;
}

recording_simulator_sensor *
recording_simulator_sensor_new(char *serial_no, rift_sensor_camera_params *calibration, posef *pose)
{
	recording_simulator_sensor *sensor = calloc(1, sizeof(recording_simulator_sensor));

	sensor->bw = blobwatch_new(calibration->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2);
	rift_pose_finder_init(&sensor->pf, calibration, (rift_pose_finder_cb) handle_found_pose, sensor);

	return sensor;
}

void
recording_simulator_sensor_process_frame(recording_simulator_sensor *sensor, ohmd_video_frame *frame)
{
	blobservation* bwobs = NULL;

	blobwatch_process(sensor->bw, frame->data, frame->width, frame->height, 0, NULL, 0, &bwobs);

	if (bwobs) {
		blobwatch_release_observation(sensor->bw, bwobs);
	}
}
