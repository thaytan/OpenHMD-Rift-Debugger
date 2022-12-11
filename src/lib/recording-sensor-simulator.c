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
	char *serial_no;
	blobwatch* bw;
	rift_pose_finder pf;

	rift_tracked_device *devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;
};

/* Callback from the pose search when it finds something */
static bool handle_found_pose (recording_simulator_sensor *sensor,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score)
{
	printf("Sensor %s Got pose for device %d pose quat %f %f %f %f  pos %f %f %f\n",
		sensor->serial_no, dev->id, obj_world_pose->orient.x, obj_world_pose->orient.y, obj_world_pose->orient.z, obj_world_pose->orient.w,
		obj_world_pose->pos.x, obj_world_pose->pos.y, obj_world_pose->pos.z);

	/* Compute an error of 1cm at right angles to the camera, and 3cm in Z.
	 * FIXME: Compute based on pixel error and distance from camera and pass
	 * it in */
	vec3f cam_obs_pos_error = {{ 0.01, 0.01, 0.03 }};
	vec3f world_obs_pos_error;

	quatf *cam_orient = &sensor->pf.camera_pose.orient;
	oquatf_get_rotated(cam_orient, &cam_obs_pos_error, &world_obs_pos_error);

  rift_tracked_device_simulator *sim_dev = (rift_tracked_device_simulator *)dev;
	bool ret = rift_tracked_device_simulator_model_pose_update(sim_dev, &frame->exposure_info, score, obj_world_pose, &world_obs_pos_error, sensor->serial_no);

	if (ret) {
		/* If this pose was accepted by the tracker, transfer these blob labels to the blobwatch object */
		blobwatch_update_labels (sensor->bw, frame->bwobs, dev->id);
	} else { /* FIXME: Only do this after a few failures? */
		rift_clear_blob_labels (frame->bwobs->blobs, frame->bwobs->num_blobs, dev->id);

	  printf("Sensor %s pose for device %d pose quat %f %f %f %f  pos %f %f %f was rejected by fusion\n",
		  sensor->serial_no, dev->id, obj_world_pose->orient.x, obj_world_pose->orient.y, obj_world_pose->orient.z, obj_world_pose->orient.w,
		  obj_world_pose->pos.x, obj_world_pose->pos.y, obj_world_pose->pos.z);
	}

	return ret;
}

recording_simulator_sensor *
recording_simulator_sensor_new(char *serial_no, rift_sensor_camera_params *calibration, posef *pose)
{
	recording_simulator_sensor *sensor = calloc(1, sizeof(recording_simulator_sensor));

	sensor->serial_no = g_strdup(serial_no);
	sensor->bw = blobwatch_new(calibration->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2);
	rift_pose_finder_init(&sensor->pf, calibration, (rift_pose_finder_cb) handle_found_pose, sensor);

	if (pose)
		recording_simulator_sensor_set_pose(sensor, pose);

	return sensor;
}

void
recording_simulator_sensor_free(recording_simulator_sensor *sensor)
{
	free(sensor->serial_no);

	blobwatch_free(sensor->bw);
	rift_pose_finder_clear(&sensor->pf);

	free(sensor);
}		

void
recording_simulator_sensor_set_pose(recording_simulator_sensor *sensor, posef *camera_pose)
{
	const vec3f gravity_vector = {{ 0.0, 1.0, 0.0 }};

	sensor->pf.camera_pose = *camera_pose;
	sensor->pf.have_camera_pose = true;
	sensor->pf.camera_pose_changed = false;

	quatf cam_orient = camera_pose->orient;

	oquatf_inverse(&cam_orient);
	oquatf_get_rotated(&cam_orient, &gravity_vector, &sensor->pf.cam_gravity_vector);
}

void recording_simulator_sensor_set_devices(recording_simulator_sensor *sensor,
	rift_tracked_device **devices, uint8_t n_devices)
{
	int i, cur;

	for (i = 0; i < n_devices; i++) {
		bool found_device = false;
		rift_tracked_device *new_dev = devices[i];

		for (cur = 0; cur < sensor->n_devices; cur++) {
			rift_tracked_device *cur_dev = sensor->devices[cur];
			if (cur_dev->id == new_dev->id) {
				found_device = true;
				break;
			}
		}

		if (!found_device) {
			sensor->devices[sensor->n_devices++] = new_dev;

			printf("adding tracked device %d to sensor %s\n", new_dev->id, sensor->serial_no);

			bool ret = correspondence_search_set_model (sensor->pf.cs, new_dev->id, new_dev->led_search);
			assert (ret);
		}
	}
}

static void init_frame_analyse(rift_sensor_analysis_frame *frame)
{
	int d;
	const rift_tracker_exposure_info *exposure_info = &frame->exposure_info;

	frame->need_long_analysis = false;
	frame->long_analysis_found_new_blobs = false;
	frame->long_analysis_start_ts = frame->long_analysis_finish_ts = 0;

	for (d = 0; d < exposure_info->n_devices; d++) {
		rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
		const rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
		const vec3f *rot_error = &exp_dev_info->rot_error;

		dev_state->capture_world_pose = exp_dev_info->capture_pose;

		/* Compute gravity error from XZ error range */
		dev_state->gravity_error_rad = OHMD_MAX(rot_error->x, rot_error->z);

		/* Mark the score as un-evaluated to start */
		dev_state->score.match_flags = 0;
		dev_state->found_device_pose = false;
	}
	frame->n_devices = exposure_info->n_devices;
}

void
recording_simulator_sensor_process_frame(recording_simulator_sensor *sensor,
		ohmd_video_frame *vframe, rift_tracker_exposure_info *exp_info)
{
	blobservation* bwobs = NULL;
	rift_sensor_analysis_frame aframe = { 0, };

	aframe.vframe = vframe;
	if (exp_info) {
		aframe.exposure_info = *exp_info;
		aframe.exposure_info_valid = true;
	}

	init_frame_analyse(&aframe);
	printf("*** Sensor %s New frame TS %" G_GUINT64_FORMAT "\n", sensor->serial_no, vframe->start_ts);

	blobwatch_process(sensor->bw, vframe->data, vframe->width, vframe->height, 0, NULL, 0, &bwobs);

	if (bwobs == NULL || bwobs->num_blobs == 0) {
		// Found no blobs to process
	  if (bwobs)
			blobwatch_release_observation(sensor->bw, bwobs);
		return;
	}

	aframe.bwobs = bwobs;

	if (sensor->pf.have_camera_pose) {
		printf("Sensor %s Found %d blobs. Doing fast analysis\n", sensor->serial_no, bwobs->num_blobs);
		rift_pose_finder_process_blobs_fast(&sensor->pf, &aframe, sensor->devices);
	} else {
		aframe.need_long_analysis = true;
	}

	if (aframe.need_long_analysis) {
		printf("Sensor %s Found %d blobs. Doing long analysis\n", sensor->serial_no, bwobs->num_blobs);

		rift_pose_finder_process_blobs_long(&sensor->pf, &aframe, sensor->devices);
	}

	blobwatch_release_observation(sensor->bw, bwobs);
}
