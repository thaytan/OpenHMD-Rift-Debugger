#include <assert.h>
#include <glib.h>
#include <stdio.h>
#include <string.h>

#include "recording-simulator.h"
#include "recording-loader.h"
#include "recording-device-simulator.h"
#include "recording-sensor-simulator.h"

#include "rift-tracker-common.h"

/* The recording simulator has 2 modes of operation: full or partial simulation.
 *
 * In a full simulation, the video frames are extracted from the recording
 * and full blob / pose extraction are executed. In a partial simulation,
 * the video frames are ignored and the pose data from the recording events
 * are used to re-run the filtering, for testing new filtering parameters
 */

typedef struct recording_simulator_stream recording_simulator_stream;
typedef struct recording_simulator_stream_video recording_simulator_stream_video;
typedef struct recording_simulator_event recording_simulator_event;
typedef struct recording_simulator_stream_events recording_simulator_stream_events;

struct recording_simulator {
	recording_loader *loader;

	gchar *json_output_dir;
	bool full_simulation;

	int n_sensors;
	recording_simulator_stream_video *sensor_video_stream[RIFT_MAX_SENSORS];

	recording_simulator_stream_events *global_metadata;

	int n_devices;
	recording_simulator_stream_events *tracked_device_event_stream[RIFT_MAX_TRACKED_DEVICES];
};

struct recording_simulator_stream {
	recording_stream_type type;
	recording_simulator *sim;
	char *stream_name;
};

struct recording_simulator_stream_video {
	recording_simulator_stream s;

	bool have_stream_config;
	int width, height, stride;

	bool have_calibration;
	char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
	rift_sensor_camera_params calibration;

	bool have_pose;
	posef pose;

	recording_simulator_sensor *sensor;
};

struct recording_simulator_event {
	uint64_t pts;
	struct data_point data;

	rift_tracked_device_simulator *device;
};

struct recording_simulator_stream_events {
	recording_simulator_stream s;

	/* Device events have a tracked device structure */
	rift_tracked_device_simulator *device;

	GQueue *events;

	/* Last time an output-pose event was seen */
	uint64_t last_output_pose;

	/* JSON output location */
	FILE *json_out;
};

static recording_simulator_stream_events *stream_events_new(
		recording_stream_type type, recording_simulator *simulator, const char *stream_name)
{
	recording_simulator_stream_events *sim_stream =
			calloc(1, sizeof(recording_simulator_stream_events));
	sim_stream->s.type = type;
	sim_stream->s.sim = simulator;
	sim_stream->s.stream_name = g_strdup (stream_name);

	sim_stream->events = g_queue_new();

	return sim_stream;
}

struct event_search_info {
	data_point_type event_type;
	const char *stream_name;
};

static gint find_sensor_event(recording_simulator_event *candidate, struct event_search_info *info)
{
	if (candidate->data.data_type != info->event_type)
		return 1;

	if (info->event_type == DATA_POINT_SENSOR_CONFIG) {
		if (g_str_equal(candidate->data.sensor_config.stream_id, info->stream_name))
			return 0;
	} else if (info->event_type == DATA_POINT_SENSOR_POSE) {
		if (strstr(info->stream_name, candidate->data.sensor_pose.serial_no) != NULL)
			return 0;
	} else {
		g_assert_not_reached();
	}

	return 1;
}

static struct recording_simulator_event *
stream_events_find_sensor_event(recording_simulator_stream_events *stream,
	data_point_type event_type, const char *stream_name)
{
	struct event_search_info info = {
		event_type, stream_name
	};

	GList *config = g_queue_find_custom(stream->events, &info, (GCompareFunc) find_sensor_event);
	if (config)
		return config->data;
	return NULL;
}

static void stream_events_free (recording_simulator_stream_events *stream) {
	recording_simulator_event *event;

	while ((event = g_queue_pop_head(stream->events)) != NULL) {
		g_free(event);
	}
	g_queue_free(stream->events);

	if (stream->json_out)
		fclose (stream->json_out);

	free(stream->s.stream_name);
	free(stream);
}

static recording_simulator_stream_video *stream_video_new (
	recording_stream_type type, recording_simulator *simulator, const char *stream_name)
{
	recording_simulator_stream_video *sim_stream =
			calloc(1, sizeof(recording_simulator_stream_video));
	sim_stream->s.type = type;
	sim_stream->s.sim = simulator;
	sim_stream->s.stream_name = g_strdup (stream_name);

	return sim_stream;
}

static void stream_video_free (recording_simulator_stream_video *stream) {
	free(stream->s.stream_name);
	free(stream);
}

static void handle_new_stream(void *cb_data, recording_loader_stream *stream,
		recording_stream_type type, const char *stream_name)
{
	recording_simulator *simulator = cb_data;

	switch (type) {
		case RECORDING_STREAM_TYPE_CAMERA_STREAM:
		{
			assert(simulator->n_sensors < RIFT_MAX_SENSORS);

			recording_simulator_stream_video *sim_stream =
					stream_video_new(type, simulator, stream_name);

			recording_loader_stream_set_cbdata(stream, sim_stream);

			/* If we're not doing a full simulation, disable decode
			 * of the video stream */
			if (!simulator->full_simulation)
				recording_loader_stream_set_enabled(stream, false);

			printf("Found camera stream %d: %s\n", simulator->n_sensors, stream_name);
			simulator->sensor_video_stream[simulator->n_sensors++] = sim_stream;
			break;
		}
		case RECORDING_STREAM_TYPE_GLOBAL_METADATA:
		{
			assert(simulator->global_metadata == NULL);

			recording_simulator_stream_events *sim_stream =
					stream_events_new(type, simulator, stream_name);

			recording_loader_stream_set_cbdata(stream, sim_stream);

			printf("Found Global metadata event stream\n");
			simulator->global_metadata = sim_stream;
			break;
		}
		case RECORDING_STREAM_TYPE_DEVICE_METADATA:
		{
			assert(simulator->n_devices < RIFT_MAX_TRACKED_DEVICES);

			recording_simulator_stream_events *sim_stream =
					stream_events_new(type, simulator, stream_name);

			recording_loader_stream_set_cbdata(stream, sim_stream);

			printf("Found event stream %d: %s\n", simulator->n_devices, stream_name);
			simulator->tracked_device_event_stream[simulator->n_devices++] = sim_stream;
			break;
		}
	}
}

static void
dump_output_pose (recording_simulator_stream_events *sim_stream, bool force, uint64_t local_ts)
{
	if (local_ts - sim_stream->last_output_pose < 10000000UL && force == false)
		return;

	if (sim_stream->json_out == NULL)
		return;

	rift_tracked_device_simulator *dev = sim_stream->device;

	posef pose;
	vec3f ang_vel, lin_accel, lin_vel;

	rift_tracked_device_simulator_get_model_pose(sim_stream->device, local_ts,
		&pose, &lin_vel, &lin_accel, &ang_vel);

	sim_stream->last_output_pose = local_ts;

	fprintf (sim_stream->json_out, "{ \"type\": \"output-pose\", \"local-ts\": %llu, "
		"\"device-ts\": %llu, \"last-imu-local-ts\": %llu, "
		"\"pos\" : [ %f, %f, %f ], "
		"\"orient\" : [ %f, %f, %f, %f ], "
		"\"ang-vel\" : [ %f, %f, %f ], "
		"\"lin-vel\" : [ %f, %f, %f ], "
		"\"lin-accel\" : [ %f, %f, %f ] "
		"}\n",
		(unsigned long long) local_ts,
		(unsigned long long) dev->device_time_ns,
		(unsigned long long) dev->last_imu_local_ts,
		pose.pos.x, pose.pos.y, pose.pos.z,

		pose.orient.x, pose.orient.y,
		pose.orient.z, pose.orient.w,

		ang_vel.x, ang_vel.y, ang_vel.z,

		lin_vel.x, lin_vel.y, lin_vel.z,
		lin_accel.x, lin_accel.y, lin_accel.z
	);
}

static void
dump_pose_report (recording_simulator_stream_events *sim_stream,
	struct data_point *data, posef *pose_after)
{
	if (sim_stream->json_out == NULL)
		return;

	fprintf(sim_stream->json_out,
		"{ \"type\": \"pose\", \"local-ts\": %llu, "
		"\"device-ts\": %llu, \"frame-start-local-ts\": %llu, "
		"\"frame-local-ts\": %llu, \"frame-hmd-ts\": %u, "
		"\"frame-exposure-count\": %u, \"frame-device-ts\": %llu, \"frame-fusion-slot\": %d, "
		"\"source\": \"%s\", "
		"\"score-flags\": %d, \"update-position\": %d, \"update-orient\": %d, "
		"\"pos\" : [ %f, %f, %f ], "
		"\"orient\" : [ %f, %f, %f, %f ], "
		"\"capture-pos\" : [ %f, %f, %f ], "
		"\"capture-orient\" : [ %f, %f, %f, %f ], "
		"\"posterior-pos\" : [ %f, %f, %f ], "
		"\"posterior-orient\" : [ %f, %f, %f, %f ], "
		"\"rot-std-dev\" : [ %f, %f, %f ], "
		"\"pos-std-dev\" : [ %f, %f, %f ] "
		"}\n",

		(unsigned long long) data->pose.local_ts,
		(unsigned long long) data->pose.device_ts,
		(unsigned long long) data->pose.frame_start_local_ts,
		(unsigned long long) data->pose.exposure_local_ts,
		data->pose.exposure_hmd_ts,
		data->pose.exposure_count,
		(unsigned long long) data->pose.frame_device_ts,
		data->pose.delay_slot,
		data->pose.serial_no,
		data->pose.score_flags, data->pose.update_position, data->pose.update_orientation,

		data->pose.pose.pos.x, data->pose.pose.pos.y, data->pose.pose.pos.z,
		data->pose.pose.orient.x, data->pose.pose.orient.y, data->pose.pose.orient.z, data->pose.pose.orient.w,

		data->pose.capture_pose.pos.x, data->pose.capture_pose.pos.y, data->pose.capture_pose.pos.z,
		data->pose.capture_pose.orient.x, data->pose.capture_pose.orient.y,
		data->pose.capture_pose.orient.z, data->pose.capture_pose.orient.w,

		pose_after->pos.x, pose_after->pos.y, pose_after->pos.z,
		pose_after->orient.x, pose_after->orient.y,
		pose_after->orient.z, pose_after->orient.w,

		data->pose.capture_rot_error.x, data->pose.capture_rot_error.y, data->pose.capture_rot_error.z,
		data->pose.capture_pos_error.x, data->pose.capture_pos_error.y, data->pose.capture_pos_error.z
	);
}

static void dump_imu_report (recording_simulator_stream_events *sim_stream,
	struct data_point *data,
	posef *pose_before, vec3f *lin_vel_before, vec3f *lin_accel_before,
	posef *pose_after, vec3f *lin_vel_after, vec3f *lin_accel_after)
{
	if (sim_stream->json_out == NULL)
		return;

	fprintf(sim_stream->json_out,
		"{ \"type\": \"imu\", \"local-ts\": %llu, "
		"\"device-ts\": %llu, \"dt\": %f, "
		"\"ang_vel\": [ %f, %f, %f ], \"accel\": [ %f, %f, %f ], "
		"\"mag\": [ %f, %f, %f ], "
		"\"pose-before\": { "
		  "\"pos\" : [ %f, %f, %f ], "
		  "\"orient\" : [ %f, %f, %f, %f ] "
		" }, "
		"\"lin-vel-before\": [ %f, %f, %f ], "
		"\"lin-accel-before\": [ %f, %f, %f ], "
		"\"pose-after\": { "
		  "\"pos\" : [ %f, %f, %f ], "
		  "\"orient\" : [ %f, %f, %f, %f ] "
		" }, "
		"\"lin-vel-after\": [ %f, %f, %f ], "
		"\"lin-accel-after\": [ %f, %f, %f ] "
		" }\n",
		(unsigned long long) data->imu.local_ts,
		(unsigned long long) data->imu.device_ts,
		data->imu.dt,
		data->imu.ang_vel.x, data->imu.ang_vel.y, data->imu.ang_vel.z,
		data->imu.accel.x, data->imu.accel.y, data->imu.accel.z,
		data->imu.mag.x, data->imu.mag.y, data->imu.mag.z,

		pose_before->pos.x, pose_before->pos.y, pose_before->pos.z,
		pose_before->orient.x, pose_before->orient.y,
		pose_before->orient.z, pose_before->orient.w,

		lin_vel_before->x, lin_vel_before->y, lin_vel_before->z,
		lin_accel_before->x, lin_accel_before->y, lin_accel_before->z,

		pose_after->pos.x, pose_after->pos.y, pose_after->pos.z,
		pose_after->orient.x, pose_after->orient.y,
		pose_after->orient.z, pose_after->orient.w,

		lin_vel_after->x, lin_vel_after->y, lin_vel_after->z,
		lin_accel_after->x, lin_accel_after->y, lin_accel_after->z
	);
}

static void handle_on_event(void *cb_data, recording_loader_stream *stream, uint64_t pts,
		struct data_point *data, const char *json_data)
{
	recording_simulator *simulator = cb_data;
	recording_simulator_stream_events *sim_stream = recording_loader_stream_get_cbdata(stream);
	assert (sim_stream != NULL);
	bool print_json = true;

#if 0
	printf("Got event PTS %" G_GUINT64_FORMAT " on %s type %s: %s\n",
	    pts, sim_stream->s.stream_name,
	    data_point_type_names[data->data_type], json_data);
#endif

	/* Simulator event - either global (camera configs) or tracked-device tracking */
	recording_simulator_event *event = malloc(sizeof(recording_simulator_event));
	event->pts = pts;
	event->data = *data;

	g_queue_push_tail(sim_stream->events, event);

	switch (data->data_type) {
		case DATA_POINT_DEVICE_ID:
			/* Set up the tracked device */
			if (sim_stream->device != NULL) {
				g_printerr ("Stream %s has repeated device-id record\n", sim_stream->s.stream_name);
				break;
			}

			sim_stream->device = rift_tracked_device_simulator_new(data->device_id.device_id,
							&data->device_id.imu_calibration, &data->device_id.imu_pose,
							&data->device_id.model_pose,
							data->device_id.num_leds, data->device_id.leds);
			if (simulator->json_output_dir != NULL && sim_stream->json_out == NULL) {
				gchar *json_out_location = g_strdup_printf ("%s/openhmd-device-%d",
				                             simulator->json_output_dir, data->device_id.device_id);

				printf("Opening output file %s for device %d\n",
				    json_out_location, data->device_id.device_id);

				sim_stream->json_out = fopen(json_out_location, "w");
				g_free (json_out_location);
			}
			break;

		case DATA_POINT_SENSOR_CONFIG:
		case DATA_POINT_SENSOR_POSE:
			/* Just store these for the video streams to collect */
			break;

		case DATA_POINT_IMU: {

			posef pose_before, pose_after;
			vec3f lin_vel_before, lin_accel_before;
			vec3f lin_vel_after, lin_accel_after;

			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
				break;
			}

			rift_tracked_device_simulator_get_model_pose(sim_stream->device, sim_stream->device->device_time_ns,
				&pose_before, &lin_vel_before, &lin_accel_before, NULL);

			rift_tracked_device_simulator_imu_update(sim_stream->device,
					data->imu.local_ts, data->imu.device_ts,
					&data->imu.ang_vel, &data->imu.accel, &data->imu.mag);

			rift_tracked_device_simulator_get_model_pose(sim_stream->device, sim_stream->device->device_time_ns,
				&pose_after, &lin_vel_after, &lin_accel_after, NULL);

			/* Replace with updated pose report */
			dump_imu_report (sim_stream, data,
				&pose_before, &lin_vel_before, &lin_accel_before,
				&pose_after, &lin_vel_after, &lin_accel_after);
			print_json = false;

			dump_output_pose (sim_stream, false, data->imu.local_ts);
			break;
		}
		case DATA_POINT_EXPOSURE:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device ID record\n", sim_stream->s.stream_name);
				break;
			}
			rift_tracked_device_simulator_on_exposure (sim_stream->device, data->exposure.local_ts,
				data->exposure.device_ts, data->exposure.exposure_ts, data->exposure.delay_slot);
			break;
		case DATA_POINT_POSE:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
				break;
			}

			rift_tracked_device_simulator_model_pose_update(sim_stream->device,
					data->pose.local_ts, data->pose.frame_device_ts, data->pose.frame_start_local_ts,
					data->pose.delay_slot,
					data->pose.score_flags, data->pose.update_position, data->pose.update_orientation,
					&data->pose.pose, data->pose.serial_no);

			posef pose_after;

			rift_tracked_device_simulator_get_model_pose(sim_stream->device, sim_stream->device->device_time_ns,
				&pose_after, NULL, NULL, NULL);

			/* Replace with updated pose report */
			dump_pose_report (sim_stream, data, &pose_after);
			print_json = false;

			dump_output_pose (sim_stream, false, data->pose.local_ts);
			break;
		case DATA_POINT_OUTPUT_POSE:
			if (sim_stream->device == NULL)
				break;

			dump_output_pose (sim_stream, true, data->output_pose.local_ts);
			print_json = false;
			break;
		case DATA_POINT_FRAME_START:
			break;
		case DATA_POINT_FRAME_CAPTURED:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
				break;
			}
			rift_tracked_device_simulator_frame_captured(sim_stream->device, data->frame_captured.local_ts,
				data->frame_captured.frame_local_ts, data->frame_captured.delay_slot,
				data->frame_captured.serial_no);
			break;
		case DATA_POINT_FRAME_RELEASE:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
				break;
			}
			rift_tracked_device_simulator_frame_release(sim_stream->device, data->frame_release.local_ts,
				data->frame_release.frame_local_ts, data->frame_release.delay_slot,
				data->frame_release.serial_no);
			break;
	}

	if (print_json) {
		if (sim_stream->json_out != NULL)
			fprintf(sim_stream->json_out, "%s\n", json_data);
	}
}

static void on_frame_config(void *cb_data, recording_loader_stream *stream, int width, int height, int stride)
{
	recording_simulator_stream_video *sim_stream = recording_loader_stream_get_cbdata(stream);
	assert (sim_stream != NULL);

	sim_stream->have_stream_config = true;
	sim_stream->width = width;
	sim_stream->height = height;
	sim_stream->stride = stride;

	printf ("Got frame size for camera stream %s width %d height %d stride %d\n", sim_stream->s.stream_name,
			width, height, stride);
}

static void handle_video_frame(void *cb_data, recording_loader_stream *stream, uint64_t pts, unsigned char *frame_data, size_t frame_len)
{
	recording_simulator *simulator = cb_data;
	recording_simulator_stream_video *sim_stream = recording_loader_stream_get_cbdata(stream);
	assert (sim_stream != NULL);
	assert (simulator->full_simulation);

	if (!sim_stream->have_stream_config) {
		g_printerr ("Received video frame for stream %s before config\n", sim_stream->s.stream_name);
		return;
	}

	if (!sim_stream->have_calibration) {
		assert(simulator->global_metadata != NULL);
		struct recording_simulator_event *calib_event =
				stream_events_find_sensor_event (simulator->global_metadata, DATA_POINT_SENSOR_CONFIG, sim_stream->s.stream_name);

		if (calib_event == NULL) {
			g_printerr ("Received video frame for stream %s before config\n", sim_stream->s.stream_name);
			return;
		}

		struct data_point *calib = &calib_event->data;

		memcpy(sim_stream->serial_no, calib->sensor_config.serial_no, RIFT_SENSOR_SERIAL_LEN+1);
		sim_stream->calibration.is_cv1 = calib->sensor_config.is_cv1;
		sim_stream->calibration.camera_matrix = calib->sensor_config.camera_matrix;

		memcpy(sim_stream->calibration.dist_coeffs, calib->sensor_config.dist_coeffs,
				sizeof(calib->sensor_config.dist_coeffs));

		/* Check the video width/height makes sense and set it into the calibration width/height */
		if (calib->sensor_config.is_cv1) {
			assert (sim_stream->width == 1280);
			assert (sim_stream->height == 960);
		}

		sim_stream->calibration.width = sim_stream->width;
		sim_stream->calibration.height = sim_stream->height;

		printf ("Got calibration for camera stream %s\n", sim_stream->s.stream_name);

		sim_stream->have_calibration = true;
	}

	if (!sim_stream->have_pose) {
		assert(simulator->global_metadata != NULL);
		struct recording_simulator_event *pose_event =
			stream_events_find_sensor_event (simulator->global_metadata,
			    DATA_POINT_SENSOR_POSE, sim_stream->s.stream_name);

		if (pose_event == NULL) {
			// g_printerr ("Received video frame for stream %s before sensor pose\n", sim_stream->s.stream_name);
			return;
		}

		struct data_point *pose = &pose_event->data;

		sim_stream->pose = pose->sensor_pose.pose;

		printf("Got pose for camera stream %s. Position %f %f %f, orientation %f %f %f %f\n",
			sim_stream->s.stream_name,
			sim_stream->pose.pos.x, sim_stream->pose.pos.y, sim_stream->pose.pos.z,
			sim_stream->pose.orient.x, sim_stream->pose.orient.y,
			sim_stream->pose.orient.z, sim_stream->pose.orient.w
		);

		sim_stream->have_pose = true;
	}

	/* We have calibration and pose, create the sensor simulator now if needed */
	if (sim_stream->sensor == NULL) {
		sim_stream->sensor = recording_simulator_sensor_new(sim_stream->serial_no,
		    &sim_stream->calibration, &sim_stream->pose);
	}

	/* And process the frame */
#if 1
	printf("Got video frame for sensor %s PTS %" G_GUINT64_FORMAT "\n",
	    sim_stream->serial_no, pts);
#endif

	assert (frame_len == sim_stream->stride * sim_stream->height);

	ohmd_video_frame *frame = calloc(1, sizeof(ohmd_video_frame));

	frame->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
	frame->pts = frame->start_ts = pts;
	frame->data = frame_data;
	frame->data_block_size = frame->data_size = frame_len;
	frame->stride = sim_stream->stride;
	frame->width = sim_stream->width;
	frame->height = sim_stream->height;

	recording_simulator_sensor_process_frame(sim_stream->sensor, frame);
}

recording_simulator *recording_simulator_new(const char *json_output_dir, bool full_simulation)
{
	if (!recording_loader_init())
		return NULL;

	recording_simulator *sim = calloc(1, sizeof(recording_simulator));

	recording_loader_callbacks callbacks = {
		.new_stream = handle_new_stream,
		.on_json_data = NULL,
		.on_event = handle_on_event,
		.on_encoded_frame = NULL,
		.on_frame_config = on_frame_config,
		.on_video_frame = handle_video_frame,
	};

	sim->loader = recording_loader_new(&callbacks, sim);
	if (sim->loader == NULL) {
		printf("Could not create file loader\n");
		goto fail;
	}

	if (json_output_dir)
		sim->json_output_dir = g_strdup(json_output_dir);

	sim->full_simulation = full_simulation;

	return sim;

fail:
	recording_simulator_free(sim);
	return NULL;
}

bool recording_simulator_load(recording_simulator *sim, const char *filename_or_uri)
{
	return recording_loader_load(sim->loader, filename_or_uri);
}

void recording_simulator_free(recording_simulator *sim)
{
	int i;

	if (sim->loader)
		recording_loader_free(sim->loader);

	g_free (sim->json_output_dir);

	if (sim->global_metadata)
		stream_events_free(sim->global_metadata);

	for (i = 0; i < sim->n_devices; i++)
		stream_events_free(sim->tracked_device_event_stream[i]);

	for (i = 0; i < sim->n_sensors; i++)
		stream_video_free(sim->sensor_video_stream[i]);

	free(sim);
}

