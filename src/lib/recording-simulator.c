#include <assert.h>
#include <glib.h>

#include "recording-simulator.h"
#include "recording-loader.h"
#include "recording-device-simulator.h"
#include "recording-sensor-simulator.h"

#include "correspondence_search.h"
#include "rift-tracker-common.h"
#include "rift-sensor-blobwatch.h"

typedef struct recording_simulator_stream recording_simulator_stream;
typedef struct recording_simulator_stream_video recording_simulator_stream_video;
typedef struct recording_simulator_event recording_simulator_event;
typedef struct recording_simulator_stream_events recording_simulator_stream_events;

struct recording_simulator {
	recording_loader *loader;

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

	ohmd_video_frame frame;

	blobwatch* bw;
	correspondence_search_t *cs;
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

	rift_tracked_device_simulator *dev = sim_stream->device;

	posef pose;
	vec3f ang_vel, lin_accel, lin_vel;

	rift_tracked_device_simulator_get_model_pose(sim_stream->device, local_ts,
		&pose, &lin_vel, &lin_accel, &ang_vel);

	sim_stream->last_output_pose = local_ts;

	printf ("{ \"type\": \"output-pose\", \"local-ts\": %llu, "
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

static void handle_on_event(void *cb_data, recording_loader_stream *stream, uint64_t pts,
		struct data_point *data, const char *json_data)
{
	recording_simulator_stream_events *sim_stream = recording_loader_stream_get_cbdata(stream);
	assert (sim_stream != NULL);

#if 0
	printf("Got event on %s type %s: %s\n", sim_stream->s.stream_name,
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
			break;

		case DATA_POINT_SENSOR_CONFIG:
		case DATA_POINT_SENSOR_POSE:
			/* Just store these for the video streams to collect */
			break;

		case DATA_POINT_IMU:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
				break;
			}
			rift_tracked_device_simulator_imu_update(sim_stream->device,
					data->imu.local_ts, data->imu.device_ts,
					&data->imu.ang_vel, &data->imu.accel, &data->imu.mag);

			dump_output_pose (sim_stream, false, data->imu.local_ts);
			break;
		case DATA_POINT_EXPOSURE:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
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

			dump_output_pose (sim_stream, false, data->pose.local_ts);
			break;
		case DATA_POINT_OUTPUT_POSE:
			if (sim_stream->device == NULL)
				break;

			dump_output_pose (sim_stream, true, data->output_pose.local_ts);
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

		/* FIXME: Set calibration width/height */

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

	/* Raw camera frame - do blob analysis */
	if (sim_stream->bw == NULL) {
		sim_stream->bw = blobwatch_new(sim_stream->calibration.is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2);
	}

	blobservation* bwobs = NULL;

	blobwatch_process(sim_stream->bw, frame_data, sim_stream->width, sim_stream->height, 0, NULL, 0, &bwobs);

	if (bwobs) {
		blobwatch_release_observation(sim_stream->bw, bwobs);
	}
}

recording_simulator *recording_simulator_new()
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

	if (sim->global_metadata)
		stream_events_free(sim->global_metadata);

	for (i = 0; i < sim->n_devices; i++)
		stream_events_free(sim->tracked_device_event_stream[i]);

	for (i = 0; i < sim->n_sensors; i++)
		stream_video_free(sim->sensor_video_stream[i]);

	free(sim);
}

