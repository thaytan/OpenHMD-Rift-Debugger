#include <assert.h>
#include <inttypes.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
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
typedef struct recording_simulator_video_frame recording_simulator_video_frame;
typedef struct recording_simulator_event recording_simulator_event;
typedef struct recording_simulator_stream_events recording_simulator_stream_events;

static void
simulator_update_tracked_devices (recording_simulator *sim);
static bool
simulator_update_sensor_calibration (recording_simulator *sim, recording_simulator_stream_video *sim_stream);
static bool
simulator_update_sensor_pose (recording_simulator *sim, recording_simulator_stream_video *sim_stream);
static recording_simulator_stream_video *
recording_simulator_find_sensor_stream (recording_simulator *simulator, const char *sensor_id);

static bool get_exposure_info (recording_simulator *ctx, uint64_t frame_start_local_ts, rift_tracker_exposure_info *info);

/* Number of exposure history slots to keep */
#define NUM_EXPOSURE_HISTORY 3

struct recording_simulator_video_frame {
	ohmd_video_frame frame;
	unsigned char frame_data[];
};

struct recording_simulator {
	recording_loader *loader;

	gchar *json_output_dir;
	bool full_simulation;

	int exposure_history_index;
	int exposure_history_size;
	rift_tracker_exposure_info exposure_history[NUM_EXPOSURE_HISTORY];

	int n_sensors;
	recording_simulator_stream_video *sensor_video_stream[RIFT_MAX_SENSORS];

	recording_simulator_stream_events *global_metadata;

	int n_devices;
	recording_simulator_stream_events *tracked_device_event_stream[RIFT_MAX_TRACKED_DEVICES];

	int n_tracked_devices;
	rift_tracked_device *tracked_devices[RIFT_MAX_TRACKED_DEVICES];
};

struct recording_simulator_stream {
	recording_stream_type type;
	recording_simulator *sim;
	char *stream_name;
};

struct recording_simulator_stream_video {
	recording_simulator_stream s;

	int id;

	bool have_stream_config;
	int width, height, stride;

	bool have_calibration;
	char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
	rift_sensor_camera_params calibration;

	bool have_pose;
	posef pose;
	bool have_video_before_pose;

	recording_simulator_sensor *sensor;

	int n_pending_frames;
	int n_pending_frames_size;
	recording_simulator_video_frame **pending_frames;
	uint64_t last_frame_local_ts;
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

	if (stream->device)
		rift_tracked_device_simulator_free(stream->device);
	free(stream->s.stream_name);
	free(stream);
}

static recording_simulator_stream_video *stream_video_new (
	recording_stream_type type, recording_simulator *simulator, int id, const char *stream_name)
{
	recording_simulator_stream_video *sim_stream =
			calloc(1, sizeof(recording_simulator_stream_video));
	sim_stream->s.type = type;
	sim_stream->s.sim = simulator;
	sim_stream->s.stream_name = g_strdup (stream_name);
	sim_stream->pose.orient.w = 1.0;
	sim_stream->id = id;

	return sim_stream;
}

static void stream_video_store_pending_frame (recording_simulator_stream_video *stream,
    recording_simulator_video_frame *frame)
{
	if (stream->n_pending_frames >= stream->n_pending_frames_size) {
		stream->n_pending_frames_size += 8;
		stream->pending_frames = realloc (stream->pending_frames, stream->n_pending_frames_size * sizeof (recording_simulator_video_frame *));
	}

	stream->pending_frames[stream->n_pending_frames] = frame;
	stream->n_pending_frames++;
}

static void stream_video_free (recording_simulator_stream_video *stream) {
	int i;

	if (stream->pending_frames) {
		for (i = 0; i < stream->n_pending_frames; i++) {
			free(stream->pending_frames[i]);
		}
		free(stream->pending_frames);
	}

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
					stream_video_new(type, simulator, simulator->n_sensors, stream_name);

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

static bool get_exposure_info (recording_simulator *ctx,
		uint64_t frame_start_local_ts, rift_tracker_exposure_info *out_info)
{
	int i;

	/* Find and populate the exposure info and return true if found */
	rift_tracker_exposure_info *found_info = NULL;

	int64_t best_time_diff_ns = 0;

	for (i = 0; i < ctx->exposure_history_size; i++) {
		rift_tracker_exposure_info *info = ctx->exposure_history + i;
		int64_t time_diff_ns = frame_start_local_ts - info->local_ts;
		if (time_diff_ns > -10000000 && time_diff_ns < 10000000) {
			if (found_info == NULL || abs(time_diff_ns) < best_time_diff_ns) {
				found_info = info;
				best_time_diff_ns = abs(time_diff_ns);
			}
		}
	}

	if (found_info == NULL)
		return false;

	/* Check that we got exposure events for all the online devices,
	 * or else skip the frame */
	if (found_info->n_devices != ctx->n_tracked_devices) {
		printf("Skipping frame HMD TS %u due to missing exposure info for a device\n",
				found_info->hmd_ts);
		return false;
	}

	/* Copy to the output info, but put devices in the right index order,
	 * as events from each device can arrive in a different order in the recording */
	*out_info = *found_info;
	for (i = 0; i < found_info->n_devices; i++) {
		rift_tracked_device_exposure_info *exp_dev_info = found_info->devices + i;
		if (exp_dev_info->device_index != i) {
			printf("Frame HMD TS %u - reordering exposure info slot %d to %d\n",
					found_info->hmd_ts, i, exp_dev_info->device_index);
			out_info->devices[exp_dev_info->device_index] = *exp_dev_info;
		}
	}
	return true;
}

static void store_exposure_info (recording_simulator *ctx,
		uint64_t local_ts, uint32_t exposure_ts, uint32_t exposure_count,
		rift_tracked_device_simulator *dev, int delay_slot)
{
	rift_tracker_exposure_info *info = NULL;
	bool is_new_exposure = true;

	if (ctx->exposure_history_size > 0) {
		if (ctx->exposure_history[ctx->exposure_history_index].count != exposure_count) {
			ctx->exposure_history_index = (ctx->exposure_history_index + 1) % NUM_EXPOSURE_HISTORY;

			if (ctx->exposure_history_size < NUM_EXPOSURE_HISTORY)
				ctx->exposure_history_size++;
		}
		else {
			is_new_exposure = false;
		}
	}
	else {
		ctx->exposure_history_index = 0;
		ctx->exposure_history_size = 1;
	}

	info = ctx->exposure_history + ctx->exposure_history_index;
	if (is_new_exposure) {
		info->local_ts = local_ts;
		info->count = exposure_count;
		info->hmd_ts = exposure_ts;
		info->led_pattern_phase = 0;

		info->n_devices = 0;
	}
	else {
		assert (info->hmd_ts == exposure_ts);
	}

	assert (info->n_devices < NUM_EXPOSURE_HISTORY);

	rift_tracked_device_exposure_info *dev_info = &info->devices[info->n_devices];
	dev_info->device_index = dev->index;

	rift_tracked_device_simulator_get_exposure_info (dev, delay_slot, dev_info);

	printf("Exposure entry %d slot %d: stored device %d HMD TS %u"
	    " delay slot %d device_ts %" PRIu64 "\n",
	    ctx->exposure_history_index, info->n_devices,
	    dev->base.id, exposure_ts, delay_slot,
	    dev_info->device_time_ns);

	info->n_devices++;
}

static void simulator_process_pending_video_frame (recording_simulator *sim,
	const char *sensor_serial_no, uint64_t frame_local_ts, uint64_t frame_pts)
{
	int i;

	/* Look up the sensor by serial no */
	recording_simulator_stream_video *sim_stream = NULL;
	for (i = 0; i < sim->n_sensors; i++) {
		recording_simulator_stream_video *cur= sim->sensor_video_stream[i];
		if (strcmp (cur->serial_no, sensor_serial_no))
			continue;
		sim_stream = cur;
		break;
	}

	if (sim_stream == NULL) {
		g_printerr ("Sensor %s not found, but we received a frame_captured event!\n", sensor_serial_no);
		return;
	}

	/* Not having any pending frames can just mean we already processed the target
	 * frame */
	if (sim_stream->pending_frames == NULL || sim_stream->n_pending_frames < 1)
		return;

	int removed_frames = 0;
	for (i = 0; i < sim_stream->n_pending_frames; i++) {
		recording_simulator_video_frame *sim_frame = sim_stream->pending_frames[i];
		int64_t time_diff = sim_frame->frame.pts - frame_pts;
		if (time_diff < -5000000) {
			/* This frame is too far in the past - this is not our frame */
			printf("Sensor %s - Skipping pending frame with PTS %" PRIu64
			    " (late by %fms) - missing frame-captured events\n",
			    sensor_serial_no, sim_frame->frame.pts, (double) (-time_diff) / 1000000.0);
			free (sim_frame);
			removed_frames++;
			continue;
		}
		if (time_diff > 5000000) {
			/* This frame is in the future. Ignore it */
			printf("Sensor %s - Leaving pending frame with PTS %" PRIu64
			    " - too far in the future %" PRIu64 "\n",
			    sensor_serial_no, sim_frame->frame.pts, time_diff);
			break;
		}

		sim_frame->frame.start_ts = frame_local_ts;

		rift_tracker_exposure_info exp_info = { 0, };

		bool res = get_exposure_info(sim, frame_local_ts, &exp_info);
		if (!res) {
			printf("Sensor %s - Missing exposure/capture events for video frame PTS %" G_GUINT64_FORMAT
			    " last frame local_ts %" G_GUINT64_FORMAT "\n", sim_stream->serial_no,
			    sim_frame->frame.pts, frame_local_ts);
			return;
		}

#if 0
		printf("Sensor %s - Matched frame-captured event (frame-local-ts %" PRIu64
		    ", PTS %" PRIu64 ") to frame PTS %" PRIu64
				" - exposure local ts %" PRIu64 " (delta %" PRId64
				") count %u\n", sensor_serial_no,
				frame_local_ts, frame_pts, sim_frame->frame.pts,
				exp_info.local_ts, (frame_local_ts - exp_info.local_ts), exp_info.count);
#endif

		recording_simulator_sensor_process_frame(sim_stream->sensor, &sim_frame->frame, &exp_info);

		removed_frames++;
		free (sim_frame);
		break;
	}

	if (removed_frames > 0) {
		sim_stream->n_pending_frames -= removed_frames;

		size_t bytes_to_remove = sizeof (recording_simulator_video_frame *) * removed_frames;
		size_t bytes_remain = sizeof (recording_simulator_video_frame *) * sim_stream->n_pending_frames;

		memmove (sim_stream->pending_frames, sim_stream->pending_frames + bytes_to_remove, bytes_remain);
	}
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

			simulator_update_tracked_devices(simulator);
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

			/* FIXME: During a full replay we could ignore the recording's delay slot
			 * assignments and make our own */
			if (data->exposure.delay_slot != -1) {
				store_exposure_info(simulator, data->exposure.local_ts, data->exposure.exposure_ts, data->exposure.exposure_count,
				    sim_stream->device, data->exposure.delay_slot);
			}
			break;
		case DATA_POINT_POSE:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has imu data before device record\n", sim_stream->s.stream_name);
				break;
			}

			g_print ("Pose observation on stream %s from sensor %s\n",
			    sim_stream->s.stream_name, data->pose.serial_no);

			/* Pose observations from the stream are dropped for now when runnning a
			 * full simulation. Later we could store them for display / comparison
			 * against the new code */
			if (simulator->full_simulation)
				break;

			/* Compute an error of 1cm at right angles to the camera, and 3cm in Z.
			 * FIXME: Compute based on pixel error and distance from camera and pass
			 * it in */
			vec3f cam_obs_pos_error = {{ 0.01, 0.01, 0.03 }};
			vec3f world_obs_pos_error;

			recording_simulator_stream_video *cam_sim_stream =
			    recording_simulator_find_sensor_stream (simulator, data->pose.serial_no);
			if (cam_sim_stream != NULL) {
				quatf *cam_orient = &cam_sim_stream->pose.orient;
				oquatf_get_rotated(cam_orient, &cam_obs_pos_error, &world_obs_pos_error);
			}
			else {
				world_obs_pos_error = cam_obs_pos_error;
			}

			rift_tracked_device_simulator_model_pose_update_reference(sim_stream->device,
					data->pose.local_ts, data->pose.frame_device_ts, data->pose.frame_start_local_ts,
					data->pose.delay_slot,
					data->pose.score_flags, data->pose.update_position, data->pose.update_orientation,
					&data->pose.pose, &world_obs_pos_error, data->pose.serial_no);

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
				g_printerr ("Stream %s has telemetry data before device record\n", sim_stream->s.stream_name);
				break;
			}

			rift_tracked_device_simulator_frame_captured(sim_stream->device, data->frame_captured.local_ts,
				data->frame_captured.frame_local_ts, data->frame_captured.delay_slot,
				data->frame_captured.serial_no);

			/* The expected PTS of the frame is calculated by subtracting the time between
			 * this event (local_ts) and the frame's start (frame_local_ts) from our PTS */
			uint64_t expected_pts = pts - (data->frame_captured.local_ts - data->frame_captured.frame_local_ts);
			simulator_process_pending_video_frame (simulator, data->frame_captured.serial_no, data->frame_captured.frame_local_ts, expected_pts);
			break;
		case DATA_POINT_FRAME_RELEASE:
			if (sim_stream->device == NULL) {
				g_printerr ("Stream %s has telemetry imu data before device record\n", sim_stream->s.stream_name);
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
		if (!simulator_update_sensor_calibration (simulator, sim_stream)) {
			g_printerr ("Received video frame for stream %s before config\n", sim_stream->s.stream_name);
			return;
		}
	}

	if (!sim_stream->have_pose) {
		if (!simulator_update_sensor_pose (simulator, sim_stream)) {
			if (!sim_stream->have_video_before_pose) {
				g_printerr ("Received video frame for stream %s before pose info\n", sim_stream->s.stream_name);
				sim_stream->have_video_before_pose = true;
			}
		}
	}

	/* We have calibration and pose, create the sensor simulator now if needed */
	if (sim_stream->sensor == NULL) {
		posef *pose = NULL;
		if (sim_stream->have_pose)
			pose = &sim_stream->pose;

		sim_stream->sensor = recording_simulator_sensor_new(sim_stream->id, sim_stream->serial_no,
		    &sim_stream->calibration, pose);

		simulator_update_tracked_devices(simulator);
	}

	/* Store the frame for processing when the CAPTURED events come in on this sensor.
	 * This is a bit annoying, but happens because the only information in the recordings
	 * about each video frame is that timestamp, which is set based on the time the frame
	 * started arriving from USB, which then needs to be matched to the closest EXPOSURE
	 * event */

#if 0
	printf("Got video frame for sensor %s PTS %" G_GUINT64_FORMAT "\n",
	    sim_stream->serial_no, pts);
#endif
	assert (frame_len == sim_stream->stride * sim_stream->height);

	recording_simulator_video_frame *sim_frame = malloc (sizeof (recording_simulator_video_frame) + frame_len);
	ohmd_video_frame *frame = (ohmd_video_frame *) sim_frame;

	frame->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
	frame->pts = pts;
	frame->start_ts = -1;
	frame->data = sim_frame->frame_data;
	frame->data_block_size = frame->data_size = frame_len;
	frame->stride = sim_stream->stride;
	frame->width = sim_stream->width;
	frame->height = sim_stream->height;

	memcpy(sim_frame->frame_data, frame_data, frame_len);

	stream_video_store_pending_frame (sim_stream, sim_frame);
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

static void
simulator_update_tracked_devices (recording_simulator *sim)
{
	int i;

	sim->n_tracked_devices = 0;
	for (i = 0; i < sim->n_devices; i++) {
		if (sim->tracked_device_event_stream[i]->device == NULL)
			continue;

		rift_tracked_device_simulator *dev = sim->tracked_device_event_stream[i]->device;
		rift_tracked_device_simulator_set_index (dev, i);

		sim->tracked_devices[i] = (rift_tracked_device *) dev;
		sim->n_tracked_devices++;
	}

	for (i = 0; i < sim->n_sensors; i++) {
		recording_simulator_stream_video *sim_stream = sim->sensor_video_stream[i];
		if (sim_stream->sensor != NULL) {
			recording_simulator_sensor_set_devices(sim_stream->sensor, sim->tracked_devices, sim->n_tracked_devices);
		}
	}
}

static bool
simulator_update_sensor_calibration (recording_simulator *sim, recording_simulator_stream_video *sim_stream)
{
	if (sim_stream->have_calibration)
		return true;

	assert(sim->global_metadata != NULL);
	struct recording_simulator_event *calib_event =
			stream_events_find_sensor_event (sim->global_metadata, DATA_POINT_SENSOR_CONFIG, sim_stream->s.stream_name);

	if (calib_event == NULL) {
		return false;
	}

	struct data_point *calib = &calib_event->data;

	memcpy(sim_stream->serial_no, calib->sensor_config.serial_no, RIFT_SENSOR_SERIAL_LEN+1);
	sim_stream->calibration.dist_fisheye = sim_stream->calibration.is_cv1 = calib->sensor_config.is_cv1;

	memcpy(&sim_stream->calibration.camera_matrix, &calib->sensor_config.camera_matrix,
			sizeof(calib->sensor_config.camera_matrix));
	memcpy(sim_stream->calibration.dist_coeffs, calib->sensor_config.dist_coeffs,
			sizeof(calib->sensor_config.dist_coeffs));
	sim_stream->calibration.is_cv1 = calib->sensor_config.is_cv1;

	/* Check the video width/height makes sense and set it into the calibration width/height */
	if (calib->sensor_config.is_cv1) {
		sim_stream->calibration.width = 1280;
		sim_stream->calibration.height = 960;
	} else {
		sim_stream->calibration.width = 640;
		sim_stream->calibration.height = 480;
	}

	printf ("Got calibration for camera stream %s\n", sim_stream->s.stream_name);

	sim_stream->have_calibration = true;
	return true;
}

static bool
simulator_update_sensor_pose (recording_simulator *sim, recording_simulator_stream_video *sim_stream)
{
	if (sim_stream->have_pose)
		return true;

	assert(sim->global_metadata != NULL);
	struct recording_simulator_event *pose_event =
		stream_events_find_sensor_event (sim->global_metadata, DATA_POINT_SENSOR_POSE, sim_stream->s.stream_name);

	if (pose_event != NULL) {
		struct data_point *pose = &pose_event->data;
		sim_stream->pose = pose->sensor_pose.pose;
	} else {
#if 0
		return false;
#else
		if (!strcmp (sim_stream->serial_no, "WMTD30333009TJ")) {
			printf("Using default pose for camera stream %s\n", sim_stream->serial_no);
			posef default_pose = {
				.orient = { { -0.965901, -0.103808, -0.227426, -0.067350 } },
				.pos = { { -0.984948, -0.361877, 1.573633 } }
			};

			sim_stream->pose = default_pose;
		}
		else {
			return false;
		}
#endif
	}

	printf("Got pose for camera stream %s. Position %f %f %f, orientation %f %f %f %f\n",
		sim_stream->s.stream_name,
		sim_stream->pose.pos.x, sim_stream->pose.pos.y, sim_stream->pose.pos.z,
		sim_stream->pose.orient.x, sim_stream->pose.orient.y,
		sim_stream->pose.orient.z, sim_stream->pose.orient.w
	);

	sim_stream->have_pose = true;

	if (sim_stream->sensor != NULL)
		recording_simulator_sensor_set_pose(sim_stream->sensor, &sim_stream->pose);

	return true;
}

static recording_simulator_stream_video *
recording_simulator_find_sensor_stream (recording_simulator *sim, const char *sensor_id)
{
	for (int i = 0; i < sim->n_sensors; i++) {
		recording_simulator_stream_video *sensor = sim->sensor_video_stream[i];

		if (!simulator_update_sensor_calibration (sim, sensor))
			continue;

		if (!simulator_update_sensor_pose (sim, sensor))
			continue;

		if (g_str_equal (sensor->serial_no, sensor_id))
			return sensor;
	}

	return NULL;
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

