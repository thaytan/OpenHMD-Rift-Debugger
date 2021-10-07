#include <assert.h>
#include <glib.h>

#include "recording-simulator.h"
#include "recording-loader.h"

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
	recording_simulator_stream_video *sensors[RIFT_MAX_SENSORS];

	recording_simulator_stream_events *global_metadata;

	int n_devices;
	recording_simulator_stream_events *tracked_devices[RIFT_MAX_TRACKED_DEVICES];
};

struct recording_simulator_stream {
	recording_stream_type type;
	recording_simulator *sim;
	char *stream_name;
};

struct recording_simulator_stream_video {
	recording_simulator_stream s;

	bool have_calibration;
  char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
  rift_sensor_camera_params calibration;

	blobwatch* bw;
	correspondence_search_t *cs;
};

struct recording_simulator_event {
	uint64_t pts;
	struct data_point data;
};

struct recording_simulator_stream_events {
	recording_simulator_stream s;

	GQueue *events;
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

static gint find_sensor_config(recording_simulator_event *candidate, const char *stream_name)
{
	if (candidate->data.data_type != DATA_POINT_SENSOR_CONFIG)
		return 1;

	if (g_str_equal(candidate->data.sensor_config.stream_id, stream_name))
		return 0;

	return 1;
}

static struct recording_simulator_event *
stream_events_find_sensor_config(recording_simulator_stream_events *stream,
	const char *stream_name)
{
	GList *config = g_queue_find_custom(stream->events, stream_name, (GCompareFunc) find_sensor_config);
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
			simulator->sensors[simulator->n_sensors++] = sim_stream;
			break;
		}
		case RECORDING_STREAM_TYPE_GLOBAL_METADATA:
		{
			assert(simulator->global_metadata == NULL);

			recording_simulator_stream_events *sim_stream =
					stream_events_new(type, simulator, stream_name);

			recording_loader_stream_set_cbdata(stream, sim_stream);

			printf("Found Global metadata event stream %s\n", stream_name);
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
			simulator->tracked_devices[simulator->n_devices++] = sim_stream;
			break;
		}
	}
}

static void handle_on_event(void *cb_data, recording_loader_stream *stream, uint64_t pts,
		struct data_point *data)
{
	recording_simulator *simulator = cb_data;
	recording_simulator_stream_events *sim_stream = recording_loader_stream_get_cbdata(stream);
	assert (sim_stream != NULL);

	printf("Got event on %s type %d\n", sim_stream->s.stream_name, data->data_type);

	/* Simulator event - either global (camera configs) or tracked-device tracking */
	recording_simulator_event *event = malloc(sizeof(recording_simulator_event));
	event->pts = pts;
	event->data = *data;

	g_queue_push_tail(sim_stream->events, event);
}

static void handle_video_frame(void *cb_data, recording_loader_stream *stream, uint64_t pts, unsigned char *frame_data, size_t frame_len)
{
	recording_simulator *simulator = cb_data;
	recording_simulator_stream_video *sim_stream = recording_loader_stream_get_cbdata(stream);
	assert (sim_stream != NULL);

	/* Raw camera frame - do blob analysis */
	if (!sim_stream->have_calibration) {
		assert(simulator->global_metadata != NULL);
		struct recording_simulator_event *calib_event = stream_events_find_sensor_config(simulator->global_metadata, sim_stream->s.stream_name);

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
		stream_events_free(sim->tracked_devices[i]);

	for (i = 0; i < sim->n_sensors; i++)
		stream_video_free(sim->sensors[i]);

	free(sim);
}

