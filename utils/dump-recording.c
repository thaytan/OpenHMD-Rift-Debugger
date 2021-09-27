#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "recording-loader.h"

static void handle_new_stream(void *cb_data, int stream_id, recording_stream_type type,
		const char *stream_name)
{
}

static void handle_on_event(void *cb_data, int stream_id, uint64_t pts, struct data_point *data)
{
}

static bool handle_on_encoded_frame(void *cb_data, int stream_id, uint64_t pts, unsigned char *data, size_t len)
{
	/* Don't decode */
	return false;
}

int
main (int argc, char *argv[])
{
	recording_loader *reader = NULL;
	recording_loader_callbacks callbacks = {
		.new_stream = handle_new_stream,
		.on_event = handle_on_event,
		.on_encoded_frame = handle_on_encoded_frame,
		.on_video_frame = NULL
	};

	/* Initialize GStreamer */
	if (!recording_loader_init(NULL, NULL)) {
		printf ("Initialisation failed\n");
		return 1;
	}

	if (argc < 2 || strcmp(argv[1], "--help") == 0) {
		printf ("Usage: %s <file>\n", argv[0]);
		return 1;
	}

	reader = recording_loader_new(&callbacks, NULL);
	if (reader == NULL) {
		printf("Could not start file reader\n");
		return 2;
	}

	if (!recording_loader_load(reader, argv[1])) {
		printf ("Could not load %s\n", argv[1]);
		recording_loader_free(reader);
		return 3;
	}

	recording_loader_free(reader);
	return 0;
}

