#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "recording-loader.h"

int
main (int argc, char *argv[])
{
	gst_reader *reader = NULL;
	bool ret;

	/* Initialize GStreamer */
	if (!recording_loader_init()) {
		printf ("Initialisation failed\n");
		return 1;
	}

	if (argc < 2 || strcmp(argv[1], "--help") == 0) {
		printf ("Usage: %s <file>\n", argv[0]);
		return 1;
	}

	reader = recording_loader_new();
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

