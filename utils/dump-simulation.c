#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "recording-simulator.h"

int
main (int argc, char *argv[])
{
	recording_simulator *sim = NULL;
	char *json_out_dir = NULL;
	bool full_simulation = false; // reprocess all video blobs
	int c;

	opterr = 0;

	while ((c = getopt (argc, argv, "fo:")) != -1) {
		switch (c) {
			case 'f':
	      full_simulation = true;
	      break;
			case 'o':
				json_out_dir = optarg;
				break;
			case '?':
				if (optopt == 'o')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr,
					    "Unknown option character `\\x%x'.\n", optopt);
				return 1;
			default:
				abort ();
		}
	}

	if (optind >= argc || strcmp(argv[optind], "--help") == 0) {
		printf ("Usage: %s [-o output-dir] <file>\n", argv[0]);
		return 1;
	}

	sim = recording_simulator_new(json_out_dir, full_simulation);
	if (sim == NULL) {
		printf("Could not start simulator\n");
		return 2;
	}

	if (!recording_simulator_load(sim, argv[optind])) {
		printf ("Could not load %s\n", argv[optind]);
		recording_simulator_free(sim);
		return 3;
	}

	recording_simulator_free(sim);
	return 0;
}

