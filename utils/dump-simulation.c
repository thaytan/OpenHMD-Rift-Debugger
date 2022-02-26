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

	if (argc < 2 || strcmp(argv[1], "--help") == 0) {
		printf ("Usage: %s <file>\n", argv[0]);
		return 1;
	}

	sim = recording_simulator_new();
	if (sim == NULL) {
		printf("Could not start simulator\n");
		return 2;
	}

	if (!recording_simulator_load(sim, argv[1])) {
		printf ("Could not load %s\n", argv[1]);
		recording_simulator_free(sim);
		return 3;
	}

	recording_simulator_free(sim);
	return 0;
}

