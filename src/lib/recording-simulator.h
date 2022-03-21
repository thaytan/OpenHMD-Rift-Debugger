#ifndef __RECORDING_SIMULATOR_H__
#define __RECORDING_SIMULATOR_H__

#include <stdbool.h>

typedef struct recording_simulator recording_simulator;

recording_simulator *recording_simulator_new(const char *json_output_dir);
bool recording_simulator_load(recording_simulator *reader, const char *filename_or_uri);
void recording_simulator_free(recording_simulator *reader);

#endif
