#ifndef __RECORDING_LOADER_H__
#define __RECORDING_LOADER_H__

#include <stdbool.h>
#include "recording-data.h"

typedef struct gst_reader gst_reader;

bool recording_loader_init();
gst_reader *recording_loader_new();
bool recording_loader_load(gst_reader *reader, const char *filename_or_uri);
void recording_loader_free(gst_reader *reader);

#endif
