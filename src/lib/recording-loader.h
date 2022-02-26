#ifndef __RECORDING_LOADER_H__
#define __RECORDING_LOADER_H__

#include <stdbool.h>
#include "recording-data.h"

typedef enum recording_stream_type recording_stream_type;

typedef struct recording_loader recording_loader;
typedef struct recording_loader_stream recording_loader_stream;
typedef struct recording_loader_callbacks recording_loader_callbacks;

enum recording_stream_type {
  RECORDING_STREAM_TYPE_CAMERA_STREAM,
  RECORDING_STREAM_TYPE_GLOBAL_METADATA,
  RECORDING_STREAM_TYPE_DEVICE_METADATA,
};

struct recording_loader_callbacks {
  void (*new_stream)(void *cb_data, recording_loader_stream *stream, recording_stream_type type, const char *stream_name);
  void (*on_json_data)(void *cb_data, recording_loader_stream *stream, uint64_t pts, const char *json_data);
  void (*on_event)(void *cb_data, recording_loader_stream *stream, uint64_t pts, struct data_point *data);
  void (*on_frame_config)(void *cb_data, recording_loader_stream *stream, int width, int height, int stride);
  bool (*on_encoded_frame)(void *cb_data, recording_loader_stream *stream, uint64_t pts, unsigned char *data, size_t len);
  void (*on_video_frame)(void *cb_data, recording_loader_stream *stream, uint64_t pts, unsigned char *frame_data, size_t frame_len);
};

bool recording_loader_init();
recording_loader *recording_loader_new(recording_loader_callbacks *cb, void *cb_data);
bool recording_loader_load(recording_loader *reader, const char *filename_or_uri);
void recording_loader_free(recording_loader *reader);

int recording_loader_stream_id(recording_loader_stream *stream);
void recording_loader_stream_set_cbdata(recording_loader_stream *stream, void *cb_data);
void *recording_loader_stream_get_cbdata(recording_loader_stream *stream);

#endif
