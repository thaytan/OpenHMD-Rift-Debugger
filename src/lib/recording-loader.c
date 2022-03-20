#include <assert.h>

#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/video/video.h>

#include "utils.h"
#include "recording-loader.h"

#define MAX_STREAMS 16

#if GST_CHECK_VERSION(1,19,1)
#define USE_URISOURCEBIN 1
#endif

struct recording_loader
{
  GstElement *pipeline;
  GstElement *source;
  GstElement *parsebin;
  GstBus *bus;
  gint next_stream_id;

  GMutex stream_lock;
  gint n_streams;
  recording_loader_stream *streams[MAX_STREAMS];

  bool finding_stream_ids;
  GstClockTime last_pts;

  recording_loader_callbacks callbacks;
  void *callback_data;
};

struct recording_loader_stream
{
  recording_loader *reader;
  gint stream_id;
  recording_stream_type stream_type;

  gboolean is_json;
  gboolean at_eos;
  guint frames_decoded;

  xml_unmarkup *xu;

  char *stream_title;

  void *callback_data;

  /* Element that handles this stream */
  GstElement *target;

  GstCaps *caps;
  GstVideoInfo vinfo;
};

static void
reader_add_stream (recording_loader * reader, recording_loader_stream * stream)
{
  g_mutex_lock (&reader->stream_lock);
  assert (reader->n_streams < MAX_STREAMS);
  reader->streams[reader->n_streams++] = stream;
  GST_INFO ("New %s stream %d\n", stream->is_json ? "JSON" : "Video",
      stream->stream_id);
  g_mutex_unlock (&reader->stream_lock);
}

static void
reader_remove_stream (recording_loader * reader,
    recording_loader_stream * stream)
{
  int i;

  g_mutex_lock (&reader->stream_lock);
  for (i = 0; i < reader->n_streams; i++) {
    if (reader->streams[i] == stream)
      reader->streams[i] = NULL;
  }
  g_mutex_unlock (&reader->stream_lock);
}

static bool
reader_have_all_ids (recording_loader * reader)
{
  bool have_all = true;
  int i;

  g_mutex_lock (&reader->stream_lock);
  for (i = 0; i < reader->n_streams; i++) {
    recording_loader_stream *stream = reader->streams[i];
    if (stream->stream_title == NULL) {
      have_all = false;
      break;
    }
  }
  g_mutex_unlock (&reader->stream_lock);

  return have_all;
}

static void
reader_announce_streams (recording_loader * reader)
{
  int i;

  if (reader->callbacks.new_stream == NULL)
    return;

  g_mutex_lock (&reader->stream_lock);
  for (i = 0; i < reader->n_streams; i++) {
    recording_loader_stream *stream = reader->streams[i];
    reader->callbacks.new_stream (reader->callback_data,
        stream, stream->stream_type, stream->stream_title);
  }
  g_mutex_unlock (&reader->stream_lock);
}

static void
recording_loader_stream_free (recording_loader_stream * stream)
{
  if (stream->reader)
    reader_remove_stream (stream->reader, stream);

  if (stream->xu)
    xml_unmarkup_free (stream->xu);

  if (stream->target)
    gst_object_unref (stream->target);

  if (stream->caps)
    gst_caps_unref (stream->caps);

  g_free (stream->stream_title);
  g_free (stream);
}

static GstElement *
create_element (const gchar * type, const gchar * name)
{
  GstElement *e;

  e = gst_element_factory_make (type, name);
  if (!e) {
    g_printerr ("Failed to create element %s\n", type);
    return NULL;
  }

  return e;
}

static void handle_new_output_pad (GstElement * decodebin, GstPad * pad,
    recording_loader * reader);
#if USE_URISOURCEBIN
static void handle_new_urisource_pad (GstElement * sourcebin, GstPad * pad,
    recording_loader * reader);
#endif

static bool
play_pipeline (struct recording_loader *reader)
{
  GstMessage *msg =
      gst_bus_timed_pop_filtered (reader->bus, GST_CLOCK_TIME_NONE,
      GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_APPLICATION);
  bool ret = true;

  if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_ERROR) {
    GError *err = NULL;
    gchar *dbg_info = NULL;

    gst_message_parse_error (msg, &err, &dbg_info);
    g_printerr ("ERROR from element %s: %s\n",
        GST_OBJECT_NAME (msg->src), err->message);
    g_printerr ("Debugging info: %s\n", (dbg_info) ? dbg_info : "none");
    g_error_free (err);
    g_free (dbg_info);
    ret = false;
  }
  else {
    GST_INFO ("Finishing playback after message %" GST_PTR_FORMAT,
        msg);
  }

  if (msg != NULL)
    gst_message_unref (msg);

  return ret;
}

static void
on_appsink_eos (GstAppSink * appsink, gpointer user_data)
{
  recording_loader_stream *stream = user_data;
  stream->at_eos = TRUE;
}

static GstPadProbeReturn
handle_stream_tags (GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  recording_loader_stream *stream = user_data;
  GstPadProbeReturn ret = GST_PAD_PROBE_OK;
  GstEvent *event;

  event = gst_pad_probe_info_get_event (info);
  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_TAG:
    {
      GstTagList *tl;
      recording_loader *reader = stream->reader;

      gst_event_parse_tag (event, &tl);

      if (tl && gst_tag_list_get_scope (tl) == GST_TAG_SCOPE_STREAM) {
        gchar *title_string = NULL;
        recording_stream_type stream_type;

        GST_DEBUG ("Got tag list %" GST_PTR_FORMAT, tl);
        if (!gst_tag_list_get_string (tl, GST_TAG_TITLE, &title_string))
          break;

        if (stream->is_json && g_str_has_prefix (title_string, "global")) {
          stream_type = RECORDING_STREAM_TYPE_GLOBAL_METADATA;
        } else if (stream->is_json
            && g_str_has_prefix (title_string, "tracked-device")) {
          stream_type = RECORDING_STREAM_TYPE_DEVICE_METADATA;
        } else if (!stream->is_json
            && g_str_has_prefix (title_string, "openhmd-rift-sensor")) {
          stream_type = RECORDING_STREAM_TYPE_CAMERA_STREAM;
        } else {
          g_free (title_string);
          break;
        }

        g_mutex_lock (&stream->reader->stream_lock);
        stream->stream_type = stream_type;
        if (stream->stream_title == NULL) {
          stream->stream_title = title_string;
          GST_INFO ("Stream %d - found ID %s\n", stream->stream_id,
              title_string);
          g_mutex_unlock (&stream->reader->stream_lock);
        } else {
          assert (g_str_equal (title_string, stream->stream_title));
          g_free (title_string);
          g_mutex_unlock (&stream->reader->stream_lock);
          break;
        }

        if (reader_have_all_ids (reader)) {
          GstMessage *msg;
          msg = gst_message_new_application (GST_OBJECT (reader->pipeline),
              gst_structure_new_empty ("found-all-streams"));
          gst_element_post_message (reader->pipeline, msg);
        }
      }
      break;
    default:
      break;
    }
  }

  return ret;
}

static GstPadProbeReturn
handle_demuxed_video_buffer (GstPad * pad, GstPadProbeInfo * info,
    gpointer user_data)
{
  recording_loader_stream *stream = user_data;
  recording_loader *reader = stream->reader;

  /* Don't do anything during initial stream ID pass */
  if (reader->finding_stream_ids) {
    return GST_PAD_PROBE_DROP;
  }

  if (reader->callbacks.on_encoded_frame != NULL) {
    bool ret;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_DATA (info);
    GstMapInfo map = GST_MAP_INFO_INIT;
    GstClockTime pts = GST_BUFFER_PTS (buffer);

    if (!gst_buffer_map (buffer, &map, GST_MAP_READ)) {
      GST_PAD_PROBE_INFO_FLOW_RETURN (info) = GST_FLOW_ERROR;
      return GST_PAD_PROBE_OK;
    }
    ret = reader->callbacks.on_encoded_frame (reader->callback_data,
        stream, (uint64_t) pts, map.data, map.size);
    gst_buffer_unmap (buffer, &map);

    if (!ret)
      return GST_PAD_PROBE_DROP;
  }

  return GST_PAD_PROBE_OK;
}

static GstFlowReturn
handle_decoded_video_buffer (recording_loader * reader,
    recording_loader_stream * stream, GstBuffer * buffer)
{
  GstMapInfo map = GST_MAP_INFO_INIT;
  GstClockTime pts;
  stream->frames_decoded++;

  pts = GST_BUFFER_PTS (buffer);
  if (!gst_buffer_map (buffer, &map, GST_MAP_READ))
    return GST_FLOW_ERROR;

  g_mutex_lock (&reader->stream_lock);
  GST_DEBUG ("stream %d video frame PTS %" GST_TIME_FORMAT, stream->stream_id,
      GST_TIME_ARGS (pts));

  if (reader->last_pts > pts) {
    GST_WARNING ("Time went backward - video frame %" GST_TIME_FORMAT
        " < prev PTS %" GST_TIME_FORMAT, GST_TIME_ARGS (pts),
        GST_TIME_ARGS (reader->last_pts));
  }
  reader->last_pts = pts;

  if (reader->callbacks.on_video_frame) {
    reader->callbacks.on_video_frame (reader->callback_data,
        stream, pts, map.data, map.size);
  }
  g_mutex_unlock (&reader->stream_lock);

  gst_buffer_unmap (buffer, &map);

  return GST_FLOW_OK;
}

static GstFlowReturn
handle_json_buffer (recording_loader * reader, recording_loader_stream * stream,
    GstBuffer * buffer)
{
  GstMapInfo info = GST_MAP_INFO_INIT;
  char *json_str;
  data_point data_point;
  GstClockTime pts = GST_BUFFER_PTS (buffer);

  if (!gst_buffer_map (buffer, &info, GST_MAP_READ))
    return GST_FLOW_ERROR;

  /* Unfortunately, GStreamer decides to escape all XML markup in our
   * JSON strings, so undo that */
  if (stream->xu == NULL)
    stream->xu = xml_unmarkup_new ();
  json_str = xml_unmarkup_string (stream->xu, (char *) info.data, info.size);

  if (reader->callbacks.on_json_data) {
    reader->callbacks.on_json_data (reader->callback_data, stream, pts,
        json_str);
  }

  if (!json_parse_data_point_string (json_str, &data_point)) {
    g_printerr ("Failed to parse JSON buffer on Stream %d len %" G_GSIZE_FORMAT
        " TS %" G_GUINT64_FORMAT ": %s\n",
        stream->stream_id, info.size, GST_BUFFER_PTS (buffer), json_str);
  } else {

    g_mutex_lock (&reader->stream_lock);
    GST_DEBUG ("stream %d metadata point PTS %" GST_TIME_FORMAT,
        stream->stream_id, GST_TIME_ARGS (pts));
#if 0
    fprintf (stderr,
        "Stream %d JSON buffer with len %" G_GSIZE_FORMAT " TS %"
        G_GUINT64_FORMAT ": %s\n", stream->stream_id, info.size,
        GST_BUFFER_PTS (buffer), json_str);
#endif
    if (reader->last_pts > pts) {
      /* This happens sometimes when events during the recording overlap - the
       * timestamp is recorded in a thread before the event is written to the log,
       * and some other thread sneaks in first */
      GST_LOG ("Time went backward - metadata point %" GST_TIME_FORMAT
          " < prev PTS %" GST_TIME_FORMAT, GST_TIME_ARGS (pts),
          GST_TIME_ARGS (reader->last_pts));
    }
    reader->last_pts = pts;

    if (reader->callbacks.on_event) {
      reader->callbacks.on_event (reader->callback_data, stream, pts,
          &data_point, json_str);
    }
    g_mutex_unlock (&reader->stream_lock);
  }

  g_free (json_str);
  gst_buffer_unmap (buffer, &info);
  return GST_FLOW_OK;
}

static GstFlowReturn
on_appsink_sample (GstAppSink * appsink, gpointer user_data)
{
  GstSample *sample = gst_app_sink_pull_sample (appsink);
  recording_loader_stream *stream = user_data;
  recording_loader *reader = stream->reader;
  GstFlowReturn ret = GST_FLOW_OK;

  if (sample) {
    GstCaps *caps = gst_sample_get_caps (sample);

    if (stream->caps == NULL || !gst_caps_is_equal (stream->caps, caps)) {
      GST_LOG ("Caps changed on stream %d from %" GST_PTR_FORMAT " to %" GST_PTR_FORMAT,
          stream->stream_id, stream->caps, caps);

      if (!stream->is_json && reader->callbacks.on_frame_config) {
        if (gst_video_info_from_caps (&stream->vinfo, caps)) {
          reader->callbacks.on_frame_config (reader->callback_data, stream,
              GST_VIDEO_INFO_WIDTH (&stream->vinfo),
              GST_VIDEO_INFO_HEIGHT (&stream->vinfo),
              GST_VIDEO_INFO_COMP_STRIDE (&stream->vinfo, 0));
        }
      }

      gst_caps_replace (&stream->caps, caps);
    }

    GstBuffer *buf = gst_sample_get_buffer (sample);
    if (buf) {
      GST_LOG ("Sample on stream %d PTS %" G_GUINT64_FORMAT, stream->stream_id,
          GST_BUFFER_PTS (buf));
    }
  }

  if (reader->finding_stream_ids) {
    if (sample) {
      gst_sample_unref (sample);
    }
    return GST_FLOW_OK;
  }

  if (sample) {
    GstBuffer *buf = gst_sample_get_buffer (sample);

    if (buf) {
      if (stream->is_json) {
        ret = handle_json_buffer (stream->reader, stream, buf);
      } else {
        ret = handle_decoded_video_buffer (stream->reader, stream, buf);
      }
    }

    gst_sample_unref (sample);
  }

  return ret;
}

static GstElement *
gen_output (recording_loader * reader, gboolean is_json)
{
  GstAppSinkCallbacks cb = { 0, };
  recording_loader_stream *stream = NULL;
  GstElement *element = NULL;
  GstElement *decode = NULL;
  GstAppSink *appsink = NULL;
  GstPad *pad;

  cb.eos = on_appsink_eos;
  cb.new_sample = on_appsink_sample;

  stream = g_new0 (recording_loader_stream, 1);

  stream->reader = reader;
  stream->stream_id = reader->next_stream_id++;
  stream->is_json = is_json;

  reader_add_stream (stream->reader, stream);

  element = gst_bin_new (NULL);
  if (element == NULL)
    goto fail;

  if (is_json) {
    decode = create_element ("identity", NULL);
  } else {
    decode = create_element ("jpegdec", NULL);
  }
  if (decode == NULL)
    goto fail;

  if (!is_json) {
    GstPad *sinkpad;

    /* Set up pad probe on the input of JPEG dec to skip decoding
     * of frames we're not currently interested in */
    sinkpad = gst_element_get_static_pad (decode, "sink");
    if (sinkpad == NULL)
      goto fail;

    gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER,
        handle_demuxed_video_buffer, stream, NULL);

    gst_object_unref (sinkpad);
  }

  appsink = (GstAppSink *) create_element ("appsink", NULL);
  if (appsink == NULL)
    goto fail;

  gst_app_sink_set_callbacks (appsink, &cb, stream,
      (GDestroyNotify) recording_loader_stream_free);
  g_object_set (G_OBJECT (appsink), "async", FALSE, "max-buffers", 1, "sync",
      FALSE, NULL);

  /* Catch TAG events. We could get these off the bus, but this way works too
   * and gives more direct access to the stream */
  pad = gst_element_get_static_pad (decode, "sink");
  gst_pad_add_probe (pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM,
      handle_stream_tags, stream, NULL);
  gst_object_unref (pad);

  gst_bin_add_many (GST_BIN (element), decode, GST_ELEMENT (appsink), NULL);
  gst_element_link_many (decode, GST_ELEMENT (appsink), NULL);

  pad = gst_element_get_static_pad (decode, "sink");
  gst_element_add_pad (element, gst_ghost_pad_new ("sink", pad));
  gst_object_unref (pad);

  stream->target = gst_object_ref (element);

  return element;

fail:
  if (element)
    gst_object_unref (element);
  if (appsink)
    gst_object_unref (appsink);
  if (decode)
    gst_object_unref (decode);
  recording_loader_stream_free (stream);

  return NULL;
}

static void
handle_new_output_pad (GstElement * decodebin, GstPad * pad,
    recording_loader * reader)
{
  GstCaps *caps;
  GstStructure *str;
  const gchar *name;
  GstStateChangeReturn ret;
  GstPad *sink;
  gchar *capsstr;
  gchar *pad_name;
  gboolean is_json;

  /* On the 2nd pass, our stream should already exist - just relink it */
  if (reader->finding_stream_ids == FALSE) {
    assert (reader->next_stream_id < MAX_STREAMS);
    assert (reader->streams[reader->next_stream_id] != NULL);

    recording_loader_stream *stream = reader->streams[reader->next_stream_id];
    assert (stream->target != NULL);

    sink =
        gst_element_get_static_pad (GST_ELEMENT_CAST (stream->target), "sink");

    GST_INFO ("Reconnecting pad %" GST_PTR_FORMAT " to target %" GST_PTR_FORMAT,
        pad, sink);

    if (gst_pad_link (pad, sink) != GST_PAD_LINK_OK) {
      gst_object_unref (sink);
      g_printerr ("Failed to re-link pad for stream %d on 2nd pass\n",
          reader->next_stream_id);
      return;
    }
    gst_object_unref (sink);
    reader->next_stream_id++;
    return;
  }

  /* Extract the caps from the pad to see if it is audio or video */
  caps = gst_pad_get_current_caps (pad);
  str = gst_caps_get_structure (caps, 0);
  name = gst_structure_get_name (str);

  if (strcmp (name, "image/jpeg") == 0) {
    is_json = FALSE;
  } else if (strcmp (name, "text/x-raw") == 0) {
    is_json = TRUE;
  } else {
    return;
  }
  /* Finished with the caps, unref them so they don't leak. */
  gst_caps_unref (caps);

  GstElement *target = gen_output (reader, is_json);
  if (target == NULL)
    return;

  caps = gst_pad_get_current_caps (pad);
  capsstr = gst_caps_to_string (caps);
  pad_name = gst_pad_get_name (pad);
  GST_INFO ("Adding %s output for pad %s with caps %s\n",
      is_json ? "JSON" : "JPEG", pad_name, capsstr);
  gst_caps_unref (caps);
  g_free (capsstr);
  g_free (pad_name);

  gst_bin_add (GST_BIN_CAST (reader->pipeline), target);

  ret = gst_element_set_state (target, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE)
    goto state_change_error;

  sink = gst_element_get_static_pad (GST_ELEMENT_CAST (target), "sink");

  if (gst_pad_link (pad, sink) != GST_PAD_LINK_OK) {
    gst_object_unref (sink);
    goto link_error;
  }

  gst_object_unref (sink);

  return;

link_error:{

    caps = gst_pad_get_current_caps (pad);
    capsstr = gst_caps_to_string (caps);
    pad_name = gst_pad_get_name (pad);

    g_printerr ("Failed to link parsebin pad %s with caps %s to output chain\n",
        pad_name, capsstr);

    gst_caps_unref (caps);
    g_free (capsstr);
    g_free (pad_name);

    gst_element_set_state (target, GST_STATE_NULL);
    gst_bin_remove (GST_BIN_CAST (reader->pipeline), target);

    return;
  }

state_change_error:
  g_printerr ("Failed to set the state of output chain\n");
  gst_bin_remove (GST_BIN_CAST (reader->pipeline), target);

  return;
}

#if USE_URISOURCEBIN
static void
handle_new_urisource_pad (GstElement * sourcebin, GstPad * pad,
    recording_loader * reader)
{
  GstPad *sink;

  sink =
      gst_element_get_static_pad (GST_ELEMENT_CAST (reader->parsebin), "sink");

  if (gst_pad_link (pad, sink) != GST_PAD_LINK_OK) {
    gst_object_unref (sink);
    goto link_error;
  }

  gst_object_unref (sink);
  return;

link_error:
  {
    gchar *capsstr;
    gchar *pad_name;
    GstCaps *caps;

    caps = gst_pad_get_current_caps (pad);
    capsstr = gst_caps_to_string (caps);
    pad_name = gst_pad_get_name (pad);

    g_print
        ("Failed to link urisourcebin pad %s with caps %s to output chain\n",
        pad_name, capsstr);

    if (caps)
      gst_caps_unref (caps);
    g_free (capsstr);
    g_free (pad_name);

    return;
  }
}
#endif

recording_loader *
recording_loader_new (recording_loader_callbacks * cb, void *cb_data)
{
  struct recording_loader *reader = NULL;

  reader = calloc (1, sizeof (recording_loader));

  g_mutex_init (&reader->stream_lock);
  if (cb)
    reader->callbacks = *cb;
  reader->callback_data = cb_data;

  /* Build the pipeline */
  reader->pipeline = create_element ("pipeline", NULL);

#if USE_URISOURCEBIN
  reader->source = create_element ("urisourcebin", NULL);
#else
  reader->source = create_element ("filesrc", NULL);
#endif
  reader->parsebin = create_element ("parsebin", NULL);

  if (reader->pipeline == NULL || reader->source == NULL
      || reader->parsebin == NULL)
    goto fail;

  reader->bus = gst_element_get_bus (reader->pipeline);

  /* Put all elements in the pipeline and link */
  gst_bin_add_many (GST_BIN (reader->pipeline), reader->source,
      reader->parsebin, NULL);

#if USE_URISOURCEBIN
  g_signal_connect (reader->source, "pad-added",
      G_CALLBACK (handle_new_urisource_pad), reader);
#else
  gst_element_link (reader->source, reader->parsebin);
#endif

  g_signal_connect (reader->parsebin, "pad-added",
      G_CALLBACK (handle_new_output_pad), reader);

  return reader;

fail:
  recording_loader_free (reader);
  return NULL;
}

static bool
reset_reader (recording_loader * reader)
{
  gst_element_set_state (reader->pipeline, GST_STATE_READY);
  reader->next_stream_id = 0;

  /* Clear pending bus messages */
  gst_bus_set_flushing (reader->bus, TRUE);
  gst_bus_set_flushing (reader->bus, FALSE);

  if (gst_element_set_state (reader->pipeline,
          GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Failed to reset pipeline\n");
    return false;
  }

  return true;
}

#if USE_URISOURCEBIN
static gchar *
canonicalise_uri (const gchar * in)
{
  if (gst_uri_is_valid (in))
    return g_strdup (in);

  return gst_filename_to_uri (in, NULL);
}
#endif

bool
recording_loader_load (recording_loader * reader, const char *filename_or_uri)
{
  bool ret = false;

  /* Start playing */
  reader->finding_stream_ids = true;

#if USE_URISOURCEBIN
  gchar *uri = canonicalise_uri ((const gchar *) filename_or_uri);
  g_object_set (reader->source, "uri", uri, NULL);
  g_free (uri);
#else
  g_object_set (reader->source, "location", filename_or_uri, NULL);
#endif

  if (gst_element_set_state (reader->pipeline,
          GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Failed to load %s\n", filename_or_uri);
    goto done;
  }
  GST_INFO ("Now playing %s\n", filename_or_uri);

  /* Play the pipeline once to discover stream tags */
  if (!play_pipeline (reader))
    goto done;

  /* Check we have all the stream ids */
  if (!reader_have_all_ids (reader)) {
    g_printerr
        ("Failed to find stream IDs for all streams. The recording is damaged\n");
    goto done;
  }

  GST_INFO ("Collected stream IDs. Starting parsing.\n");
  reader->finding_stream_ids = false;
  reader_announce_streams (reader);

  /* Reset and actually parse things now */
  if (!reset_reader (reader))
    goto done;

  if (!play_pipeline (reader))
    goto done;

  GST_INFO ("Finished playback. Exiting.\n");
  ret = true;

done:
  return ret;
}

void
recording_loader_free (recording_loader * reader)
{
  /* Free resources */
  if (reader->bus)
    gst_object_unref (reader->bus);

  if (reader->pipeline) {
    gst_element_set_state (reader->pipeline, GST_STATE_NULL);
    gst_object_unref (reader->pipeline);
  }
}

bool
recording_loader_init ()
{
  /* Initialize GStreamer */
  gst_init (NULL, NULL);

  return true;
}

int
recording_loader_stream_id (recording_loader_stream * stream)
{
  return stream->stream_id;
}

void
recording_loader_stream_set_cbdata (recording_loader_stream * stream,
    void *cb_data)
{
  stream->callback_data = cb_data;
}

void *
recording_loader_stream_get_cbdata (recording_loader_stream * stream)
{
  return stream->callback_data;
}
