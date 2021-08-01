#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <gst/gst.h>
#include <gst/app/app.h>
typedef struct gst_reader {
  GstElement *pipeline;
  GstElement *urisource;
  GstElement *parsebin;
  GstBus *bus;
  gint next_stream_id;
} gst_reader;

typedef struct gst_reader_ctx {
  gst_reader *reader;
  gint stream_id;
  gboolean is_json;
  gboolean at_eos;
  guint frames_loaded;
} gst_reader_ctx;

static GstElement *
create_element (const gchar * type, const gchar * name)
{
  GstElement *e;

  e = gst_element_factory_make (type, name);
  if (!e) {
    g_print ("Failed to create element %s\n", type);
    return NULL;
  }

  return e;
}

static void handle_new_output_pad (GstElement * decodebin, GstPad * pad, gst_reader *reader);
static void handle_new_urisource_pad (GstElement *sourcebin, GstPad * pad, gst_reader *reader);

static gchar *
canonicalise_uri (const gchar * in)
{
  if (gst_uri_is_valid (in))
    return g_strdup (in);

  return gst_filename_to_uri (in, NULL);
}

int
main (int argc, char *argv[])
{
  struct gst_reader reader = { 0, };
  GstMessage *msg;
  gchar *uri;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  if (argc < 2 || strcmp(argv[1], "--help") == 0) {
    g_print ("Usage: %s <file>\n", argv[0]);
    return 1;
  }

  /* Build the pipeline */
  reader.pipeline = create_element ("pipeline", NULL);
  reader.urisource = create_element ("urisourcebin", NULL);
  reader.parsebin = create_element ("parsebin", NULL);

  if (reader.pipeline == NULL || reader.urisource == NULL || reader.parsebin == NULL)
    exit(1);

  /* Put all elements in the pipeline and link */
  gst_bin_add_many (GST_BIN (reader.pipeline), reader.urisource, reader.parsebin, NULL);

  uri = canonicalise_uri (argv[1]);
  g_object_set (reader.urisource, "uri", uri, NULL);
  g_free (uri);

  g_signal_connect (reader.urisource, "pad-added", G_CALLBACK (handle_new_urisource_pad), &reader);
  g_signal_connect (reader.parsebin, "pad-added", G_CALLBACK (handle_new_output_pad), &reader);

  /* Start playing */
  gst_element_set_state (reader.pipeline, GST_STATE_PLAYING);
  g_print ("Now playing %s\n", argv[1]);

  /* Wait until error or EOS */
  reader.bus = gst_element_get_bus (reader.pipeline);
  msg =
      gst_bus_timed_pop_filtered (reader.bus, GST_CLOCK_TIME_NONE,
      GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
  if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_EOS) {
    g_print ("Finished playback. Exiting.\n");
  } else {
    GError *err = NULL;
    gchar *dbg_info = NULL;

    gst_message_parse_error (msg, &err, &dbg_info);
    g_printerr ("ERROR from element %s: %s\n",
        GST_OBJECT_NAME (msg->src), err->message);
    g_printerr ("Debugging info: %s\n", (dbg_info) ? dbg_info : "none");
    g_print ("Exiting.\n");
    g_error_free (err);
    g_free (dbg_info);
  }

  /* Free resources */
  if (msg != NULL)
    gst_message_unref (msg);
  gst_object_unref (reader.bus);
  gst_element_set_state (reader.pipeline, GST_STATE_NULL);
  gst_object_unref (reader.pipeline);

  return 0;
}

static void
on_appsink_eos (GstAppSink *appsink, gpointer user_data)
{
  gst_reader_ctx *cb_ctx = user_data;
  cb_ctx->at_eos = TRUE;
}

static GstPadProbeReturn
handle_demuxed_video_buffer (GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
  gst_reader_ctx *cb_ctx = user_data;
  GstPadProbeReturn ret = GST_PAD_PROBE_OK;

  if (cb_ctx->frames_loaded >= 100)
    ret = GST_PAD_PROBE_DROP;

  return ret;
}

static GstFlowReturn
handle_decoded_video_buffer(gst_reader *reader, gst_reader_ctx *cb_ctx, GstBuffer *buffer)
{
  /* FIXME: Store video frame if it's within our target range */
  cb_ctx->frames_loaded++;

  if (cb_ctx->frames_loaded == 100)
    printf ("Stream %d - loaded 100 frames\n", cb_ctx->stream_id);

  return GST_FLOW_OK;
}

static GstFlowReturn
handle_json_buffer(gst_reader *reader, gst_reader_ctx *cb_ctx, GstBuffer *buffer)
{
  GstMapInfo info = GST_MAP_INFO_INIT; 

  if (!gst_buffer_map (buffer, &info, GST_MAP_READ))
    return GST_FLOW_ERROR;

  printf("Stream %d JSON buffer with len %" G_GSIZE_FORMAT " TS %" G_GUINT64_FORMAT ": %.*s\n",
     cb_ctx->stream_id, info.size, GST_BUFFER_PTS(buffer), (int) info.size, info.data);

  gst_buffer_unmap (buffer, &info);
  return GST_FLOW_OK;
}

static GstFlowReturn on_appsink_sample (GstAppSink *appsink, gpointer user_data)
{
  GstSample *sample = gst_app_sink_pull_sample(appsink);
  gst_reader_ctx *cb_ctx = user_data;
  GstFlowReturn ret = GST_FLOW_OK;

  if (sample) {
      GstBuffer *buf = gst_sample_get_buffer (sample);

      if (buf) {
        if (cb_ctx->is_json) {
          ret = handle_json_buffer(cb_ctx->reader, cb_ctx, buf);
        }
        else {
          ret = handle_decoded_video_buffer(cb_ctx->reader, cb_ctx, buf);
        }
      }

      gst_sample_unref (sample);
  }

  return ret;
}

static GstElement *
gen_output (gst_reader *reader, gboolean is_json)
{
  GstAppSinkCallbacks cb = { 0, };
  gst_reader_ctx *cb_ctx = NULL;
  GstElement *element = NULL;
  GstElement *decode = NULL;
  GstAppSink *appsink = NULL;
  GstPad *pad;

  cb.eos = on_appsink_eos;
  cb.new_sample = on_appsink_sample;

  cb_ctx = g_new0 (gst_reader_ctx, 1);

  cb_ctx->reader = reader;
  cb_ctx->stream_id = reader->next_stream_id++;
  cb_ctx->is_json = is_json;

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

    gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER, handle_demuxed_video_buffer, cb_ctx, NULL);

    gst_object_unref (sinkpad);
  }

  appsink = (GstAppSink *) create_element ("appsink", NULL);
  if (appsink == NULL)
    goto fail;

  gst_app_sink_set_callbacks (appsink, &cb, cb_ctx, g_free);
  g_object_set (G_OBJECT(appsink), "async", FALSE, "max-buffers", 1, "sync", FALSE, NULL);

  gst_bin_add_many (GST_BIN (element), decode, GST_ELEMENT (appsink), NULL);
  gst_element_link_many (decode, GST_ELEMENT (appsink), NULL);

  pad = gst_element_get_static_pad (decode, "sink");
  gst_element_add_pad (element, gst_ghost_pad_new ("sink", pad));
  gst_object_unref (pad);

  return element;

fail:
  if (element)
    gst_object_unref (element);
  if (appsink)
    gst_object_unref (appsink);
  if (decode)
    gst_object_unref (decode);
  g_free (cb_ctx);

  return NULL;
}

static void
handle_new_output_pad (GstElement * decodebin, GstPad * pad, gst_reader *reader)
{
  GstCaps *caps;
  GstStructure *str;
  GstElement *target;
  const gchar *name;
  GstStateChangeReturn ret;
  GstPad *sink;
  gchar *capsstr;
  gchar *pad_name;
  gboolean is_json;

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

  target = gen_output (reader, is_json);
  if (target == NULL)
    return;

  caps = gst_pad_get_current_caps (pad);
  capsstr = gst_caps_to_string (caps);
  pad_name = gst_pad_get_name (pad);
  g_print ("Adding %s output for pad %s with caps %s\n",
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

    g_print ("Failed to link parsebin pad %s with caps %s to output chain\n",
        pad_name, capsstr);

    gst_caps_unref (caps);
    g_free (capsstr);
    g_free (pad_name);

    gst_element_set_state (target, GST_STATE_NULL);
    gst_bin_remove (GST_BIN_CAST (reader->pipeline), target);

    return;
  }

state_change_error:
  g_print ("Failed to set the state of output chain\n");
  gst_bin_remove (GST_BIN_CAST (reader->pipeline), target);

  return;
}

static void
handle_new_urisource_pad (GstElement *sourcebin, GstPad * pad, gst_reader *reader)
{
  GstPad *sink;

  sink = gst_element_get_static_pad (GST_ELEMENT_CAST (reader->parsebin), "sink");

  if (gst_pad_link (pad, sink) != GST_PAD_LINK_OK) {
    gst_object_unref (sink);
    goto link_error;
  }

  gst_object_unref (sink);
  return;

link_error:{
    gchar *capsstr;
    gchar *pad_name;
    GstCaps *caps;

    caps = gst_pad_get_current_caps (pad);
    capsstr = gst_caps_to_string (caps);
    pad_name = gst_pad_get_name (pad);

    g_print ("Failed to link urisourcebin pad %s with caps %s to output chain\n",
        pad_name, capsstr);

    gst_caps_unref (caps);
    g_free (capsstr);
    g_free (pad_name);

    return;
  }

}
