#include <gst/gst.h>
#include <gst/app/app.h>

#include "utils.h"
#include "recording-loader.h"

struct gst_reader {
	GstElement *pipeline;
	GstElement *urisource;
	GstElement *parsebin;
	GstBus *bus;
	gint next_stream_id;

	GMutex stream_lock;
	GList *streams;

	bool finding_stream_ids;
	GstClockTime last_pts;
};

typedef struct gst_reader_stream_ctx {
	gst_reader *reader;
	gint stream_id;
	gboolean is_json;
	gboolean at_eos;
	guint frames_loaded;

	xml_unmarkup *xu;

	char *stream_title;
} gst_reader_stream_ctx;

static void
reader_add_stream(gst_reader *reader, gst_reader_stream_ctx *stream) {
	g_mutex_lock(&reader->stream_lock);
	reader->streams = g_list_append(reader->streams, stream);
	printf("New %s stream %d\n", stream->is_json ? "JSON" : "Video", stream->stream_id);
	g_mutex_unlock(&reader->stream_lock);
}

static void
reader_remove_stream(gst_reader *reader, gst_reader_stream_ctx *stream) {
	g_mutex_lock(&reader->stream_lock);
	reader->streams = g_list_remove(reader->streams, stream);
	g_mutex_unlock(&reader->stream_lock);
}

static bool
reader_have_all_ids(gst_reader *reader) {
	bool have_all = true;
	GList *cur = NULL;

	g_mutex_lock(&reader->stream_lock);
	for (cur = reader->streams; cur != NULL; cur = g_list_next(cur)) {
		gst_reader_stream_ctx *stream = (gst_reader_stream_ctx *)(cur->data);
		if (stream->stream_title == NULL) {
			printf("Stream %d has no ID\n", stream->stream_id);
			have_all = false;
			break;
		}
	}
	g_mutex_unlock(&reader->stream_lock);

	return have_all;
}

static void
gst_reader_stream_ctx_free (gst_reader_stream_ctx *cb_ctx)
{
	if (cb_ctx->reader)
		reader_remove_stream(cb_ctx->reader, cb_ctx);

	if (cb_ctx->xu)
		xml_unmarkup_free(cb_ctx->xu);

	g_free (cb_ctx->stream_title);
	g_free (cb_ctx);
}

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

static bool
play_pipeline(struct gst_reader *reader)
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
		g_print ("Exiting.\n");
		g_error_free (err);
		g_free (dbg_info);
		ret = false;
	}

	if (msg != NULL)
		gst_message_unref (msg);

	return ret;
}

static void
on_appsink_eos (GstAppSink *appsink, gpointer user_data)
{
	gst_reader_stream_ctx *cb_ctx = user_data;
	cb_ctx->at_eos = TRUE;
}

static GstPadProbeReturn
handle_stream_tags (GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
	gst_reader_stream_ctx *cb_ctx = user_data;
	GstPadProbeReturn ret = GST_PAD_PROBE_OK;
	GstEvent *event;

	event = gst_pad_probe_info_get_event (info);
	switch (GST_EVENT_TYPE (event)) {
		case GST_EVENT_TAG:
		{
			GstTagList *tl;
			gst_reader *reader = cb_ctx->reader;

			gst_event_parse_tag (event, &tl);

			if (tl && gst_tag_list_get_scope (tl) == GST_TAG_SCOPE_STREAM) {
				gchar *title_string = NULL;

				if (gst_tag_list_get_string (tl, GST_TAG_TITLE, &title_string)) {
					if (g_str_has_prefix(title_string, "global") ||
							g_str_has_prefix(title_string, "openhmd-rift-sensor") ||
							g_str_has_prefix(title_string, "tracked-device")) {

						printf ("Stream %d - found ID %s\n", cb_ctx->stream_id, title_string);

						g_mutex_lock(&cb_ctx->reader->stream_lock);
						cb_ctx->stream_title = title_string;
						g_mutex_unlock(&cb_ctx->reader->stream_lock);
						if (reader_have_all_ids(reader)) {
							GstMessage *msg;
							msg = gst_message_new_application(GST_OBJECT(reader->pipeline),
											gst_structure_new_empty("found-all-streams"));
							gst_element_post_message(reader->pipeline, msg);
						}
					}
					else {
						g_free (title_string);
					}
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
handle_demuxed_video_buffer (GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
	gst_reader_stream_ctx *cb_ctx = user_data;
	gst_reader *reader = cb_ctx->reader;
	GstPadProbeReturn ret = GST_PAD_PROBE_OK;

	if (reader->finding_stream_ids) {
		return GST_PAD_PROBE_DROP;
	}

#if 0
	if (cb_ctx->frames_loaded >= 100)
		ret = GST_PAD_PROBE_DROP;
#endif

	return ret;
}

static GstFlowReturn
handle_decoded_video_buffer(gst_reader *reader, gst_reader_stream_ctx *cb_ctx, GstBuffer *buffer)
{
	GstClockTime pts;
	/* FIXME: Store video frame if it's within our target range */
	cb_ctx->frames_loaded++;

	pts = GST_BUFFER_PTS(buffer);

	g_mutex_lock(&reader->stream_lock);
	GST_DEBUG("stream %d video frame PTS %" GST_TIME_FORMAT, cb_ctx->stream_id, GST_TIME_ARGS(pts));

	if (reader->last_pts > pts) {
		GST_WARNING("Time went backward - video frame %" GST_TIME_FORMAT
				" < prev PTS %" GST_TIME_FORMAT, GST_TIME_ARGS(pts),
				GST_TIME_ARGS(reader->last_pts));
	}
	reader->last_pts = pts;
	g_mutex_unlock(&reader->stream_lock);

	return GST_FLOW_OK;
}

static GstFlowReturn
handle_json_buffer(gst_reader *reader, gst_reader_stream_ctx *cb_ctx, GstBuffer *buffer)
{
	GstMapInfo info = GST_MAP_INFO_INIT;
	char *json_str;
	data_point data_point;
	GstClockTime pts;

	if (!gst_buffer_map (buffer, &info, GST_MAP_READ))
		return GST_FLOW_ERROR;

	/* Unfortunately, GStreamer decides to escape all XML markup in our
	 * JSON strings, so undo that */
	if (cb_ctx->xu == NULL)
		cb_ctx->xu = xml_unmarkup_new();
	json_str = xml_unmarkup_string(cb_ctx->xu, (char *) info.data, info.size);

	if (!json_parse_data_point_string(json_str, &data_point)) {
		printf("Failed to parse JSON buffer on Stream %d len %" G_GSIZE_FORMAT
		    " TS %" G_GUINT64_FORMAT ": %s\n",
		    cb_ctx->stream_id, info.size, GST_BUFFER_PTS(buffer), json_str);
	}
	else {
		pts = GST_BUFFER_PTS(buffer);

		g_mutex_lock(&reader->stream_lock);
		GST_DEBUG("stream %d metadata point PTS %" GST_TIME_FORMAT, cb_ctx->stream_id, GST_TIME_ARGS(pts));
#if 1
	fprintf(stderr, "Stream %d JSON buffer with len %" G_GSIZE_FORMAT " TS %" G_GUINT64_FORMAT ": %s\n",
		 cb_ctx->stream_id, info.size, GST_BUFFER_PTS(buffer), json_str);
#endif
		if (reader->last_pts > pts) {
			GST_WARNING("Time went backward - metadata point %" GST_TIME_FORMAT " < prev PTS %" GST_TIME_FORMAT, GST_TIME_ARGS(pts), GST_TIME_ARGS(reader->last_pts));
		}
		reader->last_pts = pts;
		g_mutex_unlock(&reader->stream_lock);
	}

	g_free(json_str);
	gst_buffer_unmap (buffer, &info);
	return GST_FLOW_OK;
}

static GstFlowReturn on_appsink_sample (GstAppSink *appsink, gpointer user_data)
{
	GstSample *sample = gst_app_sink_pull_sample(appsink);
	gst_reader_stream_ctx *cb_ctx = user_data;
	gst_reader *reader = cb_ctx->reader;
	GstFlowReturn ret = GST_FLOW_OK;

	if (reader->finding_stream_ids) {
		if (sample) {
			gst_sample_unref (sample);
		}
		return GST_FLOW_OK;
	}

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
	gst_reader_stream_ctx *cb_ctx = NULL;
	GstElement *element = NULL;
	GstElement *decode = NULL;
	GstAppSink *appsink = NULL;
	GstPad *pad;

	cb.eos = on_appsink_eos;
	cb.new_sample = on_appsink_sample;

	cb_ctx = g_new0 (gst_reader_stream_ctx, 1);

	cb_ctx->reader = reader;
	cb_ctx->stream_id = reader->next_stream_id++;
	cb_ctx->is_json = is_json;

	reader_add_stream(cb_ctx->reader, cb_ctx);

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

	gst_app_sink_set_callbacks (appsink, &cb, cb_ctx, (GDestroyNotify) gst_reader_stream_ctx_free);
	g_object_set (G_OBJECT(appsink), "async", FALSE, "max-buffers", 1, "sync", FALSE, NULL);

	/* Catch TAG events. We could get these off the bus, but this way works too
	 * and gives more direct access to the cb_ctx */
	pad = gst_element_get_static_pad (decode, "sink");
	gst_pad_add_probe (pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, handle_stream_tags, cb_ctx, NULL);
	gst_object_unref (pad);

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
	gst_reader_stream_ctx_free (cb_ctx);

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

link_error:
	{
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

gst_reader *recording_loader_new(const char *uri)
{
	struct gst_reader *reader = NULL;

	reader = calloc(1, sizeof(gst_reader));

	g_mutex_init(&reader->stream_lock);

	/* Build the pipeline */
	reader->pipeline = create_element ("pipeline", NULL);
	reader->urisource = create_element ("urisourcebin", NULL);
	reader->parsebin = create_element ("parsebin", NULL);

	if (reader->pipeline == NULL || reader->urisource == NULL || reader->parsebin == NULL)
		goto fail;

	reader->bus = gst_element_get_bus (reader->pipeline);

	/* Put all elements in the pipeline and link */
	gst_bin_add_many (GST_BIN (reader->pipeline), reader->urisource, reader->parsebin, NULL);

	g_signal_connect (reader->urisource, "pad-added", G_CALLBACK (handle_new_urisource_pad), reader);
	g_signal_connect (reader->parsebin, "pad-added", G_CALLBACK (handle_new_output_pad), reader);

	return reader;

fail:
	recording_loader_free(reader);
	return NULL;
}

static gchar *
canonicalise_uri (const gchar * in)
{
	if (gst_uri_is_valid (in))
		return g_strdup (in);

	return gst_filename_to_uri (in, NULL);
}

bool recording_loader_load(gst_reader *reader, const char *filename_or_uri)
{
	bool ret = false;
	gchar *uri = canonicalise_uri((const gchar *) filename_or_uri);

	/* Start playing */
	reader->finding_stream_ids = true;
	g_object_set (reader->urisource, "uri", uri, NULL);
	g_free(uri);

	if (gst_element_set_state (reader->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		g_print ("Failed to load %s\n", filename_or_uri);
		goto done;
	}
	g_print ("Now playing %s\n", filename_or_uri);

	/* Play the pipeline once to discover stream tags */
	if (!play_pipeline(reader))
		goto done;

	if (reader->finding_stream_ids) {
		/* FIXME: Check we have all the stream ids */
		if (!reader_have_all_ids(reader)) {
			printf("Failed to find stream IDs for all streams. The recording is damaged\n");
			goto done;
		}

		printf("Collected stream IDs. Starting parsing.\n");
		reader->finding_stream_ids = false;

		if (!gst_element_seek_simple (GST_ELEMENT(reader->pipeline), GST_FORMAT_TIME,
				GST_SEEK_FLAG_FLUSH, 0))
			goto done;

		/* Clear all other pending messages */
		gst_bus_set_flushing(reader->bus, TRUE);
		gst_bus_set_flushing(reader->bus, FALSE);

		if (!play_pipeline(reader))
			goto done;
	}

	g_print ("Finished playback. Exiting.\n");
	ret = true;

done:
	return ret;
}

void
recording_loader_free(gst_reader *reader)
{
	/* Free resources */
	if (reader->bus)
		gst_object_unref(reader->bus);

	if (reader->pipeline) {
		gst_element_set_state (reader->pipeline, GST_STATE_NULL);
		gst_object_unref (reader->pipeline);
	}
}

bool recording_loader_init()
{
	/* Initialize GStreamer */
	gst_init (NULL, NULL);

	return true;
}
