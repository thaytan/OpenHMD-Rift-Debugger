#include <gst/gst.h>
#include <gst/app/app.h>

#include "utils.h"
#include "recording-loader.h"

struct recording_loader {
	GstElement *pipeline;
	GstElement *urisource;
	GstElement *parsebin;
	GstBus *bus;
	gint next_stream_id;

	GMutex stream_lock;
	GList *streams;

	bool finding_stream_ids;
	GstClockTime last_pts;

	recording_loader_callbacks callbacks;
	void *callback_data;
};

typedef struct recording_loader_stream_ctx {
	recording_loader *reader;
	gint stream_id;
	recording_stream_type stream_type;

	gboolean is_json;
	gboolean at_eos;
	guint frames_decoded;

	xml_unmarkup *xu;

	char *stream_title;
} recording_loader_stream_ctx;

static void
reader_add_stream(recording_loader *reader, recording_loader_stream_ctx *stream) {
	g_mutex_lock(&reader->stream_lock);
	reader->streams = g_list_append(reader->streams, stream);
	g_print("New %s stream %d\n", stream->is_json ? "JSON" : "Video", stream->stream_id);
	g_mutex_unlock(&reader->stream_lock);
}

static void
reader_remove_stream(recording_loader *reader, recording_loader_stream_ctx *stream) {
	g_mutex_lock(&reader->stream_lock);
	reader->streams = g_list_remove(reader->streams, stream);
	g_mutex_unlock(&reader->stream_lock);
}

static bool
reader_have_all_ids(recording_loader *reader) {
	bool have_all = true;
	GList *cur = NULL;

	g_mutex_lock(&reader->stream_lock);
	for (cur = reader->streams; cur != NULL; cur = g_list_next(cur)) {
		recording_loader_stream_ctx *stream = (recording_loader_stream_ctx *)(cur->data);
		if (stream->stream_title == NULL) {
			have_all = false;
			break;
		}
	}
	g_mutex_unlock(&reader->stream_lock);

	return have_all;
}

static void
reader_announce_streams(recording_loader *reader)
{
	GList *cur = NULL;

	if (reader->callbacks.new_stream == NULL)
		return;

	g_mutex_lock(&reader->stream_lock);
	for (cur = reader->streams; cur != NULL; cur = g_list_next(cur)) {
		recording_loader_stream_ctx *stream = (recording_loader_stream_ctx *)(cur->data);
		reader->callbacks.new_stream(reader->callback_data,
				stream->stream_id, stream->stream_type, stream->stream_title);
	}
	g_mutex_unlock(&reader->stream_lock);
}

static void
recording_loader_stream_ctx_free (recording_loader_stream_ctx *stream)
{
	if (stream->reader)
		reader_remove_stream(stream->reader, stream);

	if (stream->xu)
		xml_unmarkup_free(stream->xu);

	g_free (stream->stream_title);
	g_free (stream);
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

static void handle_new_output_pad (GstElement * decodebin, GstPad * pad, recording_loader *reader);
static void handle_new_urisource_pad (GstElement *sourcebin, GstPad * pad, recording_loader *reader);

static bool
play_pipeline(struct recording_loader *reader)
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
	recording_loader_stream_ctx *stream = user_data;
	stream->at_eos = TRUE;
}

static GstPadProbeReturn
handle_stream_tags (GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
	recording_loader_stream_ctx *stream = user_data;
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

				if (!gst_tag_list_get_string (tl, GST_TAG_TITLE, &title_string))
					break;

				if (stream->is_json && g_str_has_prefix(title_string, "global")) {
					stream_type = RECORDING_STREAM_TYPE_GLOBAL_METADATA;
				}
				else if (stream->is_json && g_str_has_prefix(title_string, "tracked-device")) {
					stream_type = RECORDING_STREAM_TYPE_DEVICE_METADATA;
				}
				else if (!stream->is_json && g_str_has_prefix(title_string, "openhmd-rift-sensor")) {
					stream_type = RECORDING_STREAM_TYPE_CAMERA_STREAM;
				}
				else {
					g_free (title_string);
					break;
				}

				g_print ("Stream %d - found ID %s\n", stream->stream_id, title_string);

				g_mutex_lock(&stream->reader->stream_lock);
				stream->stream_type = stream_type;
				stream->stream_title = title_string;
				g_mutex_unlock(&stream->reader->stream_lock);
				if (reader_have_all_ids(reader)) {
					GstMessage *msg;
					msg = gst_message_new_application(GST_OBJECT(reader->pipeline),
									gst_structure_new_empty("found-all-streams"));
					gst_element_post_message(reader->pipeline, msg);
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
	recording_loader_stream_ctx *stream = user_data;
	recording_loader *reader = stream->reader;

	/* Don't do anything during initial stream ID pass */
	if (reader->finding_stream_ids) {
		return GST_PAD_PROBE_DROP;
	}

	if (reader->callbacks.on_encoded_frame != NULL) {
		bool ret;
		GstBuffer *buffer = GST_PAD_PROBE_INFO_DATA(info);
		GstMapInfo map = GST_MAP_INFO_INIT;
		GstClockTime pts = GST_BUFFER_PTS(buffer);

		if (!gst_buffer_map (buffer, &map, GST_MAP_READ)) {
			GST_PAD_PROBE_INFO_FLOW_RETURN(info) = GST_FLOW_ERROR;
			return GST_PAD_PROBE_OK;
		}
		ret = reader->callbacks.on_encoded_frame(reader->callback_data,
						stream->stream_id, (uint64_t) pts, map.data, map.size);
		gst_buffer_unmap (buffer, &map);

		if (!ret)
			return GST_PAD_PROBE_DROP;
	}

	return GST_PAD_PROBE_OK;
}

static GstFlowReturn
handle_decoded_video_buffer(recording_loader *reader, recording_loader_stream_ctx *stream, GstBuffer *buffer)
{
	GstMapInfo map = GST_MAP_INFO_INIT;
	GstClockTime pts;
	stream->frames_decoded++;

	pts = GST_BUFFER_PTS(buffer);
	if (!gst_buffer_map (buffer, &map, GST_MAP_READ))
		return GST_FLOW_ERROR;

	g_mutex_lock(&reader->stream_lock);
	GST_DEBUG("stream %d video frame PTS %" GST_TIME_FORMAT, stream->stream_id, GST_TIME_ARGS(pts));

	if (reader->last_pts > pts) {
		GST_WARNING("Time went backward - video frame %" GST_TIME_FORMAT
				" < prev PTS %" GST_TIME_FORMAT, GST_TIME_ARGS(pts),
				GST_TIME_ARGS(reader->last_pts));
	}
	reader->last_pts = pts;

	if (reader->callbacks.on_video_frame) {
		reader->callbacks.on_video_frame(reader->callback_data,
					stream->stream_id, pts, map.data, map.size);
	}
	g_mutex_unlock(&reader->stream_lock);

	gst_buffer_unmap(buffer, &map);

	return GST_FLOW_OK;
}

static GstFlowReturn
handle_json_buffer(recording_loader *reader, recording_loader_stream_ctx *stream, GstBuffer *buffer)
{
	GstMapInfo info = GST_MAP_INFO_INIT;
	char *json_str;
	data_point data_point;
	GstClockTime pts;

	if (!gst_buffer_map (buffer, &info, GST_MAP_READ))
		return GST_FLOW_ERROR;

	/* Unfortunately, GStreamer decides to escape all XML markup in our
	 * JSON strings, so undo that */
	if (stream->xu == NULL)
		stream->xu = xml_unmarkup_new();
	json_str = xml_unmarkup_string(stream->xu, (char *) info.data, info.size);

	if (!json_parse_data_point_string(json_str, &data_point)) {
		g_print("Failed to parse JSON buffer on Stream %d len %" G_GSIZE_FORMAT
		    " TS %" G_GUINT64_FORMAT ": %s\n",
		    stream->stream_id, info.size, GST_BUFFER_PTS(buffer), json_str);
	}
	else {
		pts = GST_BUFFER_PTS(buffer);

		g_mutex_lock(&reader->stream_lock);
		GST_DEBUG("stream %d metadata point PTS %" GST_TIME_FORMAT, stream->stream_id, GST_TIME_ARGS(pts));
#if 0
	fprintf(stderr, "Stream %d JSON buffer with len %" G_GSIZE_FORMAT " TS %" G_GUINT64_FORMAT ": %s\n",
		 stream->stream_id, info.size, GST_BUFFER_PTS(buffer), json_str);
#endif
		if (reader->last_pts > pts) {
			GST_WARNING("Time went backward - metadata point %" GST_TIME_FORMAT " < prev PTS %" GST_TIME_FORMAT, GST_TIME_ARGS(pts), GST_TIME_ARGS(reader->last_pts));
		}
		reader->last_pts = pts;

		if (reader->callbacks.on_event) {
			reader->callbacks.on_event(reader->callback_data, stream->stream_id, pts, &data_point);
		}
		g_mutex_unlock(&reader->stream_lock);
	}

	g_free(json_str);
	gst_buffer_unmap (buffer, &info);
	return GST_FLOW_OK;
}

static GstFlowReturn on_appsink_sample (GstAppSink *appsink, gpointer user_data)
{
	GstSample *sample = gst_app_sink_pull_sample(appsink);
	recording_loader_stream_ctx *stream = user_data;
	recording_loader *reader = stream->reader;
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
				if (stream->is_json) {
					ret = handle_json_buffer(stream->reader, stream, buf);
				}
				else {
					ret = handle_decoded_video_buffer(stream->reader, stream, buf);
				}
			}

			gst_sample_unref (sample);
	}

	return ret;
}

static GstElement *
gen_output (recording_loader *reader, gboolean is_json)
{
	GstAppSinkCallbacks cb = { 0, };
	recording_loader_stream_ctx *stream = NULL;
	GstElement *element = NULL;
	GstElement *decode = NULL;
	GstAppSink *appsink = NULL;
	GstPad *pad;

	cb.eos = on_appsink_eos;
	cb.new_sample = on_appsink_sample;

	stream = g_new0 (recording_loader_stream_ctx, 1);

	stream->reader = reader;
	stream->stream_id = reader->next_stream_id++;
	stream->is_json = is_json;

	reader_add_stream(stream->reader, stream);

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

		gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER, handle_demuxed_video_buffer, stream, NULL);

		gst_object_unref (sinkpad);
	}

	appsink = (GstAppSink *) create_element ("appsink", NULL);
	if (appsink == NULL)
		goto fail;

	gst_app_sink_set_callbacks (appsink, &cb, stream, (GDestroyNotify) recording_loader_stream_ctx_free);
	g_object_set (G_OBJECT(appsink), "async", FALSE, "max-buffers", 1, "sync", FALSE, NULL);

	/* Catch TAG events. We could get these off the bus, but this way works too
	 * and gives more direct access to the stream */
	pad = gst_element_get_static_pad (decode, "sink");
	gst_pad_add_probe (pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, handle_stream_tags, stream, NULL);
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
	recording_loader_stream_ctx_free (stream);

	return NULL;
}

static void
handle_new_output_pad (GstElement * decodebin, GstPad * pad, recording_loader *reader)
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
handle_new_urisource_pad (GstElement *sourcebin, GstPad * pad, recording_loader *reader)
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

recording_loader *recording_loader_new(recording_loader_callbacks *cb, void *cb_data)
{
	struct recording_loader *reader = NULL;

	reader = calloc(1, sizeof(recording_loader));

	g_mutex_init(&reader->stream_lock);
	if (cb)
		reader->callbacks = *cb;
	reader->callback_data = cb_data;

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

bool recording_loader_load(recording_loader *reader, const char *filename_or_uri)
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

	/* Check we have all the stream ids */
	if (!reader_have_all_ids(reader)) {
		g_print("Failed to find stream IDs for all streams. The recording is damaged\n");
		goto done;
	}

	g_print("Collected stream IDs. Starting parsing.\n");
	reader->finding_stream_ids = false;
	reader_announce_streams(reader);

	/* Reset and actually parse things now */
	if (!gst_element_seek_simple (GST_ELEMENT(reader->pipeline), GST_FORMAT_TIME,
			GST_SEEK_FLAG_FLUSH, 0))
		goto done;

	/* Clear all other pending messages */
	gst_bus_set_flushing(reader->bus, TRUE);
	gst_bus_set_flushing(reader->bus, FALSE);

	if (!play_pipeline(reader))
		goto done;

	g_print ("Finished playback. Exiting.\n");
	ret = true;

done:
	return ret;
}

void
recording_loader_free(recording_loader *reader)
{
	/* Free resources */
	if (reader->bus)
		gst_object_unref(reader->bus);

	if (reader->pipeline) {
		gst_element_set_state (reader->pipeline, GST_STATE_NULL);
		gst_object_unref (reader->pipeline);
	}
}

bool recording_loader_init(recording_loader_callbacks *cb, void *cb_data)
{
	/* Initialize GStreamer */
	gst_init (NULL, NULL);

	return true;
}
