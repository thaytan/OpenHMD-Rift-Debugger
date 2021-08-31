#include <json-glib/json-glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "recording-data.h"

struct json_parse_data {
  imu_data_point *points;
  GError *error;
  gboolean warned_about_device_ts;
};


static bool
parse_json_vec3f(vec3f *out, guint index, JsonObject *obj, const char *field_name, GError **error) {
  JsonArray *field = json_object_get_array_member(obj, field_name);

  if (field == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u has no '%s' array field", index, field_name);
    return false;
  }
  if (json_array_get_length (field) != 3) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u field '%s' does not have 3 members", index, field_name);
    return false;
  }

  out->x = json_array_get_double_element (field, 0);
  out->y = json_array_get_double_element (field, 1);
  out->z = json_array_get_double_element (field, 2);
  return true;
}

static bool
parse_json_quatf(quatf *out, guint index, JsonObject *obj, const char *field_name, GError **error) {
  JsonArray *field = json_object_get_array_member(obj, field_name);

  if (field == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u has no '%s' array field", index, field_name);
    return false;
  }
  if (json_array_get_length (field) != 4) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u field '%s' does not have 4 members", index, field_name);
    return false;
  }

  out->x = json_array_get_double_element (field, 0);
  out->y = json_array_get_double_element (field, 1);
  out->z = json_array_get_double_element (field, 2);
  out->w = json_array_get_double_element (field, 3);
  return true;
}

static void
parse_json_point_node(JsonArray *array, guint index, JsonNode *node, struct json_parse_data *parse_data)
{
  imu_data_point *point = parse_data->points + index;

  if (parse_data->error != NULL)
    return; /* Stop parsing - an error occurred */

  if (!JSON_NODE_HOLDS_OBJECT(node)) {
    g_set_error(&parse_data->error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u is not a JSON object", index);
    return;
  }

  JsonObject *obj = json_node_get_object(node);
  const char *obj_type = json_object_get_string_member (obj, "type");

  if (obj_type == NULL) {
    g_set_error(&parse_data->error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u doesn't have a type field", index);
    return;
  }

  if (g_str_equal(obj_type, "device")) {
    point->data_type = DATA_POINT_DEVICE_ID;
    point->id.device_id = json_object_get_int_member(obj, "device-id");
  }
  else if (g_str_equal(obj_type, "imu")) {
    point->data_type = DATA_POINT_IMU;
    point->imu.local_ts = json_object_get_int_member(obj, "local-ts");
    point->imu.device_ts = json_object_get_int_member(obj, "device-ts");
    point->imu.dt = json_object_get_double_member(obj, "dt");

    if (!parse_json_vec3f(&point->imu.ang_vel, index, obj, "ang_vel", &parse_data->error))
      return;
    if (!parse_json_vec3f(&point->imu.accel, index, obj, "accel", &parse_data->error))
      return;
    if (!parse_json_vec3f(&point->imu.mag, index, obj, "mag", &parse_data->error))
      return;
    if (json_object_has_member(obj, "simple-orient"))
      parse_json_quatf(&point->imu.simple_orient, index, obj, "simple-orient", NULL);
    else
      memset(&point->imu.simple_orient, 0, sizeof(point->imu.simple_orient));
  }
  else if (g_str_equal(obj_type, "exposure")) {
    point->data_type = DATA_POINT_EXPOSURE;
    point->exposure.local_ts = json_object_get_int_member(obj, "local-ts");
    point->exposure.hmd_ts = json_object_get_int_member(obj, "hmd-ts");
    point->exposure.exposure_ts = json_object_get_int_member(obj, "exposure-ts");
    point->exposure.exposure_count = json_object_get_int_member(obj, "count");
    point->exposure.device_ts = json_object_get_int_member(obj, "device-ts");
    point->exposure.delay_slot = json_object_get_int_member(obj, "delay-slot");
  }
  else if (g_str_equal(obj_type, "pose")) {
    point->data_type = DATA_POINT_POSE;
    point->pose.local_ts = json_object_get_int_member(obj, "local-ts");
    point->pose.device_ts = json_object_get_int_member(obj, "device-ts");
    point->pose.frame_start_local_ts = json_object_get_int_member(obj, "frame-start-local-ts");
    point->pose.exposure_local_ts = json_object_get_int_member(obj, "frame-local-ts");
    point->pose.exposure_hmd_ts = json_object_get_int_member(obj, "frame-hmd-ts");
    point->pose.exposure_count = json_object_get_int_member(obj, "frame-exposure-count");

    if (json_object_has_member(obj, "frame-device-ts")) {
      point->pose.frame_device_ts = json_object_get_int_member(obj, "frame-device-ts");
    } else {
      if (!parse_data->warned_about_device_ts) {
        printf("Trace log poses don't have frame-device-ts. Estimating using local ts\n");
        parse_data->warned_about_device_ts = TRUE;
      }
      point->pose.frame_device_ts = point->pose.device_ts - (point->pose.local_ts - point->pose.exposure_local_ts);
    }
    point->pose.delay_slot = json_object_get_int_member(obj, "frame-fusion-slot");

    if (!parse_json_vec3f(&point->pose.pose.pos, index, obj, "pos", &parse_data->error))
      return;
    if (!parse_json_quatf(&point->pose.pose.orient, index, obj, "orient", &parse_data->error))
      return;
  }
  else if (g_str_equal(obj_type, "frame-start")) {
    point->data_type = DATA_POINT_FRAME_START;
    point->frame_start.local_ts = json_object_get_int_member(obj, "local-ts");
  }
  else if (g_str_equal(obj_type, "frame-captured")) {
    point->data_type = DATA_POINT_FRAME_CAPTURED;
    point->frame_captured.local_ts = json_object_get_int_member(obj, "local-ts");
    point->frame_captured.frame_local_ts = json_object_get_int_member(obj, "frame-start-local-ts");
    point->frame_captured.delay_slot = json_object_get_int_member(obj, "delay-slot");
  }
  else if (g_str_equal(obj_type, "frame-release")) {
    point->data_type = DATA_POINT_FRAME_RELEASE;
    point->frame_release.local_ts = json_object_get_int_member(obj, "local-ts");
    point->frame_release.frame_local_ts = json_object_get_int_member(obj, "frame-local-ts");
    point->frame_release.delay_slot = json_object_get_int_member(obj, "delay-slot");
  }
  else {
    g_set_error(&parse_data->error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "Sample point %u has unknown type '%s'", index, obj_type);
  }
}

bool imu_read_data_file(const char *fname, imu_data_point **points, uint32_t *num_points)
{
  JsonParser *parser = json_parser_new();
  JsonNode *root = NULL;
  struct json_parse_data parse_data = { 0, };

  if (!json_parser_load_from_file (parser, fname, &parse_data.error))
  {
    printf ("Parsing failre: %s\n", parse_data.error->message);
    g_error_free (parse_data.error);
    return false;
  }

  *num_points = 0;
  *points = NULL;

  root = json_parser_get_root(parser);
  if (!JSON_NODE_HOLDS_ARRAY(root)) {
    printf ("Failed to read JSON file '%s'. Top-level node should be an array\n", fname);
    goto fail;
  }

  JsonArray *points_array = json_node_get_array(root);
  guint points_array_length = json_array_get_length(points_array);
  if (points_array_length < 1) {
    printf ("Failed to read JSON file '%s'. Top-level array is empty.\n", fname);
    goto fail;
  }

  parse_data.points = calloc(points_array_length, sizeof(imu_data_point));

  json_array_foreach_element (points_array, (JsonArrayForeach) (parse_json_point_node), &parse_data);

  if (parse_data.error != NULL) {
    printf ("Failed to read JSON file '%s'. %s\n", fname, parse_data.error->message);
    g_error_free (parse_data.error);

    free (*points);
    *points = NULL;
    goto fail;
  }

  *points = parse_data.points;
  *num_points = points_array_length;

  g_object_unref(parser);
  return true;

fail:
  g_object_unref(parser);
  return false;
}
