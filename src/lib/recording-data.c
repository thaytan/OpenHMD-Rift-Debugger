#include <json-glib/json-glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "recording-data.h"

static bool
parse_json_vec3f(vec3f *out, JsonObject *obj, const char *field_name, GError **error) {
  JsonArray *field = json_object_get_array_member(obj, field_name);

  if (field == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record has no '%s' array field", field_name);
    return false;
  }
  if (json_array_get_length (field) != 3) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record field '%s' does not have 3 members", field_name);
    return false;
  }

  out->x = json_array_get_double_element (field, 0);
  out->y = json_array_get_double_element (field, 1);
  out->z = json_array_get_double_element (field, 2);
  return true;
}

static bool
parse_json_quatf(quatf *out, JsonObject *obj, const char *field_name, GError **error) {
  JsonArray *field = json_object_get_array_member(obj, field_name);

  if (field == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record has no '%s' array field", field_name);
    return false;
  }
  if (json_array_get_length (field) != 4) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record field '%s' does not have 4 members", field_name);
    return false;
  }

  out->x = json_array_get_double_element (field, 0);
  out->y = json_array_get_double_element (field, 1);
  out->z = json_array_get_double_element (field, 2);
  out->w = json_array_get_double_element (field, 3);
  return true;
}

static bool
parse_json_double_array(double *out, JsonObject *obj, const char *field_name, int n, GError **error) {
  JsonArray *field = json_object_get_array_member(obj, field_name);
  int i;

  if (field == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record has no '%s' array field", field_name);
    return false;
  }

  if (json_array_get_length (field) != n) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED,
          "JSON record field '%s' does not have %d members", field_name, n);
    return false;
  }

  for (i = 0; i < n; i++)
    out[i] = json_array_get_double_element (field, i);

  return true;
}

static bool
parse_json_dmat3(dmat3 *out, JsonObject *obj, const char *field_name, GError **error) {
  return parse_json_double_array(out->m, obj, field_name, 9, error);
}

static bool
parse_string_val(JsonObject *obj, const char *field_name, char *dest, guint dest_len, GError **error)
{
  const char *str_val;

  str_val = json_object_get_string_member (obj, field_name);
  if (str_val == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED,
        "JSON sensor-pose record has no \"%s\" field", field_name);
    return false;
  }

  strncpy(dest, str_val, dest_len);
  dest[dest_len] = '\0';

  return true;
}

static bool
parse_json_point_node(JsonNode *node, data_point *point, GError **error)
{
  JsonObject *obj = json_node_get_object(node);
  const char *obj_type = json_object_get_string_member (obj, "type");

  if (obj_type == NULL) {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record doesn't have a type field");
    return false;
  }

  if (g_str_equal(obj_type, "device")) {
    point->data_type = DATA_POINT_DEVICE_ID;
    point->device_id.device_id = json_object_get_int_member(obj, "device-id");
  }
  else if (g_str_equal(obj_type, "imu")) {
    point->data_type = DATA_POINT_IMU;
    point->imu.local_ts = json_object_get_int_member(obj, "local-ts");
    point->imu.device_ts = json_object_get_int_member(obj, "device-ts");
    point->imu.dt = json_object_get_double_member(obj, "dt");

    if (!parse_json_vec3f(&point->imu.ang_vel, obj, "ang_vel", error))
      return false;
    if (!parse_json_vec3f(&point->imu.accel, obj, "accel", error))
      return false;
    if (!parse_json_vec3f(&point->imu.mag, obj, "mag", error))
      return false;
    if (json_object_has_member(obj, "simple-orient"))
      parse_json_quatf(&point->imu.simple_orient, obj, "simple-orient", NULL);
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
      printf("Trace log poses don't have frame-device-ts.\n");
      g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON pose record has no frame-device-ts");
      return false;
    }
    point->pose.delay_slot = json_object_get_int_member(obj, "frame-fusion-slot");

    if (!parse_json_vec3f(&point->pose.pose.pos, obj, "pos", error))
      return false;
    if (!parse_json_quatf(&point->pose.pose.orient, obj, "orient", error))
      return false;
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
  else if (g_str_equal(obj_type, "sensor-config")) {
    point->data_type = DATA_POINT_SENSOR_CONFIG;

    if (!parse_string_val(obj, "device-id", point->sensor_config.serial_no, RIFT_SENSOR_SERIAL_LEN, error))
      return false;
    if (!parse_string_val(obj, "video-stream-id", point->sensor_config.stream_id, 63, error))
      return false;

    point->sensor_config.is_cv1 = json_object_get_int_member(obj, "is-cv1");

    if (!parse_json_dmat3(&point->sensor_config.camera_matrix, obj, "camera-matrix", error))
      return false;

    if (!parse_json_double_array(point->sensor_config.dist_coeffs, obj, "dist-coeffs", 5, error))
      return false;

  }
  else if (g_str_equal(obj_type, "sensor-pose")) {
    point->data_type = DATA_POINT_SENSOR_POSE;

    if (!parse_string_val(obj, "device-id", point->sensor_pose.serial_no, RIFT_SENSOR_SERIAL_LEN, error))
      return false;
    if (!parse_json_vec3f(&point->sensor_pose.pose.pos, obj, "pos", error))
      return false;
    if (!parse_json_quatf(&point->sensor_pose.pose.orient, obj, "orient", error))
      return false;
  }
  else {
    g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_FAILED, "JSON record has unknown type '%s'", obj_type);
    return false;
  }

  return true;
}

bool json_parse_data_point_string(const char *json_data, data_point *point)
{
  JsonParser *parser = json_parser_new();
  JsonNode *root = NULL;
  GError *error = NULL;
  bool ret;

  if (!json_parser_load_from_data (parser, json_data, -1, &error))
  {
    printf ("Parsing failure: %s\n", error->message);
    g_error_free (error);
    return false;
  }

  root = json_parser_get_root(parser);
  if (!JSON_NODE_HOLDS_OBJECT(root)) {
    printf ("Failed to parse JSON string '%s'. Top-level node should be an object\n", json_data);
    goto fail;
  }

  ret = parse_json_point_node(root, point, &error);
  if (error != NULL) {
    printf ("Failed to read JSON data '%s'. %s\n", json_data, error->message);
    g_error_free (error);
    goto fail;
  }

  g_object_unref(parser);
  return ret;

fail:
  g_object_unref(parser);
  return ret;
}
