#include <stdint.h>
#include <stdbool.h>

#include "omath.h"
#include "rift-tracker-common.h"
#include "rift-sensor-device.h"
#include "rift-sensor-maths.h"

#ifndef __RECORDING_DATA_H__
#define __RECORDING_DATA_H__

#define RIFT_SENSOR_SERIAL_LEN 32

typedef enum data_point_type data_point_type;
typedef struct data_point data_point;

enum data_point_type {
    DATA_POINT_DEVICE_ID,
    DATA_POINT_SENSOR_CONFIG,
    DATA_POINT_SENSOR_POSE,
    DATA_POINT_IMU,
    DATA_POINT_EXPOSURE,
    DATA_POINT_POSE,
    DATA_POINT_OUTPUT_POSE,
    DATA_POINT_FRAME_START,
    DATA_POINT_FRAME_CAPTURED,
    DATA_POINT_FRAME_RELEASE
};

#define MAX_LED_COUNT 45

extern const char *data_point_type_names[];

struct data_point {
    data_point_type data_type;
    uint64_t ts;

    union {
      struct {
        int device_id;

        rift_tracked_device_imu_calibration imu_calibration;

        posef imu_pose;
        posef model_pose;

        int num_leds;
        rift_led leds[MAX_LED_COUNT];
      } device_id;

      struct {
        char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
        char stream_id[64];
        bool is_cv1;
        dmat3 camera_matrix;
        double dist_coeffs[5];
      } sensor_config;

      struct {
        char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
        posef pose;
      } sensor_pose;

      /* IMU observation */
      struct {
        uint64_t local_ts;
        uint64_t device_ts;
        float dt;
        vec3f ang_vel;
        vec3f accel;
        vec3f mag;
        quatf simple_orient;
      } imu;

      struct {
        uint64_t local_ts;
        uint32_t hmd_ts;
        uint32_t exposure_ts;
        uint32_t exposure_count;
        uint64_t device_ts;
        int delay_slot;

        posef capture_pose;

        vec3f capture_rot_error;
        vec3f capture_pos_error;
      } exposure;

      struct {
        uint64_t local_ts;
        char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
      } frame_start;

      struct {
        uint64_t local_ts;
        uint64_t frame_local_ts;
        char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
        int delay_slot;
      } frame_captured;

      struct {
        uint64_t local_ts;
        uint64_t frame_local_ts;
        char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
        int delay_slot;
      } frame_release;

      /* Model pose (not IMU - needs pose transform before fusion) */
      struct {
        uint64_t local_ts;
        uint64_t device_ts;
        uint64_t frame_device_ts;
        uint64_t frame_start_local_ts;
        uint64_t exposure_local_ts;
        uint32_t exposure_hmd_ts;
        uint32_t exposure_count;
        int delay_slot;
        char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
        uint32_t score_flags;
        bool update_position;
        bool update_orientation;

        posef pose;
        posef capture_pose;

        vec3f capture_rot_error;
        vec3f capture_pos_error;
      } pose;

      struct {
        uint64_t local_ts;
        uint64_t device_ts;
        uint64_t last_imu_local_ts;

        posef pose;

        vec3f linear_velocity;
        vec3f linear_accel;
      } output_pose;
    };
};

bool json_parse_data_point_string(const char *json_data, data_point *point);

#endif
