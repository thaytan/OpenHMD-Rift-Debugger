#include <stdint.h>
#include <stdbool.h>

#include "omath.h"

#ifndef __RECORDING_DATA_H__
#define __RECORDING_DATA_H__

typedef enum imu_data_point_type imu_data_point_type;
typedef struct imu_data_point imu_data_point;

enum imu_data_point_type {
    DATA_POINT_DEVICE_ID,
    DATA_POINT_IMU,
    DATA_POINT_EXPOSURE,
    DATA_POINT_POSE,
    DATA_POINT_FRAME_START,
    DATA_POINT_FRAME_CAPTURED,
    DATA_POINT_FRAME_RELEASE
};

struct imu_data_point {
    imu_data_point_type data_type;
    union {
      struct {
        int device_id;
      } id;

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
      } exposure;

      struct {
        uint64_t local_ts;
      } frame_start;

      struct {
        uint64_t local_ts;
        uint64_t frame_local_ts;
        int delay_slot;
      } frame_captured;

      struct {
        uint64_t local_ts;
        uint64_t frame_local_ts;
        int delay_slot;
      } frame_release;

      struct {
        uint64_t local_ts;
        uint64_t device_ts;
        uint64_t frame_device_ts;
        uint64_t frame_start_local_ts;
        uint64_t exposure_local_ts;
        uint32_t exposure_hmd_ts;
        uint32_t exposure_count;
        int delay_slot;
        posef pose;
      } pose;
    };
};

bool imu_read_data_file(const char *fname, imu_data_point **points, uint32_t *num_points);

#endif
