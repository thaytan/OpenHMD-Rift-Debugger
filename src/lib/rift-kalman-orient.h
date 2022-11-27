// Copyright 2020-2022, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * Unscented Kalman Filter for orientation tracking
 *
 */
#ifndef RIFT_KALMAN_ORIENT_H
#define RIFT_KALMAN_ORIENT_H

#include "rift.h"
#include "../ukf.h"

/* Maximum Number of extra lagged state slots to allow for */
#define MAX_DELAY_SLOTS 5

typedef struct rift_kalman_orient_filter rift_kalman_orient_filter;

struct rift_kalman_orient_filter {
  /* ukf_base needs to be the first element in the struct */
  ukf_base ukf;

  int device_id;

  /* Current time tracking */
  bool first_update;
  uint64_t first_ts;
  uint64_t current_ts;

  /* Control vector (gyro reading) */
  vec3d ang_vel;

  /* Process noise */
  matrix2d *Q_noise;

  /* Measurment 1: Gravity orientation from IMU accel
   *   vec3f accel
   */
  ukf_measurement m_gravity;

  /* Measurment 2: Pose
   *  quatd orientation
   */
  ukf_measurement m2;

  int pose_slot; /* slot number to use for pose update */

  int num_delay_slots;
  bool slot_inuse[MAX_DELAY_SLOTS];

  uint64_t quasi_stationary_ts;
  vec3d quasi_stationary_accel_sum;
  int quasi_stationary_accel_n;
  uint64_t last_imu_update_ts;

  /* Average gravity magnitude */
  double gravity_mean;
};

void rift_kalman_orient_init(rift_kalman_orient_filter *state, posef *init_pose, int num_delay_slots);
void rift_kalman_orient_clear(rift_kalman_orient_filter *state);

void rift_kalman_orient_prepare_delay_slot(rift_kalman_orient_filter *state, uint64_t time, int delay_slot);
void rift_kalman_orient_release_delay_slot(rift_kalman_orient_filter *state, int delay_slot);

void rift_kalman_orient_imu_update (rift_kalman_orient_filter *state, uint64_t time, const vec3f* ang_vel, const vec3d* accel, const vec3f* mag_field);
void rift_kalman_orient_pose_update(rift_kalman_orient_filter *state, uint64_t time, posef *pose, int delay_slot);

void rift_kalman_orient_get_delay_slot_pose_at(rift_kalman_orient_filter *state, uint64_t time, int delay_slot, posef *pose, vec3f *ang_vel, vec3f *rot_error);
void rift_kalman_orient_get_pose_at(rift_kalman_orient_filter *state, uint64_t time, posef *pose, vec3f *ang_vel, vec3f *rot_error);

#endif
