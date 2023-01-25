// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * 6DOF Unscented Kalman Filter for positional tracking
 *
 */
#ifndef RIFT_KALMAN_6DOF_V2_H
#define RIFT_KALMAN_6DOF_V2_H

#include "rift.h"
#include "rift-kalman-orient.h"
#include "../ukf.h"


/* Maximum Number of extra lagged state slots to allow for */
#define MAX_DELAY_SLOTS 5

typedef struct rift_kalman_6dof_v2_filter rift_kalman_6dof_v2_filter;

struct rift_kalman_6dof_v2_filter {
  /* 22 element state vector:
   * Inertial frame:
   *  quatd orientation (quaternion) (0:3)
   *
   *  vec3d position (4:6)
   *  vec3d velocity; (7:9)
   *
   * Body frame:
   *  vec3d angular_velocity (10:12)
   *
   *  vec3d accel-bias; (13:15)
   *  vec3d gyro-bias (16:18)
   *
   *  + N lagged slots each:
   *    quatf orientation (quaternion) (0:3)
   *    vec3f position (4:6)
   *
   */
  /* ukf_base needs to be the first element in the struct */
  ukf_base ukf;

  int device_id;

  /* Current time tracking */
  bool first_update;
  uint64_t first_ts;
  uint64_t current_ts;

  /* Control vectors (gyro and accel reading) */
  vec3d ang_vel;
  vec3d lin_accel;

  /* Process noise */
  matrix2d *Q_noise;

  /* Measurment 1: Gravity orientation from IMU accel
   *   vec3f accel
   */
  ukf_measurement m_gravity;

  /* Measurment 2: Pose
   *  vec3f position
   *  quatd orientation
   */
  ukf_measurement m2;

  /* Measurment 3: Position
   *  vec3f position
   */
  ukf_measurement m_position;

  int pose_slot; /* slot number to use for pose update */

  int num_delay_slots;
  bool slot_inuse[MAX_DELAY_SLOTS];

  uint64_t quasi_stationary_ts;
  vec3d quasi_stationary_accel_sum;
  int quasi_stationary_accel_n;
  uint64_t last_imu_update_ts;

  /* Average gravity magnitude */
  double gravity_mean;

	/* Orientation test filter */
	rift_kalman_orient_filter orient_filter;
};

void rift_kalman_6dof_v2_init(rift_kalman_6dof_v2_filter *state, posef *init_pose, int num_delay_slots);
void rift_kalman_6dof_v2_clear(rift_kalman_6dof_v2_filter *state);

void rift_kalman_6dof_v2_prepare_delay_slot(rift_kalman_6dof_v2_filter *state, uint64_t time, int delay_slot);
void rift_kalman_6dof_v2_release_delay_slot(rift_kalman_6dof_v2_filter *state, int delay_slot);

void rift_kalman_6dof_v2_imu_update (rift_kalman_6dof_v2_filter *state, uint64_t time, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void rift_kalman_6dof_v2_pose_update(rift_kalman_6dof_v2_filter *state, uint64_t time, posef *pose, vec3f *pos_error, int delay_slot);
void rift_kalman_6dof_v2_position_update(rift_kalman_6dof_v2_filter *state, uint64_t time, vec3f *position, vec3f *pos_error, int delay_slot);

void rift_kalman_6dof_v2_get_delay_slot_pose_at(rift_kalman_6dof_v2_filter *state, uint64_t time, int delay_slot, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error);
void rift_kalman_6dof_v2_get_pose_at(rift_kalman_6dof_v2_filter *state, uint64_t time, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error);

#endif
