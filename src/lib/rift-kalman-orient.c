// Copyright 2020-2022, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * Unscented Kalman Filter for orientation tracking
 *
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "rift-kalman-orient.h"

#define DUMP_COV_UPDATES 0

#define GRAVITY_MAG 9.80665

/* IMU biases noise levels */
#define IMU_GYRO_BIAS_NOISE 3e-12 /* gyro bias (rad/s)^2 */
#define IMU_GYRO_BIAS_NOISE_INITIAL 1e-3 /* gyro bias (rad/s)^2 */

#if 0
#define IMU_ACCEL_NOISE 0.001  // Manually tuned
#define IMU_GYRO_NOISE  0.0076 // Manually tuned
#elif 1
// bmi055 datasheet values
#define IMU_ACCEL_NOISE (2.245e-05 * GRAVITY_MAG) /* bmi055 full 1000Hz bandwidth noise, in m/s^2 = 2.220725e-04 */
#define IMU_GYRO_NOISE  (DEG_TO_RAD(0.221215)) /* bmi055 230Hz bandwidth noise, in rad/s = 3.86093e-03 */
#else
// MPU6050 values
#define IMU_ACCEL_NOISE (4.837355e-05 * GRAVITY_MAG) /* MPU6050 full 260Hz bandwidth noise, in m/s^2 = 4.74382e-04 */
#define IMU_GYRO_NOISE  (DEG_TO_RAD(0.158114)) /* MPU6050 250Hz bandwidth noise, in rad/s = 2.7596e-03 */
#endif

#define POSE_PROCESS_NOISE (1e-4) /* (rad/s)^2 */

typedef struct imu_filter_state imu_filter_state;

/* The state vector is bigger than the covariance
 * matrices, because it includes the quaternion, but
 * the covariance is parameterised with an exponential
 * map */
static const int BASE_STATE_SIZE = 13;
static const int BASE_COV_SIZE = 12;

/* A lagged slot consists of orientation */
static const int DELAY_SLOT_STATE_SIZE = 4;
static const int DELAY_SLOT_COV_SIZE = 3;

#define STATE_ORIENTATION 0
#define STATE_GYRO_BIAS 4

/* Augmented noise entries for the bias and gyro inputs */
#define STATE_GYRO_BIAS_NOISE 7
#define STATE_GYRO_NOISE 10

/* Indices into covariance / noise matrices for the state */
#define COV_ORIENTATION 0
#define COV_GYRO_BIAS 3

#define COV_GYRO_BIAS_NOISE 6
#define COV_GYRO_NOISE 9

/* Indices into a delay slot for orientation */
#define DELAY_SLOT_STATE_ORIENTATION 0

#define DELAY_SLOT_COV_ORIENTATION 0

/* Indices into the IMU gravity measurement vector */
#define GRAVITY_MEAS_ORIENT 0

/* Indices into the pose measurement vector */
#define POSE_MEAS_ORIENTATION 0

#define NS_TO_SEC(t) ((double)(t) / 1000000000.0)

#if 0
static void print_col_vec(const char *label, uint64_t ts, const matrix2d *mat)
{
	int i;

	if (label)
		printf ("%s, ", label);

	printf ("%lu, ", ts);

	for (i = 0; i < mat->rows-1; i++) {
		printf ("%10.4f, ", MATRIX2D_Y(mat, i));
	}
	printf ("%10.4f\n", MATRIX2D_Y(mat, i));
}
#endif

static void delta_quat_from_ang_vel(vec3d *ang_vel, double dt, quatd *delta_q)
{
	double ang_vel_length = ovec3d_get_length (ang_vel);
	double rot_angle = ang_vel_length * dt;

	if (rot_angle == 0.0) {
			delta_q->x = delta_q->y = delta_q->z = 0.0;
			delta_q->w = 1.0;
			return;
	}

	vec3d rot_axis = {{ ang_vel->x / ang_vel_length, ang_vel->y / ang_vel_length, ang_vel->z / ang_vel_length }};
	oquatd_init_axis(delta_q, &rot_axis, rot_angle);
}

static bool process_func(const ukf_base *ukf, const double dt, const matrix2d *X_prior, matrix2d *X)
{
	/*
	 *	quatd orientation (quaternion) (0:3)
	 *
	 *	vec3d gyro-bias (7:9)
   *	vec3d gyro-noise (13:15)
	 */
	rift_kalman_orient_filter *filter_state = (rift_kalman_orient_filter *)(ukf);

	/* Most of the state stays constant - constant acceleration, constant angular velocity, biases -
	 * so copy the whole state to start, then adjust the predicted part */
	if (X != X_prior)
		matrix2d_copy(X, X_prior);

	/* Compute orientation update correctly using a delta quat from ang_vel */
	quatd orient = {{ .x = MATRIX2D_Y(X_prior, STATE_ORIENTATION),
	                  .y = MATRIX2D_Y(X_prior, STATE_ORIENTATION+1),
	                  .z = MATRIX2D_Y(X_prior, STATE_ORIENTATION+2),
	                  .w = MATRIX2D_Y(X_prior, STATE_ORIENTATION+3) }};

	vec3d imu_ang_vel;
	vec3d ang_vel_bias;

	ang_vel_bias.x = MATRIX2D_Y(X, STATE_GYRO_BIAS) + MATRIX2D_Y(X, STATE_GYRO_BIAS_NOISE);
	ang_vel_bias.y = MATRIX2D_Y(X, STATE_GYRO_BIAS+1) + MATRIX2D_Y(X, STATE_GYRO_BIAS_NOISE+1);
	ang_vel_bias.z = MATRIX2D_Y(X, STATE_GYRO_BIAS+2) + MATRIX2D_Y(X, STATE_GYRO_BIAS_NOISE+2);

	vec3d ang_vel_noise = {{
		MATRIX2D_Y(X, STATE_GYRO_NOISE),
		MATRIX2D_Y(X, STATE_GYRO_NOISE+1),
		MATRIX2D_Y(X, STATE_GYRO_NOISE+2),
	}};

	/* Subtract estimated IMU bias from the ang vel */
	ovec3d_subtract (&filter_state->ang_vel, &ang_vel_bias, &imu_ang_vel);
	/* And add the noise */
	ovec3d_add (&imu_ang_vel, &ang_vel_noise, &imu_ang_vel);

	/* Update the orientation from angular velocity */
	quatd delta_orient;
	delta_quat_from_ang_vel(&imu_ang_vel, dt, &delta_orient);
	oquatd_mult_me(&orient, &delta_orient);
	oquatd_normalize_me(&orient);

#if 1
	printf("Orient Device %d dt %f ang vel %f %f %f - bias %f %f %f + noise %f %f %f orient %f %f %f %f delta %f %f %f %f out %f %f %f %f\n",
		filter_state->device_id, dt,
		imu_ang_vel.x, imu_ang_vel.y, imu_ang_vel.z,
		ang_vel_bias.x, ang_vel_bias.y, ang_vel_bias.z,
		ang_vel_noise.x, ang_vel_noise.y, ang_vel_noise.z,
		MATRIX2D_Y(X_prior, STATE_ORIENTATION), MATRIX2D_Y(X_prior, STATE_ORIENTATION+1),
		MATRIX2D_Y(X_prior, STATE_ORIENTATION+2), MATRIX2D_Y(X_prior, STATE_ORIENTATION+3),
		delta_orient.x, delta_orient.y, delta_orient.z, delta_orient.w,
		orient.x, orient.y, orient.z, orient.w);
#endif

	MATRIX2D_Y(X, STATE_ORIENTATION) = orient.x;
	MATRIX2D_Y(X, STATE_ORIENTATION+1) = orient.y;
	MATRIX2D_Y(X, STATE_ORIENTATION+2) = orient.z;
	MATRIX2D_Y(X, STATE_ORIENTATION+3) = orient.w;

	/* Update Bias to include the noise */
	MATRIX2D_Y(X, STATE_GYRO_BIAS)   = ang_vel_bias.x;
	MATRIX2D_Y(X, STATE_GYRO_BIAS+1) = ang_vel_bias.y;
	MATRIX2D_Y(X, STATE_GYRO_BIAS+2) = ang_vel_bias.z;

	return true;
}

static bool calc_quat_mean(const matrix2d *sigmas, int quat_index, const matrix2d *weights, matrix2d *mean)
{
	/* Compute barycentric mean of quaternion component at 'quat_index' iteratively */
	vec3d error_s;

	/* Start with the prior mean as the initial quat */
	quatd mean_orient = {{ MATRIX2D_XY(sigmas, quat_index, 0),
	                       MATRIX2D_XY(sigmas, quat_index+1, 0),
	                       MATRIX2D_XY(sigmas, quat_index+2, 0),
	                       MATRIX2D_XY(sigmas, quat_index+3, 0) }};
	quatd delta_q;
	int s, iters = 0;

	do {
		error_s.x = error_s.y = error_s.z = 0.0;
		for (s = 1; s < sigmas->cols; s++) {
#if 1
			double w_M = 1.0 / sigmas->cols;
#else
			double w_M = MATRIX2D_Y(weights, s);
#endif

			quatd cur_q = {{ MATRIX2D_XY(sigmas, quat_index, s),
			                 MATRIX2D_XY(sigmas, quat_index+1, s),
			                 MATRIX2D_XY(sigmas, quat_index+2, s),
			                 MATRIX2D_XY(sigmas, quat_index+3, s) }};
			vec3d delta_rot;

			oquatd_diff(&mean_orient, &cur_q, &delta_q);
			oquatd_normalize_me(&delta_q);
			oquatd_to_rotation(&delta_q, &delta_rot);

			error_s.x += w_M * delta_rot.x;
			error_s.y += w_M * delta_rot.y;
			error_s.z += w_M * delta_rot.z;
		}

		/* Correct the mean using the error */
		oquatd_from_rotation(&delta_q, &error_s);

		quatd tmp_q = mean_orient;
		oquatd_mult(&tmp_q, &delta_q, &mean_orient);

		if (iters++ > 100) {
			printf("Quaternion mean failed\n");
			return false; /* Not converging */
		}
	} while (ovec3d_get_length(&error_s) > (1e-4));

	MATRIX2D_Y(mean, quat_index)   = mean_orient.x;
	MATRIX2D_Y(mean, quat_index+1) = mean_orient.y;
	MATRIX2D_Y(mean, quat_index+2) = mean_orient.z;
	MATRIX2D_Y(mean, quat_index+3) = mean_orient.w;

	return true;
}

/* Callback function that takes a set of sigma points (N_sigmaxN_state) and weights (1xN_sigma) and computes the mean (1xN_state) */
static bool state_mean_func(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean)
{
	rift_kalman_orient_filter *filter_state = (rift_kalman_orient_filter *)(ut);
	int i;

	/* Compute euclidean barycentric mean, then fix up the quaternion component */
	/* FIXME: Skip the quaternion parts of this multiplication to save some cycles? */
	if (matrix2d_multiply (mean, sigmas, weights) != MATRIX_RESULT_OK) {
		return false;
	}

	if (!calc_quat_mean(sigmas, STATE_ORIENTATION, weights, mean))
		return false;

	/* Delay slots */
	for (i = 0; i < filter_state->num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);

		if (filter_state->slot_inuse[i]) {
			/* Quaternion */
			if (!calc_quat_mean(sigmas, slot_index + DELAY_SLOT_STATE_ORIENTATION, weights, mean))
				return false;
		}
		else {
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION) = 0.0;
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION+1) = 0.0;
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION+2) = 0.0;
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION+3) = 1.0;
		}
	}
	return true;
}

static void calc_quat_residual(const matrix2d *X, const matrix2d *Y, int state_index, matrix2d *residual, int cov_index)
{
	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	X_q.x = MATRIX2D_Y(X, state_index);
	X_q.y = MATRIX2D_Y(X, state_index+1);
	X_q.z = MATRIX2D_Y(X, state_index+2);
	X_q.w = MATRIX2D_Y(X, state_index+3);

	Y_q.x = MATRIX2D_Y(Y, state_index);
	Y_q.y = MATRIX2D_Y(Y, state_index+1);
	Y_q.z = MATRIX2D_Y(Y, state_index+2);
	Y_q.w = MATRIX2D_Y(Y, state_index+3);

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	MATRIX2D_Y(residual, cov_index+0) = delta_rot.x;
	MATRIX2D_Y(residual, cov_index+1) = delta_rot.y;
	MATRIX2D_Y(residual, cov_index+2) = delta_rot.z;
}

static bool state_residual_func(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual)
{
	rift_kalman_orient_filter *filter_state = (rift_kalman_orient_filter *)(ut);
	int i, j;

	/* Quaternion component */
	calc_quat_residual(X, Y, STATE_ORIENTATION, residual, COV_ORIENTATION);

	/* Euclidean portion */
	for (i = STATE_GYRO_BIAS, j = COV_GYRO_BIAS; i < BASE_STATE_SIZE; i++, j++)
		MATRIX2D_Y(residual, j) = MATRIX2D_Y(X, i) - MATRIX2D_Y(Y, i);

	/* Delay slots */
	for (i = 0; i < filter_state->num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);
		int cov_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * i);

		if (filter_state->slot_inuse[i]) {
			/* Quaternion */
			calc_quat_residual(X, Y, slot_index + DELAY_SLOT_STATE_ORIENTATION, residual, cov_index + DELAY_SLOT_COV_ORIENTATION);
		}
		else {
			for (j = 0; j < DELAY_SLOT_COV_SIZE; j++)
				MATRIX2D_Y(residual, cov_index + j) = 0.0;
		}
	}

	return true;
}

static void calc_quat_sum(const matrix2d *Y, int state_index, const matrix2d *addend, int cov_index, matrix2d *X)
{
	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = MATRIX2D_Y(Y, state_index);
	out_q.y = MATRIX2D_Y(Y, state_index+1);
	out_q.z = MATRIX2D_Y(Y, state_index+2);
	out_q.w = MATRIX2D_Y(Y, state_index+3);

	delta_rot.x = MATRIX2D_Y(addend, cov_index);
	delta_rot.y = MATRIX2D_Y(addend, cov_index+1);
	delta_rot.z = MATRIX2D_Y(addend, cov_index+2);

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);
	oquatd_normalize_me(&out_q);

	MATRIX2D_Y(X, state_index) = out_q.x;
	MATRIX2D_Y(X, state_index+1) = out_q.y;
	MATRIX2D_Y(X, state_index+2) = out_q.z;
	MATRIX2D_Y(X, state_index+3) = out_q.w;
}

static bool state_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	rift_kalman_orient_filter *filter_state = (rift_kalman_orient_filter *)(ut);
	int i, j;

	/* Quaternion component */
	calc_quat_sum(Y, STATE_ORIENTATION, addend, COV_ORIENTATION, X);

	/* Euclidean portion */
	for (i = STATE_GYRO_BIAS, j = COV_GYRO_BIAS; i < BASE_STATE_SIZE; i++, j++)
		MATRIX2D_Y(X, i) = MATRIX2D_Y(Y, i) + MATRIX2D_Y(addend, j);

	/* Delay slots */
	for (i = 0; i < filter_state->num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);
		int cov_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * i);

		if (filter_state->slot_inuse[i]) {
			/* Quaternion */
			calc_quat_sum(Y, slot_index + DELAY_SLOT_STATE_ORIENTATION, addend, cov_index + DELAY_SLOT_COV_ORIENTATION, X);
		}
		else {
			for (j = 0; j < DELAY_SLOT_STATE_SIZE; j++)
				MATRIX2D_Y(X, slot_index + j) = 0.0;
			MATRIX2D_Y(X, slot_index + DELAY_SLOT_STATE_ORIENTATION + 3) = 1.0;
		}
	}

	return true;
}

static bool gravity_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *X, matrix2d *z)
{
	/* Extract the orientation due to gravity by decomposing the swing around the gravity direction */
	quatd orient = (quatd) {{
	    .x = MATRIX2D_Y(X, STATE_ORIENTATION),
	    .y = MATRIX2D_Y(X, STATE_ORIENTATION+1),
	    .z = MATRIX2D_Y(X, STATE_ORIENTATION+2),
	    .w = MATRIX2D_Y(X, STATE_ORIENTATION+3)
		}};

	const vec3d gravity_vector = {{ 0.0, 1.0, 0.0 }};
	quatd pose_gravity_swing, pose_gravity_twist;

	oquatd_decompose_swing_twist(&orient, &gravity_vector, &pose_gravity_swing, &pose_gravity_twist);

	MATRIX2D_Y(z, GRAVITY_MEAS_ORIENT)   = pose_gravity_swing.x;
	MATRIX2D_Y(z, GRAVITY_MEAS_ORIENT+1) = pose_gravity_swing.y;
	MATRIX2D_Y(z, GRAVITY_MEAS_ORIENT+2) = pose_gravity_swing.z;
	MATRIX2D_Y(z, GRAVITY_MEAS_ORIENT+3) = pose_gravity_swing.w;

	printf ("orient %f %f %f %f swing %f %f %f %f\n",
		orient.x, orient.y, orient.z, orient.w,
		pose_gravity_swing.x, pose_gravity_swing.y, pose_gravity_swing.z, pose_gravity_swing.w);

#if 0
	rift_kalman_orient_filter *state = (rift_kalman_orient_filter *)(ukf);
	print_col_vec("gravity measurement prediction (z_bar)", state->current_ts, z);
#endif

	return true;
}

static bool gravity_mean_func(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean)
{
	if (!calc_quat_mean(sigmas, GRAVITY_MEAS_ORIENT, weights, mean))
		return false;

	return true;
}

/* Do a non-linear addition of quaternion */
static bool gravity_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT);
	out_q.y = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT+1);
	out_q.z = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT+2);
	out_q.w = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT+3);

	delta_rot.x = MATRIX2D_Y(addend, 0);
	delta_rot.y = MATRIX2D_Y(addend, 1);
	delta_rot.z = MATRIX2D_Y(addend, 2);

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);

	MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT) = out_q.x;
	MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT+1) = out_q.y;
	MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT+2) = out_q.z;
	MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT+3) = out_q.w;

	return true;
}

static bool gravity_residual_func(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual)
{
	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	X_q.x = MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT);
	X_q.y = MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT+1);
	X_q.z = MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT+2);
	X_q.w = MATRIX2D_Y(X, GRAVITY_MEAS_ORIENT+3);

	Y_q.x = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT);
	Y_q.y = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT+1);
	Y_q.z = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT+2);
	Y_q.w = MATRIX2D_Y(Y, GRAVITY_MEAS_ORIENT+3);

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	MATRIX2D_Y(residual, 0) = delta_rot.x;
	MATRIX2D_Y(residual, 1) = delta_rot.y;
	MATRIX2D_Y(residual, 2) = delta_rot.z;

	return true;
}

static bool pose_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *x, matrix2d *z)
{
	rift_kalman_orient_filter *filter_state = (rift_kalman_orient_filter *)(ukf);
	int state_orientation_index = STATE_ORIENTATION;

	if (filter_state->pose_slot != -1) {
		/* A delay slot was set, get the measurement from it */
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * filter_state->pose_slot);
		state_orientation_index = slot_index + DELAY_SLOT_STATE_ORIENTATION;
	}
	/* Measure orientation */
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION)   = MATRIX2D_Y(x, state_orientation_index);
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION+1) = MATRIX2D_Y(x, state_orientation_index+1);
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION+2) = MATRIX2D_Y(x, state_orientation_index+2);
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION+3) = MATRIX2D_Y(x, state_orientation_index+3);

	// print_col_vec("Pose measurement prediction (z_bar)", -1, z);

	return true;
}

/* Callback function that takes a set of sigma points (N_sigmaxN_measurement and weights (1xN_sigma) and computes the
 * mean (1xN_measurement) */
static bool pose_mean_func(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean)
{
	if (!calc_quat_mean(sigmas, POSE_MEAS_ORIENTATION, weights, mean))
		return false;

	return true;
}

static bool pose_residual_func(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual)
{
	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	X_q.x = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION);
	X_q.y = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+1);
	X_q.z = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+2);
	X_q.w = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+3);

	Y_q.x = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION);
	Y_q.y = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+1);
	Y_q.z = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+2);
	Y_q.w = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+3);

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	MATRIX2D_Y(residual, 0) = delta_rot.x;
	MATRIX2D_Y(residual, 1) = delta_rot.y;
	MATRIX2D_Y(residual, 2) = delta_rot.z;

	return true;
}

/* The addend here is a 3 value vector of orientation expoential,
 * to be added to the 4-value measurement vector non-linearly */
static bool pose_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION);
	out_q.y = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+1);
	out_q.z = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+2);
	out_q.w = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+3);

	delta_rot.x = MATRIX2D_Y(addend, 0);
	delta_rot.y = MATRIX2D_Y(addend, 1);
	delta_rot.z = MATRIX2D_Y(addend, 2);

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);

	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION) = out_q.x;
	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+1) = out_q.y;
	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+2) = out_q.z;
	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+3) = out_q.w;

	return true;
}

void rift_kalman_orient_init(rift_kalman_orient_filter *state, posef *init_pose, int num_delay_slots)
{
	int i;

	assert(num_delay_slots <= MAX_DELAY_SLOTS);

	state->first_update = true;
	state->num_delay_slots = num_delay_slots;
	state->quasi_stationary_ts = 0;
	state->gravity_mean = GRAVITY_MAG;

	ovec3d_set (&state->quasi_stationary_accel_sum, 0.0, 0.0, 0.0);
	state->quasi_stationary_accel_n = 0;

	const int STATE_SIZE = BASE_STATE_SIZE + (num_delay_slots * DELAY_SLOT_STATE_SIZE);
	const int COV_SIZE = BASE_COV_SIZE + (num_delay_slots * DELAY_SLOT_COV_SIZE);

	/* Allocate process noise matrix */
	state->Q_noise = matrix2d_alloc0 (COV_SIZE, COV_SIZE);
	for (i = COV_ORIENTATION; i < COV_ORIENTATION + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = POSE_PROCESS_NOISE;

	/* Takes ownership of Q_noise */
	ukf_base_init(&state->ukf, STATE_SIZE, COV_SIZE, state->Q_noise, process_func, state_mean_func, state_residual_func, state_sum_func);
  //state->ukf.verbose = true;

	/* Initialise the state with the init_pose */
	for (i = 0; i < 4; i++)
		MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION + i) = init_pose->orient.arr[i];

	for (i = 0; i < num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);
		MATRIX2D_Y(state->ukf.x_prior, slot_index + DELAY_SLOT_STATE_ORIENTATION + 3) = 1.0;
		state->slot_inuse[i] = false;
	}

	/* Initialise the prior covariance / uncertainty - particularly around the biases,
	 * where we assume they are close to 0 somewhere */
	for (i = COV_GYRO_BIAS; i < COV_GYRO_BIAS + 3; i++)
		MATRIX2D_XY(state->ukf.P_prior, i, i) = IMU_GYRO_BIAS_NOISE_INITIAL;

	/* IMU measurement - gravity vector from accel */
	ukf_measurement_init(&state->m_gravity, 4, 3, &state->ukf, gravity_measurement_func, gravity_mean_func, gravity_residual_func, gravity_sum_func);

	/* This R matrix is chosen heuristically to trigger a gradual correction of gravity alignment */
	for (int i = 0; i < 3; i++)
	 MATRIX2D_XY(state->m_gravity.R, i, i) = 0.01;

	/* m2 is for pose measurements - orientation. */
	ukf_measurement_init(&state->m2, 4, 3, &state->ukf, pose_measurement_func, pose_mean_func, pose_residual_func, pose_sum_func);
	MATRIX2D_XY(state->m2.R, 0, 0) = (DEG_TO_RAD(60) * DEG_TO_RAD(60)); /* degrees std dev (don't trust observations much for X/Z, 20 degrees for yaw) */
	MATRIX2D_XY(state->m2.R, 1, 1) = (DEG_TO_RAD(20) * DEG_TO_RAD(20)); /* Y */
	MATRIX2D_XY(state->m2.R, 2, 2) = (DEG_TO_RAD(60) * DEG_TO_RAD(60)); /* Z */

	state->pose_slot = -1;

	ovec3d_set(&state->ang_vel, 0.0, 0.0, 0.0);
}

void rift_kalman_orient_clear(rift_kalman_orient_filter *state)
{
	ukf_measurement_clear(&state->m2);
	ukf_measurement_clear(&state->m_gravity);
	ukf_base_clear(&state->ukf);
}

static void reset_noise_entry(rift_kalman_orient_filter *state, matrix2d *X, matrix2d *P, int state_index, int cov_index, double noise_variance)
{
	int i;
	matrix_result ret;
	matrix2d tmp;

	/* Zero out the co-variance column and row entries, then
	 * set the variances on the diagonal, then clear the mean back to 0.0 */
	ret = matrix2d_submatrix_ref(P, cov_index, 0, 3, P->cols, &tmp);
	assert (ret ==	MATRIX_RESULT_OK);
	ret = matrix2d_fill(&tmp, 0.0);
	assert (ret ==	MATRIX_RESULT_OK);

	ret = matrix2d_submatrix_ref(P, 0, cov_index, P->rows, 3, &tmp);
	assert (ret ==	MATRIX_RESULT_OK);
	ret = matrix2d_fill(&tmp, 0.0);
	assert (ret ==	MATRIX_RESULT_OK);

	for (i = cov_index; i < cov_index + 3; i++)
		MATRIX2D_XY(P, i, i) = noise_variance;

	for (i = state_index; i < state_index + 3; i++)
		MATRIX2D_Y(X, i) = 0.0;
}

static void reset_noise(rift_kalman_orient_filter *state, matrix2d *X, matrix2d *P)
{
	/* Reset the augmented control vector covariance noise entries */
	reset_noise_entry(state, X, P, STATE_GYRO_BIAS_NOISE, COV_GYRO_BIAS_NOISE, IMU_GYRO_BIAS_NOISE*IMU_GYRO_BIAS_NOISE);
	reset_noise_entry(state, X, P, STATE_GYRO_NOISE, COV_GYRO_NOISE, IMU_GYRO_NOISE*IMU_GYRO_NOISE);
}

static void
rift_kalman_orient_update(rift_kalman_orient_filter *state, uint64_t time, ukf_measurement *m)
{
	/* Calculate dt */
	double dt = 0;
	if (time != 0) {
		if (state->first_update) {
			dt = 0;
			state->first_update = false;
			state->first_ts = time;
			printf("orient filter Device %d gyro bias noise %f initial %f \n"
			  " pose process noise %f\n",
				state->device_id, IMU_GYRO_BIAS_NOISE, IMU_GYRO_BIAS_NOISE_INITIAL,
				POSE_PROCESS_NOISE);
		}
		else {
			dt = NS_TO_SEC((int64_t)(time - state->current_ts));
		}
		state->current_ts = time;
	}

	reset_noise(state, state->ukf.x_prior, state->ukf.P_prior);
	printf("======== Orient Device %d BEGIN delay_slot %d dt %f ==========\n", state->device_id, state->pose_slot, dt);
	if (!ukf_base_predict(&state->ukf, dt)) {
		LOGE ("Failed to compute UKF prediction at time %llu (dt %f)", (unsigned long long) state->current_ts, dt);
		return;
	}
	printf("======== Orient Device %d END predict ==========\n", state->device_id);

	if (m) {
		if (!ukf_base_update(&state->ukf, m)) {
			LOGE ("Failed to perform %s UKF update at time %llu (dt %f)",
						m == &state->m_gravity ? "IMU" : "Pose", (unsigned long long) state->current_ts, dt);
			return;
		}
	}
	else {
		/* Pseudo-measurement - just commit the predicted state */
		if (!ukf_base_commit(&state->ukf)) {
			LOGE ("Failed to commit UKF prediction at time %llu (dt %f)",
					(unsigned long long) state->current_ts, dt);
			return;
		}
	}
	printf("======== Orient Device %d END update ==========\n", state->device_id);

	// print_col_vec ("UKF Mean after update", state->current_ts, state->ukf.x_prior);
}

#if DUMP_COV_UPDATES
static void print_mat(const char *label, const matrix2d *mat)
{
    int i, j;

    if (label)
        printf ("%s: %u rows, %u cols [\n", label, mat->rows, mat->cols);

    for (i = 0; i < mat->rows; i++) {
        printf ("  [");
        for (j = 0; j < mat->cols-1; j++) {
            printf ("%10.5f, ", MATRIX2D_XY(mat, i, j));
        }
        printf ("%10.5f ],\n", MATRIX2D_XY(mat, i, j));
    }
    printf("]\n");
}
#endif

void rift_kalman_orient_prepare_delay_slot(rift_kalman_orient_filter *state, uint64_t time, int delay_slot)
{
#if DUMP_COV_UPDATES
	printf ("Initialising slot %d dt %f\n",
		delay_slot, NS_TO_SEC((int64_t)(time - state->current_ts)));

	if (delay_slot == 0) {
		print_mat("Prior State", state->ukf.x_prior);
		print_mat("Prior Cov", state->ukf.P_prior);
	}
#endif

	/* If time is not the current time, project the state forward to the timestamp */
	if (time != state->current_ts)
		rift_kalman_orient_update(state, time, NULL);

	/* set up the lagged slot by cloning state and covariance blocks. We
	 * need to copy the state variables across, and the rows + columns
	 * of the primary covariance entries */

	/* Copy the rows from the state to the lagged state */
	if (matrix2d_copy_block_in_place(state->ukf.x_prior,
	    STATE_ORIENTATION, 0, 4, 1,
	    BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * delay_slot), 0) != MATRIX_RESULT_OK)
	{
			LOGE ("Failed to clone UKF state to delay slot %d at time %llu",
			    delay_slot, (unsigned long long) time);
			return;
	}

	/* Copying the covariance is a little more complex. We need to copy
	 * the block of rows (AA to CC) and block of columns (AA to BB)
	 * from the primary state
	 * to the lagged slot, then copy the covariance of the main
	 * block (AA) too. There's some duplication here, but it's easier
	 * than copying each piece separately
	 *
	 * +-------------------+
	 * +AA============BB===+
	 * +||............||...+
	 * +||............||...+
	 * +CC============AA===+
	 * +||............||...+
	 * +-------------------+
	 */

	/* Copy the rows */
	if (matrix2d_copy_block_in_place(state->ukf.P_prior,
	    COV_ORIENTATION, 0, 3, state->ukf.P_prior->cols,
	    BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot), 0) != MATRIX_RESULT_OK)
	{
	  LOGE ("Failed to clone UKF covariance rows to delay slot %d at time %llu",
				delay_slot, (unsigned long long) time);
		return;
	}

	/* Copy the columns */
	if (matrix2d_copy_block_in_place(state->ukf.P_prior,
	    0, COV_ORIENTATION, state->ukf.P_prior->rows, 3,
	    0, BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot)) != MATRIX_RESULT_OK)
	{
	  LOGE ("Failed to clone UKF covariance columns to delay slot %d at time %llu",
				delay_slot, (unsigned long long) time);
		return;
	}

	/* Copy the block */
	if (matrix2d_copy_block_in_place(state->ukf.P_prior,
	    COV_ORIENTATION, COV_ORIENTATION, 3, 3,
	    BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot),
	    BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot)) != MATRIX_RESULT_OK)
	{
	  LOGE ("Failed to clone UKF covariance to delay slot %d at time %llu",
				delay_slot, (unsigned long long) time);
		return;
	}

#if 1
	/* Delay slots require some noise added t6o keep the matrix positive definite */
	int i;
	int cov_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot);
	for (i = 0; i < DELAY_SLOT_COV_SIZE; i++)
		MATRIX2D_XY(state->ukf.P_prior, cov_index + i, cov_index + i) += 1e-14;
#endif

	state->slot_inuse[delay_slot] = true;

#if DUMP_COV_UPDATES
	if (delay_slot == 0) {
		printf ("Initialised slot %d\n", delay_slot);
		print_mat("State", state->ukf.x_prior);
		print_mat("Cov", state->ukf.P_prior);
	}
#endif
}

void rift_kalman_orient_release_delay_slot(rift_kalman_orient_filter *state, int delay_slot)
{
	state->slot_inuse[delay_slot] = false;
}

static bool orient_quat_from_gravity(quatd *orient, const vec3d *accel)
{
	// Calculate a cross product between what the device
	// thinks is up and what gravity indicates is down.
	// The values are optimized of what we would get out
	// from the cross product.
	const vec3d up = {{0, 1.0, 0}};

	vec3d gravity_vec = *accel;
	vec3d tilt_axis = {{accel->z, 0, -accel->x}};

	/* Check if we're already closely aligned to gravity */
	float tilt_angle = ovec3d_get_angle(&up, &gravity_vec);
	if (tilt_angle < 0.002)
		return false;

	ovec3d_normalize_me(&tilt_axis);
	ovec3d_normalize_me(&gravity_vec);

	oquatd_init_axis(orient, &tilt_axis, tilt_angle);

	quatd pose_gravity_swing, pose_gravity_twist;
	oquatd_decompose_swing_twist(orient, &up, &pose_gravity_swing, &pose_gravity_twist);

  printf("gravity %f %f %f %f vs swing %f %f %f %f\n",
    orient->x, orient->y, orient->z, orient->w,
    pose_gravity_swing.x, pose_gravity_swing.y, pose_gravity_swing.z, pose_gravity_swing.w);

	return true;
}

static double
oquatd_get_angle(const quatd *q1, const quatd *q2)
{
	quatd diff;

	oquatd_diff(q1, q2, &diff);
	return 2 * acos(diff.w);
}

static float
oquatf_get_angle(const quatf *q1, const quatf *q2)
{
	quatf diff;

	oquatf_diff(q1, q2, &diff);
	return 2 * acos(diff.w);
}


void rift_kalman_orient_imu_update (rift_kalman_orient_filter *state, uint64_t time, const vec3f* ang_vel, const vec3d* unbiased_accel, const vec3f* mag_field)
{
	/* Put angular velocity and accel into the input vector */
	state->ang_vel.x = ang_vel->x;
	state->ang_vel.y = ang_vel->y;
	state->ang_vel.z = ang_vel->z;

#if 1
	quatd orient_prior = {{ .x = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION),
		                  .y = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION+1),
		                  .z = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION+2),
		                  .w = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION+3) }};
	vec3d global_accel = {{ 0, 1.0, 0 }};
	quatd pose_gravity_swing, pose_gravity_twist;
	quatd pose_gravity_swing_prior;

	vec3d gravity_prior;

	/* Move global accel into the IMU body frame and decompose */
	oquatd_inverse(&orient_prior);
	oquatd_decompose_swing_twist(&orient_prior, &global_accel, &pose_gravity_swing_prior, &pose_gravity_twist);
	oquatd_get_rotated(&pose_gravity_swing_prior, &global_accel, &gravity_prior);
	ovec3d_multiply_scalar(&gravity_prior, state->gravity_mean, &gravity_prior);
#endif

	vec3d unbiased_gyro, gyro_bias;

	matrix2d *X = state->ukf.x_prior;

	gyro_bias.x = MATRIX2D_Y(X, STATE_GYRO_BIAS);
	gyro_bias.y = MATRIX2D_Y(X, STATE_GYRO_BIAS+1);
	gyro_bias.z = MATRIX2D_Y(X, STATE_GYRO_BIAS+2);

	ovec3d_subtract (&state->ang_vel, &gyro_bias, &unbiased_gyro);

	/* and accel */
	if (state->quasi_stationary_ts == 0 ||
			fabs(ovec3d_get_length(unbiased_accel) - state->gravity_mean) > 0.5f ||
			ovec3d_get_length(&unbiased_gyro) > 1.0f) {
#if 1
		printf("Orient Device %d TS %f (%f) Resetting quasi stationary ts after %f with accel %f gyro %f bias %f %f %f\n",
				state->device_id, NS_TO_SEC(time), NS_TO_SEC((int64_t)(time - state->first_ts)),
				NS_TO_SEC(time - state->quasi_stationary_ts),
				ovec3d_get_length(unbiased_accel) - state->gravity_mean,
				ovec3d_get_length(&unbiased_gyro),
				gyro_bias.x, gyro_bias.y, gyro_bias.z);
#endif
		state->quasi_stationary_ts = time;
		ovec3d_set (&state->quasi_stationary_accel_sum, 0.0, 0.0, 0.0);
		state->quasi_stationary_accel_n = 0;
	}
	else {
		ovec3d_add (&state->quasi_stationary_accel_sum, unbiased_accel, &state->quasi_stationary_accel_sum);
		state->quasi_stationary_accel_n++;
	}

	/* If it's been quasi stationary for more than 20ms, do a correction every 10 samples, averaging them */
	quatd imu_gravity_orient;

	if (0 && !state->first_update &&
			(time - state->quasi_stationary_ts >= 20000000 && state->quasi_stationary_accel_n >= 10) &&
			orient_quat_from_gravity(&imu_gravity_orient, &state->quasi_stationary_accel_sum)) {

		/* Use the accumulated accelerometer value to average out gravity over the last 10 samples */
		vec3d imu_gravity = state->quasi_stationary_accel_sum;

		/* Update the mean gravity */
		ovec3d_multiply_scalar(&imu_gravity, 1.0 / state->quasi_stationary_accel_n, &imu_gravity);
		printf("Orient gravity_mean %f + %f\n", state->gravity_mean, ovec3d_get_length(&imu_gravity));
		state->gravity_mean = 0.999 * state->gravity_mean + 0.001 * ovec3d_get_length (&imu_gravity);

#if 1
		printf("Orient Device %d TS %f (%f) IMU measurement after %f with gravity mean %f mag %f vec %f %f %f "
				"accel mag %f gyro %f bias %f %f %f gravity prior %f %f %f (error %f) orient cov %f %f %f\n",
				state->device_id, NS_TO_SEC(time), NS_TO_SEC((int64_t)(time - state->first_ts)),
				NS_TO_SEC(time - state->last_imu_update_ts),
				state->gravity_mean, ovec3d_get_length(&imu_gravity),
				imu_gravity.x, imu_gravity.y, imu_gravity.z,
				ovec3d_get_length(unbiased_accel) - state->gravity_mean,
				ovec3d_get_length(&unbiased_gyro),
				gyro_bias.x, gyro_bias.y, gyro_bias.z,
				gravity_prior.x, gravity_prior.y, gravity_prior.z,
        ovec3d_get_angle(&imu_gravity, &gravity_prior),
				MATRIX2D_XY(state->ukf.P_prior, COV_ORIENTATION, COV_ORIENTATION),
				MATRIX2D_XY(state->ukf.P_prior, COV_ORIENTATION+1, COV_ORIENTATION+1),
				MATRIX2D_XY(state->ukf.P_prior, COV_ORIENTATION+2, COV_ORIENTATION+2));

		imu_gravity = state->quasi_stationary_accel_sum;
#endif

		/* Put normalized acceleration in the measurement vector to correct the orientation by gravity */
		ukf_measurement *m = &state->m_gravity;

		ovec3d_normalize_me(&imu_gravity);
		MATRIX2D_Y(m->z, GRAVITY_MEAS_ORIENT+0) = imu_gravity_orient.x;
		MATRIX2D_Y(m->z, GRAVITY_MEAS_ORIENT+1) = imu_gravity_orient.y;
		MATRIX2D_Y(m->z, GRAVITY_MEAS_ORIENT+2) = imu_gravity_orient.z;
		MATRIX2D_Y(m->z, GRAVITY_MEAS_ORIENT+3) = imu_gravity_orient.w;

		/* FIXME: Do a measurement only if the device has been stable / quasi-stationary long enough */
		rift_kalman_orient_update(state, time, m);
		state->last_imu_update_ts = time;

		/* Restart the quasi-stationary timer too */
		//state->quasi_stationary_ts = time;
		ovec3d_set (&state->quasi_stationary_accel_sum, 0.0, 0.0, 0.0);
		state->quasi_stationary_accel_n = 0;

	} else {
#if 1
		orient_quat_from_gravity(&imu_gravity_orient, unbiased_accel);
#endif
		// Apply the process model, but no measurement
		rift_kalman_orient_update(state, time, NULL);
	}

#if 1
		quatd orient_post = {{ .x = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION),
		                  .y = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION+1),
		                  .z = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION+2),
		                  .w = MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION+3) }};
		//vec3d global_accel = {{ 0, 1.0, 0 }};

		/* Move global accel into the IMU body frame */
		vec3d new_gravity;
		oquatd_inverse(&orient_post);
	  oquatd_decompose_swing_twist(&orient_post, &global_accel, &pose_gravity_swing, &pose_gravity_twist);
		oquatd_get_rotated(&pose_gravity_swing, &global_accel, &new_gravity);
		ovec3d_multiply_scalar(&new_gravity, state->gravity_mean, &new_gravity);

		//orient_quat_from_gravity(&orient_prior, &gravity_prior);
		//orient_quat_from_gravity(&orient_post, &new_gravity);

		printf("Orient Device %d TS %f (%f) post IMU measurement f vec %f %f %f cov %f %f %f\n",
				state->device_id, NS_TO_SEC(time), NS_TO_SEC((int64_t)(time - state->first_ts)),
				new_gravity.x, new_gravity.y, new_gravity.z,
				MATRIX2D_XY(state->ukf.P_prior, COV_ORIENTATION, COV_ORIENTATION),
				MATRIX2D_XY(state->ukf.P_prior, COV_ORIENTATION+1, COV_ORIENTATION+1),
				MATRIX2D_XY(state->ukf.P_prior, COV_ORIENTATION+2, COV_ORIENTATION+2));

		printf("Orient Device %d TS %f (%f) IMU ang_vel %f %f %f accel %f %f %f orient %f %f %f %f prior %f %f %f %f error %f post %f %f %f %f error %f\n",
				state->device_id, NS_TO_SEC(time), NS_TO_SEC((int64_t)(time - state->first_ts)),
				unbiased_gyro.x, unbiased_gyro.y, unbiased_gyro.z,
				unbiased_accel->x, unbiased_accel->y, unbiased_accel->z,
				imu_gravity_orient.x, imu_gravity_orient.y, imu_gravity_orient.z, imu_gravity_orient.w,
				pose_gravity_swing_prior.x, pose_gravity_swing_prior.y, pose_gravity_swing_prior.z, pose_gravity_swing_prior.w,
				oquatd_get_angle(&pose_gravity_swing_prior, &imu_gravity_orient),
				pose_gravity_swing.x, pose_gravity_swing.y, pose_gravity_swing.z, pose_gravity_swing.w,
				oquatd_get_angle(&pose_gravity_swing, &imu_gravity_orient));
#endif
}

void rift_kalman_orient_pose_update(rift_kalman_orient_filter *state, uint64_t time, posef *pose, int delay_slot)
{
	/* Use lagged state vector entries to correct for delay */
	state->pose_slot = delay_slot;

	posef prior_pose;
  rift_kalman_orient_get_delay_slot_pose_at(state, time, delay_slot, &prior_pose, NULL, NULL);
	printf("Orient Device %d TS %f rel %f dt %f Pose update delay slot %d orient %f %f %f %f (prior orient %f %f %f %f error %f)\n",
			state->device_id, NS_TO_SEC(time), NS_TO_SEC((int64_t)(time - state->first_ts)),
			NS_TO_SEC((int64_t)(time - state->current_ts)), delay_slot, 
			pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
			prior_pose.orient.x, prior_pose.orient.y, prior_pose.orient.z, prior_pose.orient.w,
			oquatf_get_angle(&prior_pose.orient, &pose->orient));

	/* If doing a delayed update, then the slot must be in use (
	 * or else it contains empty data */
	if (delay_slot != -1) {
		assert(state->slot_inuse[delay_slot]);
		/* HACK: Ignore the time when doing a delay slot update,
		 * since we don't want time to go backward. The delay slot is
		 * already tracking the delay */
		time = 0;
	 }

	ukf_measurement *m = &state->m2;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+0) = pose->orient.x;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+1) = pose->orient.y;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+2) = pose->orient.z;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+3) = pose->orient.w;

	rift_kalman_orient_update(state, time, m);
}

/* Get the pose info from a delay slot, or the main state
 * if delay_slot = -1 */
void rift_kalman_orient_get_delay_slot_pose_at(rift_kalman_orient_filter *state, uint64_t time, int delay_slot, posef *pose,
  vec3f *ang_vel, vec3f *rot_error)
{
	matrix2d *x = state->ukf.x_prior;
	matrix2d *P = state->ukf.P_prior; /* Covariance */

	int state_orientation_index = STATE_ORIENTATION;
	int cov_orientation_index = COV_ORIENTATION;

	if (delay_slot != -1) {
	  assert(state->slot_inuse[delay_slot]);
	  assert(delay_slot < MAX_DELAY_SLOTS);

		/* We want the position / orientation info from a delay slot */
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * delay_slot);
		state_orientation_index = slot_index + DELAY_SLOT_STATE_ORIENTATION;

		slot_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot);
		cov_orientation_index = slot_index + DELAY_SLOT_COV_ORIENTATION;
	}

	/* FIXME: Do prediction using the time? */
	quatd orient = { .x = MATRIX2D_Y(x, state_orientation_index),
	                 .y = MATRIX2D_Y(x, state_orientation_index+1),
	                 .z = MATRIX2D_Y(x, state_orientation_index+2),
	                 .w = MATRIX2D_Y(x, state_orientation_index+3)
	};

	pose->orient.x = orient.x;
	pose->orient.y = orient.y;
	pose->orient.z = orient.z;
	pose->orient.w = orient.w;

	if (ang_vel) {
		ang_vel->x = state->ang_vel.x;
		ang_vel->y = state->ang_vel.y;
		ang_vel->z = state->ang_vel.z;
	}

	if (rot_error) {
		rot_error->x = sqrtf(MATRIX2D_XY(P, cov_orientation_index, cov_orientation_index));
		rot_error->y = sqrtf(MATRIX2D_XY(P, cov_orientation_index+1, cov_orientation_index+1));
		rot_error->z = sqrtf(MATRIX2D_XY(P, cov_orientation_index+2, cov_orientation_index+2));
	}
}

void rift_kalman_orient_get_pose_at(rift_kalman_orient_filter *state, uint64_t time, posef *pose,
  vec3f *ang_vel, vec3f *rot_error)
{
	rift_kalman_orient_get_delay_slot_pose_at(state, time, -1, pose, ang_vel, rot_error);
}
