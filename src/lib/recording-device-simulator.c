#include <assert.h>

#include "recording-device-simulator.h"

/* Length of time (milliseconds) we will interpolate position before declaring
 * tracking lost */
#define POSE_LOST_THRESHOLD 500

/* Length of time (milliseconds) we can ignore orientation from cameras before
 * we force an update */
#define POSE_LOST_ORIENT_THRESHOLD 100


rift_tracked_device_simulator *rift_tracked_device_simulator_new(
	int device_id, rift_tracked_device_imu_calibration *imu_calibration,
	posef *imu_pose, posef *model_pose, int num_leds, rift_led *leds)
{
	rift_tracked_device_simulator *dev =
			calloc(1, sizeof(rift_tracked_device_simulator));
	int s;

	/* Rotate our initial pose 180 deg to point along the -Z axis */
	posef init_pose = { .pos = {{ 0.0, 0.0, 0.0 }}, .orient = {{ 0.0, 1.0, 0.0, 0.0 }}};

	dev->base.id = device_id;

	rift_kalman_6dof_init(&dev->ukf_fusion, &init_pose, NUM_POSE_DELAY_SLOTS);
	dev->ukf_fusion.device_id = device_id;

	dev->last_reported_pose = dev->last_observed_orient_ts = dev->last_observed_pose_ts = dev->device_time_ns = 0;

	exp_filter_pose_init(&dev->pose_output_filter);

	/* Init delay slot bookkeeping */
	for (s = 0; s < NUM_POSE_DELAY_SLOTS; s++) {
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + s;

		slot->slot_id = s;
		slot->valid = false;
	}

	/* Compute the device->IMU conversion from the imu->device pose passed */
	dev->device_from_fusion = *imu_pose;
	oposef_inverse(&dev->device_from_fusion);

	/* Compute the IMU->model transform by composing imu->device->model */
	oposef_apply(imu_pose, model_pose, &dev->fusion_from_model);

	/* And the inverse fusion->model conversion */
	dev->model_from_fusion = dev->fusion_from_model;
	oposef_inverse(&dev->model_from_fusion);

	return dev;
}

void rift_tracked_device_simulator_free(rift_tracked_device_simulator *dev)
{
	rift_kalman_6dof_clear(&dev->ukf_fusion);
	free(dev);
}

void rift_tracked_device_simulator_imu_update(rift_tracked_device_simulator *dev,
	uint64_t local_ts, uint64_t device_ts,
	const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field)
{
	dev->device_time_ns = device_ts;
	dev->last_imu_local_ts = local_ts;

	rift_kalman_6dof_imu_update (&dev->ukf_fusion, dev->device_time_ns, ang_vel, accel, mag_field);
}

void rift_tracked_device_simulator_model_pose_update(rift_tracked_device_simulator *dev,
	uint64_t local_ts, uint64_t device_ts, uint64_t frame_start_local_ts, int delay_slot,
	uint32_t score_flags, bool update_position, bool update_orientation,
	posef *model_pose, const char *source)
{
	if (delay_slot == -1)
		return;

	if (!update_position && !update_orientation)
		return;

	posef imu_pose;

	/* Apply the fusion->model pose on top of the passed model->global pose,
	 * to get the global IMU pose */
	oposef_apply(&dev->fusion_from_model, model_pose, &imu_pose);

	if (update_position) {
		if (update_orientation) {
			rift_kalman_6dof_pose_update(&dev->ukf_fusion, dev->device_time_ns, &imu_pose, delay_slot);
			dev->last_observed_orient_ts = dev->device_time_ns;
		} else {
			rift_kalman_6dof_position_update(&dev->ukf_fusion, dev->device_time_ns, &imu_pose.pos, delay_slot);
		}

		dev->last_observed_pose_ts = dev->device_time_ns;
		dev->last_observed_pose = imu_pose;
	}
}

void rift_tracked_device_simulator_on_exposure (rift_tracked_device_simulator *dev, uint64_t local_ts,
	uint64_t device_ts, uint64_t exposure_ts, int delay_slot)
{
	if (delay_slot != -1) {
		rift_tracker_pose_delay_slot *slot = &dev->delay_slots[delay_slot];

		dev->device_time_ns = device_ts;

		slot->device_time_ns = dev->device_time_ns;
		slot->valid = true;
		slot->use_count = 0;
		slot->n_pose_reports = 0;
		slot->n_used_reports = 0;

		/* Tell the kalman filter to prepare the delay slot */
		rift_kalman_6dof_prepare_delay_slot(&dev->ukf_fusion, dev->device_time_ns, delay_slot);
	}
}

void rift_tracked_device_simulator_frame_captured (rift_tracked_device_simulator *dev, uint64_t local_ts,
	uint64_t frame_start_local_ts, int delay_slot, const char *source)
{
	if (delay_slot != -1) {
		assert (delay_slot < NUM_POSE_DELAY_SLOTS);
		if (dev->delay_slots[delay_slot].valid)
			dev->delay_slots[delay_slot].use_count++;
	}
}

void rift_tracked_device_simulator_frame_release (rift_tracked_device_simulator *dev, uint64_t local_ts,
	uint64_t frame_local_ts, int delay_slot, const char *source)
{
	if (delay_slot != -1) {
		rift_tracker_pose_delay_slot *slot = &dev->delay_slots[delay_slot];

		if (slot->use_count > 0) {
			slot->use_count--;
		}
	}
}

void rift_tracked_device_simulator_get_model_pose(rift_tracked_device_simulator *dev, uint64_t local_ts,
		posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel)
{
	posef imu_global_pose, model_pose;
	vec3f imu_ang_vel = { 0, };
	vec3f imu_vel = { 0, }, imu_accel = { 0, };

	rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns,
			&imu_global_pose, &imu_vel, &imu_accel, &imu_ang_vel, NULL, NULL);

	/* Apply the pose conversion from IMU->model */
	oposef_apply(&dev->model_from_fusion, &imu_global_pose, &model_pose);

	dev->reported_pose.orient = model_pose.orient;
	if (dev->device_time_ns - dev->last_observed_pose_ts >= (POSE_LOST_THRESHOLD * 1000000UL)) {
					/* Don't let the device move unless there's a recent observation of actual position */
					model_pose.pos = dev->reported_pose.pos;
					imu_vel.x = imu_vel.y = imu_vel.z = 0.0;
					imu_accel.x = imu_accel.y = imu_accel.z = 0.0;
	}

	exp_filter_pose_run(&dev->pose_output_filter, dev->device_time_ns, &model_pose, &dev->reported_pose);
	dev->last_reported_pose = dev->device_time_ns;

	/* Angular Velocity and acceleration need rotating into the device space.
	 * Linear velocity should also acquire a component from angular velocity */
	oquatf_get_rotated(&dev->model_from_fusion.orient, &imu_ang_vel, &dev->reported_ang_vel);
	oquatf_get_rotated(&dev->model_from_fusion.orient, &imu_accel, &dev->reported_lin_accel);

	/* Linear velocity generated by the angular velocity at the IMU offset
	 * is the cross product of the (rotated) position and the angular
	 * velocity */
	vec3f rotated_imu_pos, extra_lin_vel;
	oquatf_get_rotated(&dev->model_from_fusion.orient, &dev->model_from_fusion.pos, &rotated_imu_pos);
	ovec3f_cross(&dev->reported_ang_vel, &rotated_imu_pos, &extra_lin_vel);

	oquatf_get_rotated(&dev->model_from_fusion.orient, &imu_vel, &dev->reported_lin_vel);
	ovec3f_add(&dev->reported_lin_vel, &dev->reported_lin_vel, &extra_lin_vel);

	if (pose)
		*pose = dev->reported_pose;
	if (ang_vel)
		*ang_vel = dev->reported_ang_vel;
	if (accel)
		*accel = dev->reported_lin_accel;
	if (vel)
		*vel = dev->reported_lin_vel;
}

/* Rift pose search calls this to get updates of the latest pose, which makes
 * sense in the real-time system with asynchronous and overlapping pose updates
 * happening, but for here where all pose search is synchronous, we don't
 * update anything and just return TRUE
 */
bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev_base, rift_tracked_device_exposure_info *dev_info)
{
	return true;
}
