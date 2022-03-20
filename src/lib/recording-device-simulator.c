#include "recording-device-simulator.h"

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

void rift_tracked_device_simulator_imu_update(rift_tracked_device_simulator *dev,
	uint64_t local_ts, uint64_t device_ts,
	const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field)
{
	dev->device_time_ns = device_ts;
	dev->last_imu_local_ts = local_ts;

	rift_kalman_6dof_imu_update (&dev->ukf_fusion, dev->device_time_ns, ang_vel, accel, mag_field);
}

