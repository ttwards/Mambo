/*
 * Copyright (c) 2026 ttwards
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/wheel.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define FLOAT_TOLERANCE 0.0001f

#define MECANUM_DEV           DEVICE_DT_GET(DT_NODELABEL(mecanum0))
#define MECANUM_MOTOR_DEV     DEVICE_DT_GET(DT_NODELABEL(mecanum_motor))
#define STEER_DEV             DEVICE_DT_GET(DT_NODELABEL(steer0))
#define STEER_MOTOR_DEV       DEVICE_DT_GET(DT_NODELABEL(steer_motor))
#define STEER_DRIVE_DEV       DEVICE_DT_GET(DT_NODELABEL(steer_drive_motor))
#define CHASSIS_DEV           DEVICE_DT_GET(DT_NODELABEL(chassis0))
#define CHASSIS_WHEEL0_DEV    DEVICE_DT_GET(DT_NODELABEL(chassis_wheel0))
#define CHASSIS_WHEEL1_DEV    DEVICE_DT_GET(DT_NODELABEL(chassis_wheel1))
#define CHASSIS_WHEEL2_DEV    DEVICE_DT_GET(DT_NODELABEL(chassis_wheel2))
#define CHASSIS_WHEEL3_DEV    DEVICE_DT_GET(DT_NODELABEL(chassis_wheel3))
#define EXPECTED_RPM_1MPS_R01 (RADPS_TO_RPM / 0.1f)

void cchassis_resolve(chassis_data_t *data, const chassis_cfg_t *cfg);

struct test_motor_data {
	motor_status_t status;
	motor_setpoint_t last_setpoint;
	enum motor_cmd last_cmd;
	uint32_t set_count;
	uint32_t control_count;
};

struct test_wheel_data {
	wheel_status_t status;
	wheel_status_t target;
	float last_speed;
	float last_angle;
	float last_static_angle;
	uint32_t set_speed_count;
	uint32_t set_static_count;
	uint32_t disable_count;
	int static_return;
};

static void assert_float_close(float actual, float expected)
{
	zassert_true(fabsf(actual - expected) < FLOAT_TOLERANCE, "actual=%f expected=%f",
		     (double)actual, (double)expected);
}

static struct test_motor_data *test_motor_data(const struct device *dev)
{
	return dev->data;
}

static struct test_wheel_data *test_wheel_data(const struct device *dev)
{
	return dev->data;
}

static void test_motor_reset(const struct device *dev)
{
	struct test_motor_data *data = test_motor_data(dev);
	motor_status_t status = data->status;

	memset(data, 0, sizeof(*data));
	data->status = status;
}

static void test_motor_set_status(const struct device *dev, float angle, float rpm)
{
	struct test_motor_data *data = test_motor_data(dev);

	data->status.angle = angle;
	data->status.rpm = rpm;
}

static void test_wheel_reset(const struct device *dev)
{
	struct test_wheel_data *data = test_wheel_data(dev);

	memset(data, 0, sizeof(*data));
}

static int test_motor_get(const struct device *dev, motor_status_t *status)
{
	struct test_motor_data *data = test_motor_data(dev);

	*status = data->status;
	return 0;
}

static int test_motor_set(const struct device *dev, motor_setpoint_t *setpoint)
{
	struct test_motor_data *data = test_motor_data(dev);

	data->last_setpoint = *setpoint;
	data->set_count++;
	return 0;
}

static void test_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct test_motor_data *data = test_motor_data(dev);

	data->last_cmd = cmd;
	data->control_count++;
}

static const struct motor_driver_api test_motor_api = {
	.motor_control = test_motor_control,
	.motor_set = test_motor_set,
	.motor_get = test_motor_get,
};

#define DT_DRV_COMPAT ares_test_motor

#define TEST_MOTOR_DEFINE(inst)                                                                    \
	static struct test_motor_data test_motor_data_##inst;                                      \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &test_motor_data_##inst, NULL, POST_KERNEL, 80,    \
			      &test_motor_api);

DT_INST_FOREACH_STATUS_OKAY(TEST_MOTOR_DEFINE)

#undef DT_DRV_COMPAT

static void test_wheel_set_speed(const struct device *dev, float speed, float angle)
{
	struct test_wheel_data *data = test_wheel_data(dev);

	data->last_speed = speed;
	data->last_angle = angle;
	data->target.speed = speed;
	data->target.angle = angle;
	data->set_speed_count++;
}

static int test_wheel_set_static(const struct device *dev, float angle)
{
	struct test_wheel_data *data = test_wheel_data(dev);

	data->last_static_angle = angle;
	data->target.speed = 0.0f;
	data->target.angle = angle;
	data->set_static_count++;
	return data->static_return;
}

static wheel_status_t *test_wheel_get_speed(const struct device *dev)
{
	struct test_wheel_data *data = test_wheel_data(dev);

	return &data->status;
}

static wheel_status_t *test_wheel_get_target(const struct device *dev)
{
	struct test_wheel_data *data = test_wheel_data(dev);

	return &data->target;
}

static void test_wheel_disable(const struct device *dev)
{
	struct test_wheel_data *data = test_wheel_data(dev);

	data->disable_count++;
}

static const struct wheel_driver_api test_wheel_api = {
	.wheel_set_speed = test_wheel_set_speed,
	.wheel_set_static = test_wheel_set_static,
	.wheel_get_speed = test_wheel_get_speed,
	.wheel_get_target = test_wheel_get_target,
	.wheel_disable = test_wheel_disable,
};

#define DT_DRV_COMPAT ares_test_wheel

#define TEST_WHEEL_DEFINE(inst)                                                                    \
	static struct test_wheel_data test_wheel_data_##inst;                                      \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &test_wheel_data_##inst, NULL, POST_KERNEL, 80,    \
			      &test_wheel_api);

DT_INST_FOREACH_STATUS_OKAY(TEST_WHEEL_DEFINE)

#undef DT_DRV_COMPAT

ZTEST(ares_native_sim_wheel_chassis, test_devices_are_ready)
{
	zassert_true(device_is_ready(MECANUM_DEV));
	zassert_true(device_is_ready(MECANUM_MOTOR_DEV));
	zassert_true(device_is_ready(STEER_DEV));
	zassert_true(device_is_ready(STEER_MOTOR_DEV));
	zassert_true(device_is_ready(STEER_DRIVE_DEV));
	zassert_true(device_is_ready(CHASSIS_DEV));
	zassert_true(device_is_ready(CHASSIS_WHEEL0_DEV));
	zassert_true(device_is_ready(CHASSIS_WHEEL1_DEV));
	zassert_true(device_is_ready(CHASSIS_WHEEL2_DEV));
	zassert_true(device_is_ready(CHASSIS_WHEEL3_DEV));
}

ZTEST(ares_native_sim_wheel_chassis, test_mecanum_projects_speed_and_latches_static_angle)
{
	test_motor_reset(MECANUM_MOTOR_DEV);
	test_motor_set_status(MECANUM_MOTOR_DEV, 12.5f, 60.0f);

	wheel_set_speed(MECANUM_DEV, 2.0f, 0.0f);

	struct test_motor_data *motor = test_motor_data(MECANUM_MOTOR_DEV);
	wheel_status_t *target = wheel_get_target(MECANUM_DEV);
	wheel_status_t *status = wheel_get_speed(MECANUM_DEV);

	zassert_equal(motor->last_setpoint.mode, VO);
	zassert_equal(motor->last_setpoint.target, MOTOR_TARGET_SPEED);
	assert_float_close(target->speed, 2.0f);
	assert_float_close(motor->last_setpoint.rpm, EXPECTED_RPM_1MPS_R01 * 2.0f);
	assert_float_close(status->speed, RPM2RADPS(60.0f) * 0.1f);

	int ret = wheel_set_static(MECANUM_DEV, 45.0f);

	zassert_ok(ret);
	zassert_equal(motor->last_setpoint.target, MOTOR_TARGET_POSITION);
	assert_float_close(motor->last_setpoint.angle, 12.5f);
}

ZTEST(ares_native_sim_wheel_chassis, test_steerwheel_short_path_stop_and_disable)
{
	test_motor_reset(STEER_MOTOR_DEV);
	test_motor_reset(STEER_DRIVE_DEV);
	test_motor_set_status(STEER_MOTOR_DEV, 120.0f, 0.0f);
	test_motor_set_status(STEER_DRIVE_DEV, 0.0f, 0.0f);

	wheel_set_speed(STEER_DEV, 1.0f, 0.0f);

	struct test_motor_data *steer = test_motor_data(STEER_MOTOR_DEV);
	struct test_motor_data *drive = test_motor_data(STEER_DRIVE_DEV);
	wheel_status_t *target = wheel_get_target(STEER_DEV);

	assert_float_close(target->speed, 1.0f);
	assert_float_close(target->angle, -180.0f);
	assert_float_close(steer->last_setpoint.angle, -180.0f);
	assert_float_close(drive->last_setpoint.rpm, -EXPECTED_RPM_1MPS_R01);

	wheel_set_speed(STEER_DEV, 0.0f, 30.0f);

	target = wheel_get_target(STEER_DEV);
	assert_float_close(target->speed, 0.0f);
	assert_float_close(drive->last_setpoint.rpm, 0.0f);

	wheel_disable(STEER_DEV);

	zassert_equal(steer->last_setpoint.target, MOTOR_TARGET_TORQUE);
	zassert_equal(drive->last_setpoint.target, MOTOR_TARGET_TORQUE);
	assert_float_close(steer->last_setpoint.torque, 0.0f);
	assert_float_close(drive->last_setpoint.torque, 0.0f);
}

ZTEST(ares_native_sim_wheel_chassis, test_chassis_public_commands_and_resolve)
{
	chassis_data_t *data = CHASSIS_DEV->data;
	const chassis_cfg_t *cfg = CHASSIS_DEV->config;
	const struct device *wheels[] = {
		CHASSIS_WHEEL0_DEV,
		CHASSIS_WHEEL1_DEV,
		CHASSIS_WHEEL2_DEV,
		CHASSIS_WHEEL3_DEV,
	};

	chassis_set_speed(CHASSIS_DEV, 1.25f, -0.5f);
	assert_float_close(data->target_status.speedX, 0.5f);
	assert_float_close(data->target_status.speedY, 1.25f);

	chassis_set_gyro(CHASSIS_DEV, 2.5f);
	assert_float_close(data->target_status.gyro, 2.5f);
	zassert_false(data->angleControl);

	chassis_set_angle(CHASSIS_DEV, 45.0f);
	assert_float_close(data->target_status.angle, 45.0f);
	zassert_true(data->angleControl);

	for (size_t i = 0; i < ARRAY_SIZE(wheels); i++) {
		test_wheel_reset(wheels[i]);
	}

	data->static_angle = false;
	data->track_angle = false;
	data->set_status.speedX = 1.0f;
	data->set_status.speedY = 0.0f;
	data->set_status.gyro = 0.0f;

	cchassis_resolve(data, cfg);

	for (size_t i = 0; i < ARRAY_SIZE(wheels); i++) {
		struct test_wheel_data *wheel = test_wheel_data(wheels[i]);

		zassert_equal(wheel->set_speed_count, 1);
		assert_float_close(wheel->last_speed, 1.0f);
		assert_float_close(wheel->last_angle, 90.0f);
	}
}

ZTEST(ares_native_sim_wheel_chassis, test_chassis_static_mode_uses_wheel_static_angles)
{
	chassis_data_t *data = CHASSIS_DEV->data;
	const chassis_cfg_t *cfg = CHASSIS_DEV->config;
	const struct device *wheels[] = {
		CHASSIS_WHEEL0_DEV,
		CHASSIS_WHEEL1_DEV,
		CHASSIS_WHEEL2_DEV,
		CHASSIS_WHEEL3_DEV,
	};
	const float expected_angles[] = {
		45.0f,
		135.0f,
		225.0f,
		-45.0f,
	};

	for (size_t i = 0; i < ARRAY_SIZE(wheels); i++) {
		test_wheel_reset(wheels[i]);
	}

	data->static_angle = true;
	data->set_status.speedX = 0.0f;
	data->set_status.speedY = 0.0f;
	data->set_status.gyro = 0.0f;

	cchassis_resolve(data, cfg);

	for (size_t i = 0; i < ARRAY_SIZE(wheels); i++) {
		struct test_wheel_data *wheel = test_wheel_data(wheels[i]);

		zassert_equal(wheel->set_static_count, 1);
		zassert_equal(wheel->set_speed_count, 0);
		assert_float_close(wheel->last_static_angle, expected_angles[i]);
	}
}

ZTEST_SUITE(ares_native_sim_wheel_chassis, NULL, NULL, NULL, NULL, NULL);
