/*
 * Copyright (c) 2026 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gq_motor_demo, LOG_LEVEL_INF);

#define GQ_MOTOR_NODE DT_PATH(motor, gq_motor)

BUILD_ASSERT(DT_NODE_HAS_STATUS(GQ_MOTOR_NODE, okay), "gq_motor node must be enabled");

static const struct device *const gq_motor = DEVICE_DT_GET(GQ_MOTOR_NODE);

static void log_motor_status(void)
{
	motor_status_t status = {0};
	int ret = motor_get(gq_motor, &status);

	if (ret < 0) {
		LOG_INF("status unavailable: %d", ret);
		return;
	}

	LOG_INF("status: angle %.1f deg, speed %.1f rpm, torque %.2f Nm, error %d",
		(double)status.angle, (double)status.rpm, (double)status.torque, status.error);
}

int main(void)
{
	LOG_INF("Gaoqing motor demo started");

	if (!device_is_ready(gq_motor)) {
		LOG_ERR("Gaoqing motor device is not ready");
		return -ENODEV;
	}

	motor_control(gq_motor, ENABLE_MOTOR);
	k_msleep(200);

	while (true) {
		LOG_INF("command: speed 60 rpm");
		motor_set_speed(gq_motor, 60.0f);
		k_msleep(1000);
		log_motor_status();

		LOG_INF("command: position 90 deg");
		motor_set_angle(gq_motor, 90.0f);
		k_msleep(1000);
		log_motor_status();

		LOG_INF("command: MIT hold near zero");
		motor_set_mit(gq_motor, 0.0f, 0.0f, 0.0f);
		k_msleep(1000);
		log_motor_status();
	}

	return 0;
}
