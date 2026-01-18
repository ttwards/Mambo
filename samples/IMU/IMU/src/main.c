/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <ares/board/init.h>
#include <ares/ekf/imu_task.h>
#include <ares/vofa/justfloat.h>
#include <ares/usb_bulk_trans/usb_trans.h>
#include "ares/ekf/QuaternionEKF.h"
#include "devices.h"
#include <zephyr/debug/thread_analyzer.h>
#include <ares/ekf/algorithm.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#ifndef PI
#define PI 3.14159265f
#endif

/* CAN Feedback to console*/
void console_feedback(void *arg1, void *arg2, void *arg3)
{
	while (1) {
		LOG_INF("q: %f, %f, %f, %f", (double)QEKF_INS.q[0], (double)QEKF_INS.q[1],
			(double)QEKF_INS.q[2], (double)QEKF_INS.q[3]);
		LOG_INF("Yaw: %f, Pitch: %f, Roll: %f", (double)QEKF_INS.Yaw,
			(double)QEKF_INS.Pitch, (double)QEKF_INS.Roll);
		k_msleep(200);
	}
}
K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, -1, 0, 100);

float vel[3] = {0};
float pos[3] = {0};
float turned_accel[3] = {0};
void Sensor_update_cb(QEKF_INS_t *QEKF)
{
}

float vel_data[4] = {0};
void trans_cb(int arg)
{
	return;
}

int main(void)
{
	k_msleep(2000);

	IMU_Sensor_trig_init(accel_dev, gyro_dev);
	IMU_Sensor_set_update_cb(Sensor_update_cb);

	while (1) {
		k_msleep(10);
	}
}
