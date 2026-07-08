/*
 * Copyright (c) 2026 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_GQ_H_
#define MOTOR_GQ_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>

#define DT_DRV_COMPAT gq_motor

#define GQ_CLASSIC_EXT_PREFIX_CONFIG 0x8000U
#define GQ_CLASSIC_EXT_PREFIX_MIT    0x18000U
#define GQ_REPLY_ID_SHIFT            8U
#define GQ_REPLY_ID_MASK             0x1FFFFF00U

#define GQ_POS_SCALE_INT16    10000.0f
#define GQ_VEL_SCALE_INT16    4000.0f
#define GQ_TORQUE_SCALE_INT16 100.0f
#define GQ_PID_SCALE_INT16    10.0f

#define GQ_INT16_NAN ((int16_t)0x8000)

struct gq_motor_data {
	struct motor_driver_data common;
	struct can_filter filter;
	int filter_id;
	int error;

	float target_angle;
	float target_rpm;
	float target_torque;
	float kp;
	float kd;
};

struct gq_motor_config {
	struct motor_driver_config common;
	bool enable_canfd;
};

int gq_init(const struct device *dev);
int gq_set(const struct device *dev, motor_setpoint_t *setpoint);
int gq_get(const struct device *dev, motor_status_t *status);
void gq_control(const struct device *dev, enum motor_cmd cmd);

static const struct motor_driver_api gq_motor_api = {
	.motor_get = gq_get,
	.motor_set = gq_set,
	.motor_control = gq_control,
};

#define GQ_MOTOR_DATA_INST(inst)                                                                   \
	static struct gq_motor_data gq_motor_data_##inst = {                                      \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                    \
		.filter_id = -1,                                                                  \
		.error = 0,                                                                       \
		.target_angle = 0.0f,                                                            \
		.target_rpm = 0.0f,                                                              \
		.target_torque = 0.0f,                                                           \
		.kp = 0.0f,                                                                       \
		.kd = 0.0f,                                                                       \
	}

#define GQ_MOTOR_CONFIG_INST(inst)                                                                 \
	static const struct gq_motor_config gq_motor_cfg_##inst = {                              \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                  \
		.enable_canfd = DT_PROP(DT_DRV_INST(inst), enable_canfd),                         \
	}

#define GQ_MOTOR_DEFINE_INST(inst)                                                                 \
	BUILD_ASSERT(!DT_PROP(DT_DRV_INST(inst), enable_canfd) || IS_ENABLED(CONFIG_CAN_FD_MODE), \
		     "gq,motor enable-canfd requires CONFIG_CAN_FD_MODE=y");                    \
	DEVICE_DT_INST_DEFINE(inst, gq_init, NULL, &gq_motor_data_##inst, &gq_motor_cfg_##inst,   \
			      POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, &gq_motor_api)

#define GQ_MOTOR_INST(inst)                                                                        \
	GQ_MOTOR_CONFIG_INST(inst);                                                                \
	GQ_MOTOR_DATA_INST(inst);                                                                  \
	GQ_MOTOR_DEFINE_INST(inst);

#endif /* MOTOR_GQ_H_ */
