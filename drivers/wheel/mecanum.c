/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MECANUM_C
#define MECANUM_C

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/wheel.h>
#include <zephyr/drivers/motor.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358f
#endif

typedef struct {
	wheel_cfg_t common;

	float free_angle;
	bool inverse_wheel;
	const struct device *wheel_motor;
} mecanum_cfg_t;

typedef struct {
	wheel_status_t status;
	wheel_status_t target;
	float set_rpm;

	float static_angle;
} mecanum_data_t;

void vector_projection(float target_size, float target_angle, float proj_angle, float *result_size)
{
	// 将12点钟为0度顺时针的角度转换为x轴为0度逆时针的角度
	float target_convert = 90.0f - target_angle;
	float proj_convert = 90.0f - proj_angle;

	// 转换为弧度
	float target_rad = target_convert * PI / 180.0f;
	float proj_rad = proj_convert * PI / 180.0f;

	// 计算夹角的余弦值
	float cos_theta = cosf(target_rad - proj_rad);

	// 计算投影长度
	*result_size = target_size * cos_theta;
}

static inline void mecanum_set_speed(const struct device *dev, float speed, float angle)
{
	mecanum_data_t *data = dev->data;
	const mecanum_cfg_t *cfg = dev->config;

	float curr_speed = cfg->inverse_wheel ? -motor_get_speed(cfg->wheel_motor)
					      : motor_get_speed(cfg->wheel_motor);
	data->status.speed =
		RPM2RADPS(curr_speed) * cfg->common.wheel_radius * cosf(cfg->free_angle);

	if (cfg->inverse_wheel) {
		angle = -angle;
	}

	vector_projection(speed, angle - cfg->common.angle_offset, cfg->common.angle_offset,
			  &data->target.speed);
	data->set_rpm = (cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * data->target.speed /
			cfg->common.wheel_radius / cosf(cfg->free_angle);

	data->static_angle = NAN;

	motor_set_speed(cfg->wheel_motor, data->set_rpm);
}

static inline int mecanum_set_static(const struct device *dev, float angle)
{
	mecanum_data_t *data = dev->data;
	const mecanum_cfg_t *cfg = dev->config;

	data->set_rpm = 0;

	data->target.speed = 0;
	float curr_speed = cfg->inverse_wheel ? -motor_get_speed(cfg->wheel_motor)
					      : motor_get_speed(cfg->wheel_motor);
	data->status.speed =
		RPM2RADPS(curr_speed) * cfg->common.wheel_radius * cosf(cfg->free_angle);

	if (cfg->inverse_wheel) {
		angle = -angle;
	}

	if (isnan(data->static_angle)) {
		data->static_angle = motor_get_angle(cfg->wheel_motor);
	}

	return motor_set_angle(cfg->wheel_motor, data->static_angle);
}

static inline wheel_status_t *mecanum_get_speed(const struct device *dev)
{
	mecanum_data_t *data = dev->data;
	const mecanum_cfg_t *cfg = dev->config;

	float curr_speed = cfg->inverse_wheel ? -motor_get_speed(cfg->wheel_motor)
					      : motor_get_speed(cfg->wheel_motor);
	data->status.speed =
		RPM2RADPS(curr_speed) * cfg->common.wheel_radius * cosf(cfg->free_angle);

	return &data->status;
}

static inline wheel_status_t *mecanum_get_target(const struct device *dev)
{
	mecanum_data_t *data = dev->data;

	return &data->target;
}

struct wheel_driver_api mecanum_driver_api = {
	.wheel_set_speed = mecanum_set_speed,
	.wheel_set_static = mecanum_set_static,
	.wheel_get_speed = mecanum_get_speed,
	.wheel_get_target = mecanum_get_target,
};

#define MECANUM_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)        \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define DT_DRV_COMPAT ares_mecanum

#define MECANUM_DEVICE_DEFINE(inst)                                                                \
	static mecanum_data_t mecanum_data_##inst = {                                              \
		.status = {0,                                                                      \
			   DT_STRING_UNQUOTED(DT_DRV_INST(inst), angle_offset) +                   \
				   DT_STRING_UNQUOTED(DT_DRV_INST(inst), free_angle) - 90.0f,      \
			   false},                                                                 \
		.target = {0,                                                                      \
			   DT_STRING_UNQUOTED(DT_DRV_INST(inst), angle_offset) +                   \
				   DT_STRING_UNQUOTED(DT_DRV_INST(inst), free_angle) - 90.0f,      \
			   false},                                                                 \
	};                                                                                         \
	static const mecanum_cfg_t mecanum_cfg_##inst = {                                          \
		.common = DT_WHEEL_CONFIG_GET(inst),                                               \
		.wheel_motor = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), wheel_motor)),             \
		.inverse_wheel = DT_PROP(DT_DRV_INST(inst), inverse_wheel),                        \
		.free_angle = DT_STRING_UNQUOTED(DT_DRV_INST(inst), free_angle),                   \
	};                                                                                         \
	MECANUM_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, &mecanum_data_##inst,              \
				 &mecanum_cfg_##inst, POST_KERNEL, 90, &mecanum_driver_api);

DT_INST_FOREACH_STATUS_OKAY(mecanum_DEVICE_DEFINE)

#endif // MECANUM_C
