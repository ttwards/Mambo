/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef STEERWHEEL_H
#define STEERWHEEL_H

#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <errno.h>

#define RADPS_TO_RPM 9.54929659f

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358f
#endif

#define DT_WHEEL_CONFIG_GET(inst)                                                                  \
	{                                                                                          \
		.angle_offset = (float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), angle_offset),        \
		.wheel_radius = (float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), wheel_radius),        \
	}

typedef struct {
	float angle_offset;
	float wheel_radius;
} wheel_cfg_t;

typedef struct {
	float speed;
	float angle;
	bool restricted;
} wheel_status_t;

/* Typedefs */
typedef void (*wheel_api_set_speed_t)(const struct device *dev, float speed, float angle);
typedef int (*wheel_api_set_static_t)(const struct device *dev, float angle);
typedef wheel_status_t *(*wheel_api_get_speed_t)(const struct device *dev);
typedef wheel_status_t *(*wheel_api_get_target_t)(const struct device *dev);
typedef void (*wheel_api_disable_t)(const struct device *dev);

/* API Structure */
__subsystem struct wheel_driver_api {
	wheel_api_set_speed_t wheel_set_speed;
	wheel_api_set_static_t wheel_set_static;
	wheel_api_get_speed_t wheel_get_speed;
	wheel_api_get_target_t wheel_get_target;
	wheel_api_disable_t wheel_disable;
};

/* Syscalls and inline implementations */
__syscall void wheel_set_speed(const struct device *dev, float speed, float angle);
static inline void z_impl_wheel_set_speed(const struct device *dev, float speed, float angle)
{
	const struct wheel_driver_api *api = (const struct wheel_driver_api *)dev->api;
	if (api->wheel_set_speed == NULL) {
		return;
	}
	api->wheel_set_speed(dev, speed, angle);
}

__syscall int wheel_set_static(const struct device *dev, float angle);
static inline int z_impl_wheel_set_static(const struct device *dev, float angle)
{
	const struct wheel_driver_api *api = (const struct wheel_driver_api *)dev->api;
	if (api->wheel_set_static == NULL) {
		return -ENOSYS;
	}
	return api->wheel_set_static(dev, angle);
}

__syscall wheel_status_t *wheel_get_speed(const struct device *dev);
static inline wheel_status_t *z_impl_wheel_get_speed(const struct device *dev)
{
	const struct wheel_driver_api *api = (const struct wheel_driver_api *)dev->api;
	if (api->wheel_get_speed == NULL) {
		return NULL;
	}
	return api->wheel_get_speed(dev);
}

__syscall wheel_status_t *wheel_get_target(const struct device *dev);
static inline wheel_status_t *z_impl_wheel_get_target(const struct device *dev)
{
	const struct wheel_driver_api *api = (const struct wheel_driver_api *)dev->api;
	if (api->wheel_get_target == NULL) {
		return NULL;
	}
	return api->wheel_get_target(dev);
}

__syscall void wheel_disable(const struct device *dev);
static inline void z_impl_wheel_disable(const struct device *dev)
{
	const struct wheel_driver_api *api = (const struct wheel_driver_api *)dev->api;
	if (api->wheel_disable == NULL) {
		return;
	}
	api->wheel_disable(dev);
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/wheel.h>

#endif // STEERWHEEL_H
