/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_SBUS_H_
#define ZEPHYR_DRIVERS_SBUS_H_

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SBUS driver API
 */
struct sbus_driver_api {
	float (*getchannel_percentage)(const struct device *dev, uint8_t channelid);
	int (*getchannel_digital)(const struct device *dev, uint8_t channelid);
};

/**
 * @brief Get the percentage value of a SBUS channel
 *
 * @param dev Pointer to the SBUS device
 * @param channelid Channel ID to read
 * @return float Percentage value (-1.0 to 1.0)
 */
__syscall float sbus_get_percent(const struct device *dev, uint8_t channelid);

static inline float z_impl_sbus_get_percent(const struct device *dev, uint8_t channelid)
{
	const struct sbus_driver_api *api = (const struct sbus_driver_api *)dev->api;
	return api->getchannel_percentage(dev, channelid);
}

/**
 * @brief Get the digital value of a SBUS channel
 *
 * @param dev Pointer to the SBUS device
 * @param channelid Channel ID to read
 * @return int Digital value
 */
__syscall int sbus_get_digit(const struct device *dev, uint8_t channelid);

static inline int z_impl_sbus_get_digit(const struct device *dev, uint8_t channelid)
{
	const struct sbus_driver_api *api = (const struct sbus_driver_api *)dev->api;
	return api->getchannel_digital(dev, channelid);
}

#ifdef __cplusplus
}
#endif

#include <syscalls/sbus.h>

#endif /* ZEPHYR_DRIVERS_SBUS_H_ */
