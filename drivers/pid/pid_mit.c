/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_MIT_H
#define PID_MIT_H

#include "zephyr/devicetree.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT pid_mit

#define PID_SINGLE_DT_DRIVER_CONFIG_GET(node_id)                                                   \
	{                                                                                          \
		.k_p = DT_STRING_UNQUOTED(node_id, k_p),                                           \
		.integral_limit = DT_STRING_UNQUOTED_OR(node_id, i_max, 0),                        \
		.output_limit = DT_STRING_UNQUOTED_OR(node_id, out_max, 0),                        \
		.detri_lpf = DT_STRING_UNQUOTED_OR(node_id, detri_lpf, NAN),                       \
		.k_i = DT_STRING_UNQUOTED_OR(node_id, k_i, NAN),                                   \
		.k_d = DT_STRING_UNQUOTED_OR(node_id, k_d, NAN),                                   \
		.mit = true,                                                                       \
		.output_offset = DT_STRING_UNQUOTED_OR(node_id, offset, 0),                        \
	}

#define PID_CONFIG_DEFINE(inst)                                                                    \
	static const struct pid_config mit_config_##inst =                                         \
		PID_SINGLE_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst));

#define PID_INST(inst)                                                                             \
	PID_CONFIG_DEFINE(inst)                                                                    \
	PID_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, NULL, &mit_config_##inst, POST_KERNEL, \
			     90, NULL);

DT_INST_FOREACH_STATUS_OKAY(PID_INST)

#endif // PID_MIT_H
