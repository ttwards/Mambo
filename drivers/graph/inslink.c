/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <soc.h>
#include "graph.h"
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#define DT_DRV_COMPAT ares_rttgraph

LOG_MODULE_REGISTER(insLink, CONFIG_INS_LINK_LOG_LEVEL);

DT_INST_FOREACH_STATUS_OKAY(RTT_GRAPH_INST)
