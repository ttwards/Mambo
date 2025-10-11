/*
 * ARES Plotter Protocol Example
 * Demonstrates how to use the new plotter protocol and interface for variable monitoring
 *
 * Copyright (c) 2024 ttwards
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <ares/ares_comm.h>
#include <ares/interface/uart/uart.h>
#include "ares/protocol/plotter/aresplot_protocol.h"

LOG_MODULE_REGISTER(plotter_demo, LOG_LEVEL_INF);

#ifndef PI
#define PI 3.14159265f
#endif

// Variables to monitor
static float sine_wave = 0.0f;
static float cosine_wave = 0.0f;
static int32_t counter = 0;
static uint16_t random_value = 0;
static bool toggle_flag = false;

/**
 * @brief Variable update thread
 */
void variables_update_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	uint32_t time_ms = 0;

	while (1) {
		k_msleep(1); // 50Hz update rate
		time_ms += 1;

		// Update various types of variables
		float time_sec = time_ms / 1000.0f;

		// Sine and cosine wave (1Hz)
		sine_wave = sinf(2.0f * PI * time_sec);
		cosine_wave = cosf(2.0f * PI * time_sec);

		// Counter
		counter++;

		// Simulate random value
		random_value = (uint16_t)(sine_wave * 1000 + 1000);

		// Toggle boolean value (toggle every 2 seconds)
		if ((time_ms % 2000) < 50) {
			toggle_flag = !toggle_flag;
		}
	}
}

K_THREAD_DEFINE(variables_update, 1024, variables_update_thread, NULL, NULL, NULL, -1, 0, 0);

/**
 * @brief Main function
 */
int main(void)
{
	LOG_INF("=== ARES Plotter Protocol Example ===");
	LOG_INF("Demonstrating variable monitoring feature");

	LOG_INF("Start monitoring variables");
	LOG_INF("Connect serial tool to view data transmission");
	LOG_INF("Baudrate: 921600, Data format: 8N1");

	// Main loop
	uint32_t last_status_time = 0;
	while (1) {
		k_msleep(1000);

		uint32_t now = k_uptime_get_32();
		if (now - last_status_time >= 5000) {
			LOG_INF("Status update - Sine: %.2f, Counter: %d, Random: %d, Flag: %s",
				sine_wave, counter, random_value, toggle_flag ? "true" : "false");
			last_status_time = now;
		}
	}

	return 0;
}
