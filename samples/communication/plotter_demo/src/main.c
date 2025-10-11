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

// UART device definition
#define UART_DEV DT_NODELABEL(usart6)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEV);

// Protocol and interface definition
PLOTTER_PROTOCOL_DEFINE(plotter_protocol);
ARES_UART_INTERFACE_DEFINE(uart_interface);

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
		k_msleep(2); // 50Hz update rate
		time_ms += 2;

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

K_THREAD_DEFINE(variables_update, 1024, variables_update_thread, NULL, NULL, NULL, 6, 0, 0);

/**
 * @brief Main function
 */
int main(void)
{
	LOG_INF("=== ARES Plotter Protocol Example ===");
	LOG_INF("Demonstrating variable monitoring feature");

	// Check if UART device is ready
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device is not ready");
		return -ENODEV;
	}

	// Initialize UART interface
	ares_uart_init_dev(&uart_interface, uart_dev);

	// Bind plotter protocol to UART interface
	int ret = ares_bind_interface(&uart_interface, &plotter_protocol);
	if (ret != 0) {
		LOG_ERR("Failed to bind plotter protocol to UART interface: %d", ret);
		return ret;
	}

	LOG_INF("Plotter protocol is bound to UART interface");

	// Wait for a while to let the interface initialize
	k_msleep(1000);

	// Add variables to monitor
	plotter_add_variable(&plotter_protocol, &sine_wave, ARES_TYPE_FLOAT32, "sine_wave");
	plotter_add_variable(&plotter_protocol, &cosine_wave, ARES_TYPE_FLOAT32, "cosine_wave");
	plotter_add_variable(&plotter_protocol, &counter, ARES_TYPE_INT32, "counter");
	plotter_add_variable(&plotter_protocol, &random_value, ARES_TYPE_UINT16, "random_value");
	plotter_add_variable(&plotter_protocol, &toggle_flag, ARES_TYPE_BOOL, "toggle_flag");

	LOG_INF("5 variables have been added for monitoring");

	// Set sample rate to 1ms (1000Hz)
	// Not necessary to set sample_rate, setting it will override Kconfig
	plotter_set_sample_rate(&plotter_protocol, 1);

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
