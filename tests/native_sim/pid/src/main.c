/*
 * Copyright (c) 2026 ttwards
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/ztest.h>

#define FLOAT_TOLERANCE 0.0001f

static void assert_float_close(float actual, float expected)
{
	zassert_true(fabsf(actual - expected) < FLOAT_TOLERANCE, "actual=%f expected=%f",
		     (double)actual, (double)expected);
}

static struct device fake_pid_device(const struct pid_config *config)
{
	return (struct device){
		.name = "fake_pid",
		.config = config,
	};
}

ZTEST(ares_native_sim_pid, test_pid_calc_in_clamps_output)
{
	const struct pid_config config = {
		.k_p = 2.0f,
		.k_i = NAN,
		.k_d = NAN,
		.output_limit = 5.0f,
		.output_offset = 1.0f,
	};
	struct device dev = fake_pid_device(&config);
	float output = 0.0f;
	struct pid_data data = {
		.pid_dev = &dev,
		.output = &output,
	};

	float result = pid_calc_in(&data, 3.0f, 1000.0f);

	assert_float_close(result, 5.0f);
	assert_float_close(output, 5.0f);
}

ZTEST(ares_native_sim_pid, test_pid_calc_in_clamps_integral)
{
	const struct pid_config config = {
		.k_p = 1.0f,
		.k_i = 10.0f,
		.k_d = NAN,
		.integral_limit = 1.5f,
	};
	struct device dev = fake_pid_device(&config);
	struct pid_data data = {
		.pid_dev = &dev,
	};

	float positive = pid_calc_in(&data, 1.0f, 100.0f);
	float negative = pid_calc_in(&data, -1.0f, 100.0f);

	assert_float_close(positive, 2.5f);
	assert_float_close(negative, -2.5f);
}

ZTEST(ares_native_sim_pid, test_pid_calc_in_ignores_zero_delta_time)
{
	const struct pid_config config = {
		.k_p = 4.0f,
		.k_i = NAN,
		.k_d = NAN,
	};
	struct device dev = fake_pid_device(&config);
	float output = 123.0f;
	struct pid_data data = {
		.pid_dev = &dev,
		.output = &output,
	};

	float result = pid_calc_in(&data, 3.0f, 0.0f);

	assert_float_close(result, 0.0f);
	assert_float_close(output, 123.0f);
}

ZTEST(ares_native_sim_pid, test_pid_calc_mit_uses_velocity_error)
{
	const struct pid_config config = {
		.k_p = 2.0f,
		.k_i = NAN,
		.k_d = 4.0f,
		.detri_lpf = NAN,
	};
	struct device dev = fake_pid_device(&config);
	float output = 0.0f;
	struct pid_data data = {
		.pid_dev = &dev,
		.output = &output,
	};

	float result = pid_calc_mit(&data, 1.0f, 0.5f, 2.0f);

	assert_float_close(result, 4.0f);
	assert_float_close(output, 4.0f);
}

ZTEST_SUITE(ares_native_sim_pid, NULL, NULL, NULL, NULL, NULL);
