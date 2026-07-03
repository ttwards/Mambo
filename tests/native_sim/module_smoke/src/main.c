/*
 * Copyright (c) 2026 ttwards
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

ZTEST(ares_native_sim_module_smoke, test_runs_on_native_sim)
{
	zassert_true(IS_ENABLED(CONFIG_BOARD_NATIVE_SIM), "test must run on native_sim");
}

ZTEST_SUITE(ares_native_sim_module_smoke, NULL, NULL, NULL, NULL, NULL);
