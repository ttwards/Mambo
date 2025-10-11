/*
 * DJI M3508电机控制示例 - 简化版
 * 自动循环：设置零点 -> 100rpm转10s -> 停下 -> 设置零点 -> 正转180度 -> 循环
 *
 * Copyright (c) 2024 ttwards
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(dji_m3508_demo, LOG_LEVEL_INF);

// 电机设备定义
#define M3508_NODE DT_NODELABEL(m3508_motor)
static const struct device *m3508_motor = DEVICE_DT_GET(M3508_NODE);

// 测试序列状态
typedef enum {
	STATE_SET_ZERO_1 = 0, // 设置零点1
	STATE_SPEED_TEST,     // 100rpm转10秒
	STATE_STOP,           // 停下
	STATE_SET_ZERO_2,     // 设置零点2
	STATE_ANGLE_TEST,     // 正转180度
	STATE_MAX
} test_state_t;

static test_state_t current_state = STATE_SET_ZERO_1;
static uint32_t state_start_time = 0;

/**
 * @brief 电机状态监控线程
 */
void motor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_msleep(500); // 每秒检查一次

		motor_status_t status;
		int ret = motor_get(m3508_motor, &status);

		if (ret != 0) {
			LOG_ERR("读取电机状态失败: %d", ret);
			continue;
		}

		LOG_INF("motor torque: %.2f, speed: %.2f, angle: %.2f", status.torque, status.rpm,
			status.angle);
	}
}

K_THREAD_DEFINE(motor_monitor, 1024, motor_monitor_thread, NULL, NULL, NULL, 5, 0, 0);

/**
 * @brief 执行测试序列
 */
void execute_test_sequence(void)
{
	uint32_t elapsed_time = k_uptime_get_32() - state_start_time;
	motor_status_t status;

	switch (current_state) {
	case STATE_SET_ZERO_1:
		LOG_INF("=== 步骤1: 设置零点1 ===");
		motor_control(m3508_motor, SET_ZERO);
		k_msleep(500); // 等待零点设置完成

		// 切换到下一状态
		current_state = STATE_SPEED_TEST;
		state_start_time = k_uptime_get_32();
		LOG_INF("=== 步骤2: 开始速度测试 (100 RPM, 10秒) ===");
		break;

	case STATE_SPEED_TEST:
		motor_set_speed(m3508_motor, 100.0f);

		// 每秒显示状态
		if (elapsed_time % 1000 < 50) {
			motor_get(m3508_motor, &status);
			LOG_INF("速度测试 - 目标: 100.0 RPM, 实际: %.1f RPM, 剩余: %d秒",
				status.rpm, (10000 - elapsed_time) / 1000);
		}

		// 10秒后停止
		if (elapsed_time >= 10000) {
			current_state = STATE_STOP;
			state_start_time = k_uptime_get_32();
			LOG_INF("=== 步骤3: 停止电机 ===");
		}
		break;

	case STATE_STOP:
		motor_set_torque(m3508_motor, 0.0f);

		// 停止2秒
		if (elapsed_time >= 2000) {
			current_state = STATE_SET_ZERO_2;
			state_start_time = k_uptime_get_32();
			LOG_INF("=== 步骤4: 设置零点2 ===");
		}
		break;

	case STATE_SET_ZERO_2:
		motor_control(m3508_motor, SET_ZERO);
		k_msleep(500); // 等待零点设置完成

		// 切换到角度测试
		current_state = STATE_ANGLE_TEST;
		state_start_time = k_uptime_get_32();
		LOG_INF("=== 步骤5: 开始角度测试 (正转180度) ===");
		break;

	case STATE_ANGLE_TEST:
		motor_set_angle(m3508_motor, 180.0f);

		// 每500ms显示状态
		if (elapsed_time % 500 < 50) {
			motor_get(m3508_motor, &status);
			LOG_INF("角度测试 - 目标: 180.0°, 实际: %.1f°, 速度: %.1f RPM",
				status.angle, status.rpm);
		}

		// 检查是否到达目标角度或超时
		motor_get(m3508_motor, &status);
		if (fabsf(status.angle - 180.0f) < 5.0f || elapsed_time >= 8000) {
			if (fabsf(status.angle - 180.0f) < 5.0f) {
				LOG_INF("角度测试完成！实际角度: %.1f°", status.angle);
			} else {
				LOG_WRN("角度测试超时！当前角度: %.1f°", status.angle);
			}

			// 等待1秒后重新开始循环
			k_msleep(1000);
			current_state = STATE_SET_ZERO_1;
			state_start_time = k_uptime_get_32();
			LOG_INF("=== 循环完成，重新开始 ===");
		}
		break;

	default:
		current_state = STATE_SET_ZERO_1;
		state_start_time = k_uptime_get_32();
		break;
	}
}

/**
 * @brief 主函数
 */
int main(void)
{
	LOG_INF("=== DJI M3508电机控制示例 - 简化版 ===");
	LOG_INF("测试序列：设置零点 -> 100rpm转10s -> 停下 -> 设置零点 -> 正转180度 -> 循环");

	// 检查电机设备是否就绪
	if (!device_is_ready(m3508_motor)) {
		LOG_ERR("M3508电机设备未就绪");
		return -ENODEV;
	}

	// 启用电机
	motor_control(m3508_motor, ENABLE_MOTOR);
	LOG_INF("电机已启用");

	// 初始化状态
	state_start_time = k_uptime_get_32();

	// 主控制循环
	while (1) {
		k_msleep(50); // 20Hz控制频率
		execute_test_sequence();
	}

	return 0;
}
