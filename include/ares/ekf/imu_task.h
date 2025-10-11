#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include "QuaternionEKF.h"
#include <zephyr/drivers/pid.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <sys/types.h>

static const int X = 0;
static const int Y = 1;
static const int Z = 2;

#define FREQ 800

typedef void (*update_cb_t)(QEKF_INS_t *QEKF);

typedef struct {
	// IMU量测值
	float Gyro[3];  // 角速度
	float Accel[3]; // 加速度

	float lpf_Accel[3]; // 加速度低通滤波

	uint32_t gyro_prev_cyc;
	uint32_t accel_prev_cyc;

	float AccelLPF;

	uint32_t gyro_curr_cyc;
	uint32_t accel_curr_cyc;

	update_cb_t update_cb;

	uint32_t temp_update_ms;
	float imu_temp;

	const struct device *accel_dev;
	const struct device *gyro_dev;
} INS_t;

typedef struct {
	bool flag;
	float Gyro[3]; // 角速度
} IMU_Bias_t;

/* 用于修正安装误差的参数 */
typedef struct {
	uint8_t flag;

	float scale[3];

	float Yaw;
	float Pitch;
	float Roll;
} IMU_Param_t;

float IMU_temp_read(const struct device *dev);
void IMU_Sensor_set_update_cb(update_cb_t cb);
#ifdef CONFIG_IMU_PWM_TEMP_CTRL
void IMU_Sensor_set_IMU_temp(float temp);
#endif // CONFIG_IMU_PWM_TEMP_CTRL
void IMU_Sensor_trig_init(const struct device *accel_dev, const struct device *gyro_dev);

#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

extern INS_t INS;

#if DT_HAS_CHOSEN(ares_bias)
#define GYRO_BIAS_NODE DT_CHOSEN(ares_bias)
#if DT_NODE_HAS_PROP(GYRO_BIAS_NODE, gyro_bias)
#endif // DT_NODE_HAS_PROP(GYRO_BIAS_NODE, gyro_bias)
#endif // DT_HAS_CHOSEN(ares_bias)

#endif // IMU_TASK_H
