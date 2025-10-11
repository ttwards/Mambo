/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "stdlib.h"
#include "memory.h"
#include "math.h"
#include "algorithm.h"

#ifndef user_malloc
#define user_malloc malloc
#endif

void *zmalloc(size_t size)
{
	void *ptr = malloc(size);
	memset(ptr, 0, size);
	return ptr;
}

// 快速开方
float Sqrt(float x)
{
	float y;
	float delta;
	float maxError;

	if (x <= 0) {
		return 0;
	}

	// initial guess
	y = x / 2;

	// refine
	maxError = x * 0.001f;

	do {
		delta = (y * y) - x;
		y -= delta / (2 * y);
	} while (delta > maxError || delta < -maxError);

	return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
	if (num > Limit) {
		num = Limit;
	} else if (num < -Limit) {
		num = -Limit;
	}
	return num;
}

// 判断符号位
float sign(float value)
{
	if (value >= 0.0f) {
		return 1.0f;
	} else {
		return -1.0f;
	}
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
	if (Value < maxValue && Value > minValue) {
		Value = 0.0f;
	}
	return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
	if (Value < minValue) {
		return minValue;
	} else if (Value > maxValue) {
		return maxValue;
	} else {
		return Value;
	}
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
	if (Value < minValue) {
		return minValue;
	} else if (Value > maxValue) {
		return maxValue;
	} else {
		return Value;
	}
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
	if (maxValue < minValue) {
		return Input;
	}

	if (Input > maxValue) {
		float len = maxValue - minValue;
		while (Input > maxValue) {
			Input -= len;
		}
	} else if (Input < minValue) {
		float len = maxValue - minValue;
		while (Input < minValue) {
			Input += len;
		}
	}
	return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
	return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
	static int integer;
	static float decimal;
	integer = (int)raw;
	decimal = raw - integer;
	if (decimal > 0.5f) {
		integer++;
	}
	return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
	float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	v[0] /= len;
	v[1] /= len;
	v[2] /= len;
	return v;
}

// 计算模长
float NormOf3d(float *v)
{
	return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
	res[0] = v1[1] * v2[2] - v1[2] * v2[1];
	res[1] = v1[2] * v2[0] - v1[0] * v2[2];
	res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len)
{
	float sum = 0;
	for (uint8_t i = 0; i < len - 1; i++) {
		buf[i] = buf[i + 1];
		sum += buf[i];
	}
	buf[len - 1] = new_data;
	sum += new_data;
	return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
	m->numCols = col;
	m->numRows = row;
	m->pData = (float *)zmalloc(row * col * sizeof(float));
}

void GetGroundAccel(float *q, float *accel, float g, float *ground_accel)
{
	// 构造旋转矩阵(四元数转旋转矩阵)
	float R[3][3];
	R[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
	R[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
	R[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);

	R[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
	R[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
	R[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);

	R[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	R[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
	R[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

	// 将加速度从板子坐标系转换到地面坐标系
	ground_accel[0] = R[0][0] * accel[0] + R[0][1] * accel[1] + R[0][2] * accel[2];
	ground_accel[1] = R[1][0] * accel[0] + R[1][1] * accel[1] + R[1][2] * accel[2];
	ground_accel[2] = R[2][0] * accel[0] + R[2][1] * accel[1] + R[2][2] * accel[2];

	// 减去重力分量
	// 注意:假设重力方向为-z轴方向
	ground_accel[2] -= g;
}

void GetDeviceAccel(float *q, float *ground_accel, float g, float *device_accel)
{
	// 构造旋转矩阵的转置(相当于逆矩阵)
	float R[3][3];
	// 原来的R[0][0]变成R[0][0], R[0][1]变成R[1][0], R[0][2]变成R[2][0]，以此类推
	R[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
	R[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
	R[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);

	R[0][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
	R[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
	R[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);

	R[0][2] = 2 * (q[1] * q[3] - q[0] * q[2]);
	R[1][2] = 2 * (q[2] * q[3] + q[0] * q[1]);
	R[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

	// 先加回重力分量
	float temp_accel[3];
	temp_accel[0] = ground_accel[0];
	temp_accel[1] = ground_accel[1];
	temp_accel[2] = ground_accel[2] + g;

	// 将加速度从地面坐标系转换回设备坐标系
	device_accel[0] =
		R[0][0] * temp_accel[0] + R[1][0] * temp_accel[1] + R[2][0] * temp_accel[2];
	device_accel[1] =
		R[0][1] * temp_accel[0] + R[1][1] * temp_accel[1] + R[2][1] * temp_accel[2];
	device_accel[2] =
		R[0][2] * temp_accel[0] + R[1][2] * temp_accel[1] + R[2][2] * temp_accel[2];
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 将四元数转换为欧拉角 (Yaw, Pitch, Roll)，输出为角度制
 * @param q 四元数数组，q[0]=w, q[1]=x, q[2]=y, q[3]=z
 * @param yaw   输出的偏航角 (角度)
 * @param pitch 输出的俯仰角 (角度)
 * @param roll  输出的横滚角 (角度)
 */
void quaternionToYawPitchRoll(float *q, float *yaw, float *pitch, float *roll)
{
	// Roll（横滚，x轴旋转）
	float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
	float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
	float roll_rad = atan2f(sinr_cosp, cosr_cosp);

	// Pitch（俯仰，y轴旋转）
	float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
	float pitch_rad;
	if (fabsf(sinp) >= 1.0f) {
		// 万向节死锁处理
		pitch_rad = copysignf((float)M_PI / 2.0f, sinp); // 超出范围时用90度
	} else {
		pitch_rad = asinf(sinp);
	}

	// Yaw（偏航，z轴旋转）
	float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
	float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
	float yaw_rad = atan2f(siny_cosp, cosy_cosp);

	// 转换为角度
	*roll = roll_rad * 180.0f / (float)M_PI;
	*pitch = pitch_rad * 180.0f / (float)M_PI;
	*yaw = yaw_rad * 180.0f / (float)M_PI;
}
