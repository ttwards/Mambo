//=====================================================================================================
// MahonyAHRS.c (版本：完整函数库，带全面内部诊断日志)
//=====================================================================================================
//
// 修正了之前版本的所有问题，并为库中所有公开函数添加了详细的 printf 日志。
// 保持了 “修正函数调用预测函数” 的原始代码结构。
//
//=====================================================================================================

#include "MahonyAHRS.h"
#include <stdio.h> // 用于printf
#include <math.h>
#include <float.h> // 用于FLT_EPSILON

// =================================================================================================
// --- 诊断开关 ---
// #define MAHONY_DEBUG_INTERNAL 1
// 设置为 1 来开启详细的内部日志打印。
// 设置为 0 来关闭日志，恢复正常运行。
// =================================================================================================

// --- 安全修正：增加积分限幅 (Anti-windup) ---
#define INTEGRAL_LIMIT (5000000.0f)

// --- 默认增益参数 ---
#define twoKpDef (2.0f * 0.0f)
#define twoKiDef (2.0f * 0.0f)

// --- 全局变量 ---
volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

MahonyAHRS_INS_t MahonyAHRS_INS;

// --- 函数声明 ---
float invSqrt(float x);

//====================================================================================================
// 函数实现
//====================================================================================================

/**
 * @brief 纯陀螺仪积分（预测步骤）
 */
void MahonyAHRSupdateGyro(float gx, float gy, float gz, float dt)
{
	if (dt <= 0.0f) {
#if MAHONY_DEBUG_INTERNAL
		printf("[MAHONY_DEBUG][Gyro] FAILED: dt is zero or negative (%.4f)!\n", dt);
#endif
		return;
	}
#if MAHONY_DEBUG_INTERNAL
	printf("\n[MAHONY_DEBUG][Gyro] ENTRY: G=[%.3f, %.3f, %.3f], dt=%.4f\n", gx, gy, gz, dt);
	printf("[MAHONY_DEBUG][Gyro] State before: q=[%.4f, %.4f, %.4f, %.4f]\n", q0, q1, q2, q3);
#endif

	float recipNorm;
	float qa, qb, qc;

	// 直接使用全局的积分补偿值
	gx += integralFBx;
	gy += integralFBy;
	gz += integralFBz;

	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

#if MAHONY_DEBUG_INTERNAL
	printf("[MAHONY_DEBUG][Gyro] q before norm: [%.4f, %.4f, %.4f, %.4f]\n", q0, q1, q2, q3);
#endif

	float mag_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
	if (mag_sq < FLT_EPSILON || isnan(mag_sq) || isinf(mag_sq)) {
#if MAHONY_DEBUG_INTERNAL
		printf("[MAHONY_DEBUG][Gyro] FATAL: Quaternion magnitude is %.4e, invalid! "
		       "Resetting state.\n",
		       mag_sq);
#endif
		q0 = 1.0f;
		q1 = 0.0f;
		q2 = 0.0f;
		q3 = 0.0f;
		return;
	}
	recipNorm = invSqrt(mag_sq);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/**
 * @brief 9轴AHRS更新（修正步骤）
 */
void MahonyAHRSupdate(float gx, float gy, float gz, float gyro_dt, float ax, float ay, float az,
		      float mx, float my, float mz, float accel_dt)
{
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

	if (accel_dt <= 0.0f) {
#if MAHONY_DEBUG_INTERNAL
		printf("[MAHONY_DEBUG][AHRS] FAILED: accel_dt is zero or negative (%.4f)!\n",
		       accel_dt);
#endif
		return;
	}
#if MAHONY_DEBUG_INTERNAL
	printf("\n[MAHONY_DEBUG][AHRS] ENTRY: G=[%.3f,%.3f,%.3f], A=[%.3f,%.3f,%.3f], "
	       "M=[%.3f,%.3f,%.3f], g_dt=%.4f, a_dt=%.4f\n",
	       gx, gy, gz, ax, ay, az, mx, my, mz, gyro_dt, accel_dt);
	printf("[MAHONY_DEBUG][AHRS] integralFB before: [%.4f, %.4f, %.4f]\n", integralFBx,
	       integralFBy, integralFBz);
#endif

	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, gyro_dt, ax, ay, az, accel_dt);
		return;
	}

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float recipNorm;
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
		float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q2q2 = q2 * q2;
		float q2q3 = q2 * q3, q3q3 = q3 * q3;

		float hx = 2.0f *
			   (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		float hy = 2.0f *
			   (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		float h_sq = hx * hx + hy * hy;
		float bx = (h_sq > FLT_EPSILON) ? (h_sq * invSqrt(h_sq)) : 0.0f;
		float bz = 2.0f *
			   (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		float halfvx = q1q3 - q0q2;
		float halfvy = q0q1 + q2q3;
		float halfvz = q0q0 - 0.5f + q3q3;
		float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * accel_dt;
			integralFBy += twoKi * halfey * accel_dt;
			integralFBz += twoKi * halfez * accel_dt;
			if (integralFBx > INTEGRAL_LIMIT)
				integralFBx = INTEGRAL_LIMIT;
			else if (integralFBx < -INTEGRAL_LIMIT)
				integralFBx = -INTEGRAL_LIMIT;
			if (integralFBy > INTEGRAL_LIMIT)
				integralFBy = INTEGRAL_LIMIT;
			else if (integralFBy < -INTEGRAL_LIMIT)
				integralFBy = -INTEGRAL_LIMIT;
			if (integralFBz > INTEGRAL_LIMIT)
				integralFBz = INTEGRAL_LIMIT;
			else if (integralFBz < -INTEGRAL_LIMIT)
				integralFBz = -INTEGRAL_LIMIT;
		} else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
	}

#if MAHONY_DEBUG_INTERNAL
	printf("[MAHONY_DEBUG][AHRS] integralFB after: [%.4f, %.4f, %.4f]\n", integralFBx,
	       integralFBy, integralFBz);
#endif

	float final_gx = gx + twoKp * halfex;
	float final_gy = gy + twoKp * halfey;
	float final_gz = gz + twoKp * halfez;

#if MAHONY_DEBUG_INTERNAL
	printf("[MAHONY_DEBUG][AHRS] Gyro for integration (raw+prop): [%.3f, %.3f, %.3f]\n",
	       final_gx, final_gy, final_gz);
#endif

	MahonyAHRSupdateGyro(final_gx, final_gy, final_gz, gyro_dt);
}

/**
 * @brief 6轴IMU更新（修正步骤）
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float gyro_dt, float ax, float ay, float az,
			 float accel_dt)
{
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

	if (accel_dt <= 0.0f) {
#if MAHONY_DEBUG_INTERNAL
		printf("[MAHONY_DEBUG][IMU] FAILED: accel_dt is zero or negative (%.4f)!\n",
		       accel_dt);
#endif
		return;
	}
#if MAHONY_DEBUG_INTERNAL
	printf("\n[MAHONY_DEBUG][IMU] ENTRY: G=[%.3f, %.3f, %.3f], A=[%.3f, %.3f, %.3f], "
	       "g_dt=%.4f, a_dt=%.4f\n",
	       gx, gy, gz, ax, ay, az, gyro_dt, accel_dt);
	printf("[MAHONY_DEBUG][IMU] integralFB before: [%.4f, %.4f, %.4f]\n", integralFBx,
	       integralFBy, integralFBz);
#endif

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		float halfvx = q1 * q3 - q0 * q2;
		float halfvy = q0 * q1 + q2 * q3;
		float halfvz = q0 * q0 - 0.5f + q3 * q3;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * accel_dt;
			integralFBy += twoKi * halfey * accel_dt;
			integralFBz += twoKi * halfez * accel_dt;
			if (integralFBx > INTEGRAL_LIMIT)
				integralFBx = INTEGRAL_LIMIT;
			else if (integralFBx < -INTEGRAL_LIMIT)
				integralFBx = -INTEGRAL_LIMIT;
			if (integralFBy > INTEGRAL_LIMIT)
				integralFBy = INTEGRAL_LIMIT;
			else if (integralFBy < -INTEGRAL_LIMIT)
				integralFBy = -INTEGRAL_LIMIT;
			if (integralFBz > INTEGRAL_LIMIT)
				integralFBz = INTEGRAL_LIMIT;
			else if (integralFBz < -INTEGRAL_LIMIT)
				integralFBz = -INTEGRAL_LIMIT;
		} else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
	}

#if MAHONY_DEBUG_INTERNAL
	printf("[MAHONY_DEBUG][IMU] integralFB after: [%.4f, %.4f, %.4f]\n", integralFBx,
	       integralFBy, integralFBz);
#endif

	float final_gx = gx + twoKp * halfex;
	float final_gy = gy + twoKp * halfey;
	float final_gz = gz + twoKp * halfez;

#if MAHONY_DEBUG_INTERNAL
	printf("[MAHONY_DEBUG][IMU] Gyro for integration (raw+prop): [%.3f, %.3f, %.3f]\n",
	       final_gx, final_gy, final_gz);
#endif

	MahonyAHRSupdateGyro(final_gx, final_gy, final_gz, gyro_dt);
}

/**
 * @brief 快速反平方根
 */
float invSqrt(float x)
{
	if (x <= 0.0f)
		return 0.0f;
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
