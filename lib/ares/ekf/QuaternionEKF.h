/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h"
#include <zephyr/drivers/sensor.h>

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

#define PHY_G 9.7887f

typedef struct {
	uint8_t Initialized;
	KalmanFilter_t IMU_QuaternionEKF;
	uint8_t ConvergeFlag;
	uint8_t StableFlag;
	uint64_t ErrorCount;
	uint64_t UpdateCount;

	float q[4];         // 四元数估计值
	float GyroBias[3];  // 陀螺仪零偏估计值
	float AccelBias[3]; // 加速度计零偏估计值
	float AccelBeta[3]; // 加速度计标定系数

	float accel_dt; // 加速度计采样周期
	float gyro_dt;  // 陀螺仪采样周期

	float g; // 重力加速度

	float Gyro[3];
	float Accel[3];

	float OrientationCosine[3];

	float accLPFcoef;
	float gyro_norm;
	float accl_norm;
	float AdaptiveGainScale;

	float Roll;
	float Pitch;
	float Yaw;

	float YawTotalAngle;

	float Q1; // 四元数更新过程噪声
	float Q2; // 陀螺仪零偏过程噪声
	float R;  // 加速度计量测噪声

	mat ChiSquare;
	float ChiSquare_Data[1];      // 卡方检验检测函数
	float ChiSquareTestThreshold; // 卡方检验阈值
	float lambda;                 // 渐消因子

	int16_t YawRoundCount;

	float YawAngleLast;

	uint64_t UpdateTime;

	bool hasStoredBias; // 是否存储过偏置

	// New fields for split update
	float RawGyro[3]; // Store raw gyro for bias estimation
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float *init_quaternion, float process_noise1, float process_noise2,
			    float measure_noise, float lambda, float lpf);

void IMU_QuaternionEKF_Predict_Update(float gx, float gy, float gz, float gyro_dt);
void IMU_QuaternionEKF_Measurement_Update(float gx, float gy, float gz, float gyro_dt, float ax,
					  float ay, float az, float accel_dt);
void CalcBias(float *q, float *accel, float g, float *bias);

#define default_EKF_F                                                                              \
	{                                                                                          \
		1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,                                    \
	}

#define default_EKF_P                                                                              \
	{                                                                                          \
		100000, 0.1, 0.1,    0.1, 0.1, 100000, 0.1, 0.1,                                   \
		0.1,    0.1, 100000, 0.1, 0.1, 0.1,    0.1, 100000,                                \
	}

#endif
