#include "QuaternionEKF.h"
#include "kalman_filter.h"
#include <math.h>
#include <zephyr/kernel.h> // For bool if not available, or use int
#include "algorithm.h"
#include <string.h>

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef bool // If zephyr/kernel.h doesn't provide it or not always included directly
typedef int bool;
#endif

#if DT_HAS_CHOSEN(zephyr_ccm)
__ccm_bss_section QEKF_INS_t QEKF_INS;
__ccm_data_section float IMU_QuaternionEKF_P[16] = default_EKF_P;
__ccm_bss_section float IMU_QuaternionEKF_K[12];
__ccm_bss_section float IMU_QuaternionEKF_H[12];
#elif DT_HAS_CHOSEN(zephyr_dtcm)
__dtcm_bss_section QEKF_INS_t QEKF_INS;
__dtcm_data_section float IMU_QuaternionEKF_P[16] = default_EKF_P;
__dtcm_bss_section float IMU_QuaternionEKF_K[12];
__dtcm_bss_section float IMU_QuaternionEKF_H[12];
#else
QEKF_INS_t QEKF_INS;
float IMU_QuaternionEKF_P[16] = default_EKF_P;
float IMU_QuaternionEKF_K[12];
float IMU_QuaternionEKF_H[12];
#endif

const float IMU_QuaternionEKF_F[16] = default_EKF_F;

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001 (Not directly used in this split, Q1
 * applies to quaternion states)
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float *init_quaternion, float process_noise1, float process_noise2,
			    float measure_noise, float lambda, float lpf)
{
	QEKF_INS.Initialized = 1;
	QEKF_INS.Q1 = process_noise1;
	QEKF_INS.Q2 = process_noise2; // Note: Q2 is not directly used for state noise Q in the
				      // provided code
	QEKF_INS.R = measure_noise;
	QEKF_INS.ChiSquareTestThreshold = 1e-8;
	QEKF_INS.ConvergeFlag = 0;
	QEKF_INS.ErrorCount = 0;
	if (lambda > 1) {
		lambda = 1;
	}
	QEKF_INS.lambda = lambda;
	QEKF_INS.accLPFcoef = lpf;
	QEKF_INS.UpdateCount = 0; // Initialize for LPF logic

	// Initialize g if not defined by PHY_G and auto-probing
#ifndef PHY_G
	QEKF_INS.g = 9.8f; // Default g
#else
	QEKF_INS.g = PHY_G;
#endif

	// Initialize other members of QEKF_INS
	QEKF_INS.hasStoredBias = FALSE;
	QEKF_INS.gyro_dt = 0.0f;
	QEKF_INS.accel_dt = 0.0f;
	memset(QEKF_INS.RawGyro, 0, sizeof(QEKF_INS.RawGyro));
	memset(QEKF_INS.Gyro, 0, sizeof(QEKF_INS.Gyro));
	memset(QEKF_INS.Accel, 0, sizeof(QEKF_INS.Accel)); // Will be set by LPF on first data too
	QEKF_INS.gyro_norm = 0.0f;
	QEKF_INS.accl_norm = 0.0f;
	QEKF_INS.StableFlag = 0; // FALSE
	memset(QEKF_INS.OrientationCosine, 0, sizeof(QEKF_INS.OrientationCosine));
	QEKF_INS.AdaptiveGainScale = 1.0f;

	// Initialize quaternion q and derived Euler angles
	memcpy(QEKF_INS.q, init_quaternion, sizeof(QEKF_INS.q));
	float norm_init_q = invSqrt(QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1] +
				    QEKF_INS.q[2] * QEKF_INS.q[2] + QEKF_INS.q[3] * QEKF_INS.q[3]);
	for (int i = 0; i < 4; ++i) {
		QEKF_INS.q[i] *= norm_init_q;
	}

	QEKF_INS.Yaw =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Pitch =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Roll =
		asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) *
		57.295779513f;

	QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
	QEKF_INS.YawRoundCount = 0;
	QEKF_INS.YawTotalAngle = QEKF_INS.Yaw;

	// 初始化矩阵维度信息
	Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 4, 0, 3);
	Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

	// 姿态初始化
	for (int i = 0; i < 4; i++) {
		QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = init_quaternion[i];
	}
	// Initialize GyroBias to zero or from storage if available
	// QEKF_INS.GyroBias[0] = ...; (This depends on how bias is handled at startup)
	// Assuming hasStoredBias will be set if DTS provides bias, otherwise GyroBias starts at 0
	// or probed.
	if (!QEKF_INS.hasStoredBias) {
		memset(QEKF_INS.GyroBias, 0, sizeof(QEKF_INS.GyroBias));
	}
	// 自定义函数初始化,用于扩展或增加kf的基础功能
	QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
	// We don't probe bias now. (Comment seems outdated, bias is handled)
	QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
	QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

	// 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
	QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
	QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

	memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
	memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief Quaternion EKF Prediction Update using Gyroscope Data
 * @param[in] gx raw gyro x in rad/s
 * @param[in] gy raw gyro y in rad/s
 * @param[in] gz raw gyro z in rad/s
 * @param[in] gyro_dt update period in s
 */
void IMU_QuaternionEKF_Predict_Update(float gx, float gy, float gz, float gyro_dt)
{
	if (!QEKF_INS.Initialized) {
		return;
	}

	QEKF_INS.gyro_dt = gyro_dt;

	// Store raw gyro data for potential bias estimation in measurement update
	QEKF_INS.RawGyro[0] = gx;
	QEKF_INS.RawGyro[1] = gy;
	QEKF_INS.RawGyro[2] = gz;

	float gx_corrected = gx;
	float gy_corrected = gy;
	float gz_corrected = gz;

	if (!QEKF_INS.hasStoredBias) {
		gx_corrected -= QEKF_INS.GyroBias[0];
		gy_corrected -= QEKF_INS.GyroBias[1];
		gz_corrected -= QEKF_INS.GyroBias[2];
	}

	QEKF_INS.Gyro[0] = gx_corrected;
	QEKF_INS.Gyro[1] = gy_corrected;
	QEKF_INS.Gyro[2] = gz_corrected;

	QEKF_INS.StableFlag = 0;
	bool BiasFlag = 0;
	if (QEKF_INS.gyro_norm < 0.2f && fabsf(NormOf3d(QEKF_INS.Accel) - 9.8f) < 0.35f) {
		BiasFlag = true;
	}

#if DT_HAS_CHOSEN(ares_bias) && CONFIG_AUTO_PROBE_GYRO_BIAS
#error Do not use bias specified in dts and auto probe at the same time!!
#endif // DT_HAS_CHOSEN(ares_bias) && CONFIG_AUTO_PROBE_BIAS

#if CONFIG_AUTO_PROBE_GYRO_BIAS
	// 陀螺仪偏置估计：
	// 仅当 BiasFlag 为 true 且 !QEKF_INS.hasStoredBias (即不使用固定的预存偏置) 时更新偏置
	if (BiasFlag && !QEKF_INS.hasStoredBias) {
		if (QEKF_INS.UpdateCount == 0) { // 首次更新时，用原始值初始化偏置估计的LPF
			QEKF_INS.GyroBias[0] = QEKF_INS.RawGyro[0];
			QEKF_INS.GyroBias[1] = QEKF_INS.RawGyro[1];
			QEKF_INS.GyroBias[2] = QEKF_INS.RawGyro[2];
		}
		// 使用原始陀螺仪数据进行低通滤波来更新偏置估计
		QEKF_INS.GyroBias[0] =
			QEKF_INS.GyroBias[0] * 0.9995f + QEKF_INS.RawGyro[0] * 0.0005f;
		QEKF_INS.GyroBias[1] =
			QEKF_INS.GyroBias[1] * 0.9995f + QEKF_INS.RawGyro[1] * 0.0005f;
		QEKF_INS.GyroBias[2] =
			QEKF_INS.GyroBias[2] * 0.9995f + QEKF_INS.RawGyro[2] * 0.0005f;
	}
#endif // CONFIG_AUTO_PROBE_GYRO_BIAS

	QEKF_INS.gyro_norm = invSqrt(gx_corrected * gx_corrected + gy_corrected * gy_corrected +
				     gz_corrected * gz_corrected);
	if (QEKF_INS.gyro_norm !=
	    0.0f) { // Avoid division by zero if gyro_norm is calculated from invSqrt(sum_sq)
		QEKF_INS.gyro_norm = 1.0f / QEKF_INS.gyro_norm;
	} else {
		QEKF_INS.gyro_norm = 0.0f; // Or handle as error / very small number
	}

	// set F
	// 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
	static float halfgxdt, halfgydt,
		halfgzdt; // Keep static if preferred for minor optimization
	halfgxdt = 0.5f * gx_corrected * gyro_dt;
	halfgydt = 0.5f * gy_corrected * gyro_dt;
	halfgzdt = 0.5f * gz_corrected * gyro_dt;

	memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

	QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

	QEKF_INS.IMU_QuaternionEKF.F_data[4] = halfgxdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[7] = -halfgydt;

	QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[11] = halfgxdt;

	QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[13] = halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[14] = -halfgxdt;

	// set Q,过程噪声矩阵
	QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[5] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[10] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[15] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	// Note: R is set in Measurement_Update

	Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

	// 获取预测后的数据 (from FilteredValue, which xhatUpdate will set to xhatminus)
	QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
	QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
	QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
	QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];

	// Normalize quaternion from prediction
	float norm = invSqrt(QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1] +
			     QEKF_INS.q[2] * QEKF_INS.q[2] + QEKF_INS.q[3] * QEKF_INS.q[3]);
	for (int i = 0; i < 4; ++i) {
		QEKF_INS.q[i] *= norm;
	}

	// 利用四元数反解欧拉角 (based on predicted quaternion)
	QEKF_INS.Yaw =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Pitch =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Roll =
		asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) *
		57.295779513f;

	// get Yaw total (based on predicted quaternion)
	if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
		QEKF_INS.YawRoundCount--;
	} else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
		QEKF_INS.YawRoundCount++;
	}
	QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
	QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
}

/**
@brief Quaternion EKF measurement update step.
       Processes gyro for prediction and accel for correction.
       The EKF cycle (predict and correct) runs once per call to this function.

@param[in] gx, gy, gz      Gyroscope readings in rad/s (raw values)
@param[in] gyro_dt         Time interval for gyroscope data in s (used for F and Q matrices)
@param[in] ax, ay, az      Accelerometer readings in m/s² (raw values)
@param[in] accel_dt        Time interval for accelerometer data in s (used for accel LPF)
*/
void IMU_QuaternionEKF_Measurement_Update(float gx_raw, float gy_raw, float gz_raw, float gyro_dt,
					  float ax, float ay, float az, float accel_dt)
{
	if (gyro_dt <= 0 || accel_dt <= 0) {
		return;
	}
	// 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
	static float halfgxdt, halfgydt, halfgzdt;
	static float accelInvNorm;

	// 将传入的 dt 分别赋值给 QEKF_INS 内部使用的变量
	// QEKF_INS.gyro_dt 用于陀螺仪相关的预测步骤 (F矩阵, Q矩阵)
	// QEKF_INS.accel_dt 用于加速度计的低通滤波
	QEKF_INS.gyro_dt = gyro_dt;
	QEKF_INS.accel_dt = accel_dt;

	// 创建陀螺仪工作副本，用于后续可能的偏置补偿
	float gx_for_F = gx_raw;
	float gy_for_F = gy_raw;
	float gz_for_F = gz_raw;

	// 如果hasStoredBias为false，意味着我们正在使用或估计陀螺仪偏置
	// 此时，从陀螺仪读数中减去当前估计的偏置，得到用于F矩阵的角速度
	// 如果hasStoredBias为true，意味着使用了一个固定的、预存的偏置（或者认为偏置为0且不估计）
	// 此时，QEKF_INS.GyroBias中可能是预设值。注释说明不明确，但通常应减去当前最佳偏置估计。
	// 为保持与原逻辑的兼容性（仅当!hasStoredBias时补偿），同时修复偏置估计bug，我们如下处理：
	// F矩阵使用的陀螺仪值：
	if (!QEKF_INS.hasStoredBias) { // 仅当不是用“已存储（通常指固定或已知）”偏置时，才减去动态估计的偏置
		gx_for_F -= QEKF_INS.GyroBias[0];
		gy_for_F -= QEKF_INS.GyroBias[1];
		gz_for_F -= QEKF_INS.GyroBias[2];
	}
	// 如果 QEKF_INS.hasStoredBias 为 true，则 gx_for_F, gy_for_F, gz_for_F 保持为原始输入值
	// 这暗示着如果有一个“已存储”的偏置，它可能已经被外部处理了，或者这个 EKF
	// 假定原始输入已经是补偿过的。 然而，更标准的做法是 QEKF_INS.GyroBias
	// 总是代表要减去的偏置，无论它是估计的还是固定的。
	// 为了最小化改动并修复偏置估计bug，这里我们让F矩阵的陀螺仪值按原逻辑处理。
	// 偏置估计部分（后续）将明确使用 gx_raw, gy_raw, gz_raw。

	QEKF_INS.Gyro[0] = gx_for_F; // 存储用于F矩阵的（可能补偿了偏置的）陀螺仪值
	QEKF_INS.Gyro[1] = gy_for_F;
	QEKF_INS.Gyro[2] = gz_for_F;

	// set F (状态转移矩阵)
	// 使用 gyro_dt (即 QEKF_INS.gyro_dt) 和补偿后的陀螺仪数据 (gx_for_F, gy_for_F, gz_for_F)
	halfgxdt = 0.5f * gx_for_F * QEKF_INS.gyro_dt;
	halfgydt = 0.5f * gy_for_F * QEKF_INS.gyro_dt;
	halfgzdt = 0.5f * gz_for_F * QEKF_INS.gyro_dt;

	memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

	QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

	QEKF_INS.IMU_QuaternionEKF.F_data[4] = halfgxdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[7] = -halfgydt;

	QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[11] = halfgxdt;

	QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[13] = halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[14] = -halfgxdt;

	// accel low pass filter, 使用 accel_dt (即 QEKF_INS.accel_dt)
	if (QEKF_INS.UpdateCount == 0) {
		QEKF_INS.Accel[0] = ax;
		QEKF_INS.Accel[1] = ay;
		QEKF_INS.Accel[2] = az;
	}
	// 注意：这里的 accel_dt 是 QEKF_INS.accel_dt
	QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef /
				    (QEKF_INS.accel_dt + QEKF_INS.accLPFcoef) +
			    ax * QEKF_INS.accel_dt / (QEKF_INS.accel_dt + QEKF_INS.accLPFcoef);
	QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef /
				    (QEKF_INS.accel_dt + QEKF_INS.accLPFcoef) +
			    ay * QEKF_INS.accel_dt / (QEKF_INS.accel_dt + QEKF_INS.accLPFcoef);
	QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef /
				    (QEKF_INS.accel_dt + QEKF_INS.accLPFcoef) +
			    az * QEKF_INS.accel_dt / (QEKF_INS.accel_dt + QEKF_INS.accLPFcoef);

	// set z (观测向量), 单位化重力加速度向量
	accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] +
			       QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
			       QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
	for (uint8_t i = 0; i < 3; ++i) {
		QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm;
	}

	// get body state
	// gyro_norm 基于用于F矩阵的陀螺仪数据计算
	QEKF_INS.gyro_norm =
		1.0f / invSqrt(gx_for_F * gx_for_F + gy_for_F * gy_for_F + gz_for_F * gz_for_F);
	QEKF_INS.accl_norm = 1.0f / accelInvNorm;

	float acc_input[3] = {ax, ay, az};               // 使用原始加速度计读数判断稳定性
	if (fabsf(NormOf3d(acc_input) - 9.8f) < 0.25f) { // 使用传入的原始ax,ay,az进行判断
		QEKF_INS.StableFlag = 1;
	} else {
		QEKF_INS.StableFlag = 0;
	}

	// set Q R, 过程噪声和观测噪声矩阵
	// Q 矩阵使用 gyro_dt (即 QEKF_INS.gyro_dt)
	QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[5] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[10] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[15] = QEKF_INS.Q1 * QEKF_INS.gyro_dt;
	// R 矩阵通常不依赖dt
	QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
	QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
	QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

	// 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
	// EKF的一个完整预测和校正周期在此处执行
	Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

	// 获取融合后的数据,包括四元数
	QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
	QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
	QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
	QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];

	// Normalize quaternion from prediction
	float norm = invSqrt(QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1] +
			     QEKF_INS.q[2] * QEKF_INS.q[2] + QEKF_INS.q[3] * QEKF_INS.q[3]);
	for (int i = 0; i < 4; ++i) {
		QEKF_INS.q[i] *= norm;
	}

	// 利用四元数反解欧拉角
	QEKF_INS.Yaw =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Pitch =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Roll =
		asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) *
		57.295779513f;

	// get Yaw total
	if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
		QEKF_INS.YawRoundCount--;
	} else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
		QEKF_INS.YawRoundCount++;
	}
	QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
	QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
	QEKF_INS.UpdateCount++;
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
	static float doubleq0, doubleq1, doubleq2, doubleq3;
	/* H
	0     1     2     3
	4     5     6     7
	8     9    10    11
	*/
	// set H
	doubleq0 = 2 * kf->xhatminus_data[0];
	doubleq1 = 2 * kf->xhatminus_data[1];
	doubleq2 = 2 * kf->xhatminus_data[2];
	doubleq3 = 2 * kf->xhatminus_data[3];

	memset(kf->H_data, 0, sizeof(float) * kf->zSize * kf->xhatSize);

	kf->H_data[0] = -doubleq2;
	kf->H_data[1] = doubleq3;
	kf->H_data[2] = -doubleq0;
	kf->H_data[3] = doubleq1;

	kf->H_data[4] = doubleq1;
	kf->H_data[5] = doubleq0;
	kf->H_data[6] = doubleq3;
	kf->H_data[7] = doubleq2;

	kf->H_data[8] = doubleq0;
	kf->H_data[9] = -doubleq1;
	kf->H_data[10] = -doubleq2;
	kf->H_data[11] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
	static float q0, q1, q2, q3;

	kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
	kf->temp_matrix.numRows = kf->H.numRows;
	kf->temp_matrix.numCols = kf->Pminus.numCols;
	kf->MatStatus =
		Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
	kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
	kf->temp_matrix1.numCols = kf->HT.numCols;
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT,
					&kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
	kf->S.numRows = kf->R.numRows;
	kf->S.numCols = kf->R.numCols;
	kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
	kf->MatStatus =
		Matrix_Inverse(&kf->S, &kf->temp_matrix1); // temp_matrix1 = inv(H·P'(k)·HT + R)

	q0 = kf->xhatminus_data[0];
	q1 = kf->xhatminus_data[1];
	q2 = kf->xhatminus_data[2];
	q3 = kf->xhatminus_data[3];

	kf->temp_vector.numRows = kf->H.numRows;
	kf->temp_vector.numCols = 1;
	// 计算预测得到的重力加速度方向(通过姿态获取的)
	kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
	kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
	kf->temp_vector_data[2] =
		q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

	// 计算预测值和各个轴的方向余弦
	for (uint8_t i = 0; i < 3; ++i) {
		// Protect against acosf domain errors if temp_vector_data[i] is slightly outside
		// [-1, 1] due to precision
		float val = fabsf(kf->temp_vector_data[i]);
		if (val > 1.0f) {
			val = 1.0f;
		}
		QEKF_INS.OrientationCosine[i] = acosf(val);
	}

	// 利用加速度计数据修正
	kf->temp_vector1.numRows = kf->z.numRows;
	kf->temp_vector1.numCols = 1;
	kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector,
					&kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

	// chi-square test,卡方检验
	kf->temp_matrix.numRows = kf->temp_vector1.numRows;
	kf->temp_matrix.numCols = 1;
	kf->MatStatus = Matrix_Multiply(
		&kf->temp_matrix1, &kf->temp_vector1,
		&kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
	kf->temp_vector.numRows = 1;
	kf->temp_vector.numCols = kf->temp_vector1.numRows;
	kf->MatStatus = Matrix_Transpose(&kf->temp_vector1,
					 &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
	kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
	// rk is small,filter converged/converging
	if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold) {
		QEKF_INS.ConvergeFlag = 1;
	}
	// rk is bigger than three but once converged
	if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
		if (QEKF_INS.StableFlag) {
			QEKF_INS.ErrorCount++; // 载体静止时仍无法通过卡方检验
		} else {
			QEKF_INS.ErrorCount = 0;
		}

		if (QEKF_INS.ErrorCount > 50) {
			// 滤波器发散
			QEKF_INS.ConvergeFlag = 0;
			kf->SkipEq5 = FALSE; // step-5 is cov mat P updating, allow P update with
					     // measurement
		} else {
			//  残差未通过卡方检验 仅预测
			//  xhat(k) = xhat'(k)
			//  P(k) = P'(k)
			memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
			memcpy(kf->P_data, kf->Pminus_data,
			       sizeof(float) * kf->xhatSize * kf->xhatSize);
			kf->SkipEq5 =
				TRUE; // part5 is P updating, skip it as we are using predicted P
			return;
		}
	} else // if divergent or rk is not that big/acceptable,use adaptive gain
	{
		// scale adaptive,rk越小则增益越大,否则更相信预测值
		if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold &&
		    QEKF_INS.ConvergeFlag) {
			QEKF_INS.AdaptiveGainScale =
				(QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) /
				(0.9f * QEKF_INS.ChiSquareTestThreshold);
		} else {
			QEKF_INS.AdaptiveGainScale = 1;
		}
		QEKF_INS.ErrorCount = 0;
		kf->SkipEq5 = FALSE; // Allow P update with measurement
	}

	// cal kf-gain K
	kf->temp_matrix.numRows = kf->Pminus.numRows;
	kf->temp_matrix.numCols = kf->HT.numCols;
	kf->MatStatus =
		Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

	// implement adaptive
	for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i) {
		kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
	}

	kf->temp_vector.numRows = kf->K.numRows;
	kf->temp_vector.numCols = 1;
	kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1,
					&kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

	kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
	// Note: P update (Eq5: P = (I - KH)Pminus) is handled by Kalman_Filter_Update if SkipEq5 is
	// FALSE. If SkipEq5 is TRUE (e.g. chi-square rejection), P is already set to Pminus above.
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
	memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
	memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
	memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
	if (x == 0.0f) {
		return 0.0f; // Avoid NaN for x=0, though typically x > 0 for norms
	}
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	// y = y * (1.5f - (halfx * y * y)); // Optional: second iteration for more precision
	return y;
}
