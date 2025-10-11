//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

#include <stdbool.h>

#ifndef MahonyAHRS_h
#define MahonyAHRS_h

typedef struct {
	float q[4];
	float Accel[3];
	float Gyro[3];
	float Mag[3];
	float Yaw, Pitch, Roll;
	float g;
} MahonyAHRS_INS_t;

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;          // 2 * proportional gain (Kp)
extern volatile float twoKi;          // 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

extern MahonyAHRS_INS_t MahonyAHRS_INS;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float gyro_dt, float ax, float ay, float az,
		      float mx, float my, float mz, float accel_dt);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float gyro_dt, float ax, float ay, float az,
			 float accel_dt);
void MahonyAHRSupdateGyro(float gx, float gy, float gz, float gyro_dt);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
