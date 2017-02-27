#ifndef _IMU_H_
#define _IMU_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>
//----------------------------------------------------------------------------------------------------
// Variable declaration
extern  float beta;				// algorithm gain

extern  float twoKp;			// 2 * proportional gain (Kp)
extern  float twoKi;			// 2 * integral gain (Ki)
extern  float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void GetTheEuler(float *Roll,float *Pitch,float *Yaw);

void ZghAHRSupdate(float gx, float gy, float gz, float mx, float my, float mz);
void Madgwick_ZGH_AHRSupdate(float gx, float gy, float gz, float mx, float my, float mz);

//void IMU_Gyro_Updata_Mag_Correct(float gx, float gy, float gz, float mx, float my, float mz);
void IMU_Gyro_Updata_Mag_Correct(float yaw, float pitch, float mx, float my, float mz);
void IMU_Mag_Updata(float mx, float my, float mz,float *Roll);
void QuaternUpdate(float Roll,float Pitch,float Yaw);
void AHRSupdate(float gx, float gy, float gz);
#ifdef __cplusplus
}
#endif
#endif
