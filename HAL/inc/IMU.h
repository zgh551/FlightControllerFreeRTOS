/**
 ******************************************************************************
 * @file    IMU.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    01-Merch-2017
 * @brief   This file
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2017 
 ******************************************************************************  
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "imu_types.h"
/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
 /**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ    500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  4
/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)
/* Exported macro ------------------------------------------------------------*/
   
/* Exported variables --------------------------------------------------------*/
   
/* Exported function prototypes ----------------------------------------------*/
void IMU_Init(void); 
bool IMU_Test(void);
void imu6Read(Axis3f *gyro,Axis3f *acc);
bool imu6IsCalibrated(void);
bool imuHasBarometer(void);
bool imuHasMangnetometer(void);
void imu9Read(Axis3f *gyro,Axis3f *acc,Axis3f *mag);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT ZGH *****END OF FILE****/
