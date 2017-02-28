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

/* Exported typedef ----------------------------------------------------------*/
   typedef struct {
         int16_t x;
         int16_t y;
         int16_t z;
 } Axis3i16;

 typedef struct {
         int32_t x;
         int32_t y;
         int32_t z;
 } Axis3i32;

 typedef struct _Axis3f{
         float x;
         float y;
         float z;
 } Axis3f;
/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void IMU_Init(void); 
 
#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT ZGH *****END OF FILE****/
