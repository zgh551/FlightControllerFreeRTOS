/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f4xx.h"
    
/* configure file */
#include "BoardDefine.h"
#include "config.h"
    
/* Module File include */
#include "HMC5983.h"
#include "ICM20601.h"
#include "ms5611.h"
#include "nRF24L01.h"
#include "PPM_Encode.h" 
#include "motors.h"

//FAST
#include "sdio_sd.h"
#include "ff.h"			/* Declarations of FatFs API */
#include "diskio.h"		/* Declarations of device I/O functions */
#include "malloc.h"	
    
/* FreeRTOS Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Application File */
#include "system.h"    
#include "LedSeq.h"
    
/*DLL = data link layer*/
#include "RadioLink.h"
#include "CRTP.h"
#include "commander.h"
#include "ComModule.h"

/*HAL = Hardware Aplication Level*/
#include "IMU.h"
#include "imu_types.h"
    
/* Utils file */
#include "num.h"
#include "filter.h"
    
/*Cintrol*/
#include "stabilizer.h"
#include "sensors.h"
#include "sensfusion6.h"
#include "estimator.h"
#include "sitaw.h"
#include "pid.h"
#include "pidctrl.h" 
#include "controller.h"
#include "attitude_controller.h"
#include "position_estimator.h"
#include "position_controller.h"
#include "power_distribution.h"    

extern FIL *fil;
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);

extern void Delay(__IO uint32_t nTime);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
