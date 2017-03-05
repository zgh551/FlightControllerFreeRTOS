/**
 ******************************************************************************
 * @file    Motors.h
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
#ifndef __MOTORS_H__
#define __MOTORS_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported typedef ----------------------------------------------------------*/
  typedef enum
{
  BRUSHED,
  BRUSHLESS
} motorsDrvType;

typedef struct
{
  motorsDrvType drvType;
  uint32_t      gpioPerif;
  GPIO_TypeDef* gpioPort;
  uint32_t      gpioPin;
  uint32_t      gpioPinSource;
  uint32_t      gpioOType;
  uint32_t      gpioAF;
  uint32_t      timPerif;
  TIM_TypeDef*  tim;
  uint16_t      timPolarity;
  uint32_t      timDbgStop;
  uint32_t      timPeriod;
  uint16_t      timPrescaler;
  /* Function pointers */
  void (*setCompare)(TIM_TypeDef* TIMx, uint32_t Compare);
  uint32_t (*getCompare)(TIM_TypeDef* TIMx);
  
  void (*ocInit)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void (*preloadConfig)(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
} MotorPerifDef;
/* Exported define -----------------------------------------------------------*/
// The following defines gives a PWM of 8 bits at ~328KHz for a sysclock of 180MHz
// CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702 regulator is affected.
  #define TIM_CLOCK_HZ 90000000
  #define MOTORS_PWM_BITS           8
  #define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
  #define MOTORS_PWM_PRESCALE       0
  #define MOTORS_TIM_BEEP_CLK_FREQ  (90000000L / 5)
  #define MOTORS_POLARITY           TIM_OCPolarity_High

// Abstraction of ST lib functions
  #define MOTORS_GPIO_MODE          GPIO_Mode_AF
  #define MOTORS_RCC_GPIO_CMD       RCC_AHB1PeriphClockCmd
  #define MOTORS_RCC_TIM_CMD        RCC_APB1PeriphClockCmd
  #define MOTORS_TIM_DBG_CFG        DBGMCU_APB2PeriphConfig
  #define MOTORS_GPIO_AF_CFG(a,b,c) GPIO_PinAFConfig(a,b,c)

// Compensate thrust depending on battery voltage so it will produce about the same
// amount of thrust independent of the battery voltage. Based on thrust measurement.
  #define ENABLE_THRUST_BAT_COMPENSATED

/**
* *VARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
* brushed motors connected they can turn on at full speed when it is powered on!
*
* Generates a PWM wave (50 - 400 Hz update rate with 1-2 ms high pulse) using the timer. That way we can use the same
* base as for the regular PWM driver. This means it will be a PWM with a period of the update rate configured to be high
* only in the 1-2 ms range.
*/
  #define BLMC_PERIOD 0.0025   // 2.5ms = 400Hz
  #define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
  #define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
  #define MOTORS_BL_PWM_CNT_FOR_1MS    (uint32_t)(TIM_CLOCK_HZ * 0.001 / MOTORS_BL_PWM_PRESCALE_RAW)
  #define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
  #define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)
  #define MOTORS_BL_POLARITY           TIM_OCPolarity_Low

#define NBR_OF_MOTORS 4
// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

// Sound defines
#define C4    262
#define DES4  277
#define D4    294
#define ES4   311
#define E4    330
#define F4    349
#define GES4  370
#define G4    392
#define AS4   415
#define A4    440
#define B4    466
#define H4    493
#define C5    523
#define DES5  554
#define D5    587
#define ES5   622
#define E5    659
#define F5    698
#define GES5  740
#define G5    783
#define AS5   830
#define A5    880
#define B5    932
#define H5    987
#define C6    1046
#define DES6  1108
#define D6    1174
#define ES6   1244
#define E6    1318
#define F6    1396
#define GES6  1479
#define G6    1567
#define AS6   1661
#define A6    1760
#define B6    1864
#define H6    1975
#define C7    2093
#define DES7  2217
#define D7    2349
#define ES7   2489
#define E7    2637
#define F7    2793
#define GES7  2959
#define G7    3135
#define AS7   3322
#define A7    3520
#define H7    3729
#define B7    3951

// Sound duration defines
#define EIGHTS 125
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0
  
  
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT ZGH *****END OF FILE****/
