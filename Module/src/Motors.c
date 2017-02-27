/**
******************************************************************************
* @file    Motors.c
* @author  Zhu Guohua
* @version V1.0
* @date    27-January-2017
* @brief   This file is high level communication module
******************************************************************************
* @attention
* Quadcopter = qdcpt
* COPYRIGHT 2017 
******************************************************************************  
Motors.c - lower level hardware module
*/
#include "Motors.h"

static bool isInit = false;

void MotorsInit(void)
{
  if (isInit)
  return;

  PPM_GPIO_Config();
  PPM_TIM_Config();
  printf("PPM  Init Finish\n");

  isInit = true;
}

void motorsSetRatio(MOTOR id, uint16_t ratio)
{
  switch(id)
  {
    case MOTOR_M1:
      TIM_SetCompare1(PPM_TIM,1000 + ratio);
      break;
    case MOTOR_M2:
      TIM_SetCompare2(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M3:
      TIM_SetCompare3(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M4:
      TIM_SetCompare4(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
  }
}