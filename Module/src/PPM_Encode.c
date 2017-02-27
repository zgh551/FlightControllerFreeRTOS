#include "PPM_Encode.h"

//99.99 - 199.98
void PPM1_SetRatio(uint32_t ratio)
{
  TIM_SetCompare1(PPM_TIM,1000 + ratio);
}

void PPM2_SetRatio(uint32_t ratio)
{
  TIM_SetCompare2(PPM_TIM,1000 + ratio);
}

void PPM3_SetRatio(uint32_t ratio)
{
  TIM_SetCompare3(PPM_TIM,1000 + ratio);
}

void PPM4_SetRatio(uint32_t ratio)
{
  TIM_SetCompare4(PPM_TIM,1000 + ratio);
}


void Motor_Start_Test(void)
{
  Delay(3000);
  PPM1_SetRatio(52);//125 start
  PPM2_SetRatio(52);
  PPM3_SetRatio(52);
  PPM4_SetRatio(52);
}

void Motor_Range_Set(void)
{
  PPM1_SetRatio(1000);//125 start
  PPM2_SetRatio(1000);
  PPM3_SetRatio(1000);
  PPM4_SetRatio(1000);
  Delay(3000);
  PPM1_SetRatio(0);//125 start
  PPM2_SetRatio(0);
  PPM3_SetRatio(0);
  PPM4_SetRatio(0);
  Delay(1000);
}