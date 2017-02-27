#ifndef _PPM_ENCODE_H_
#define _PPM_ENCODE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
void PPM1_SetRatio(uint32_t ratio);
void PPM2_SetRatio(uint32_t ratio);
void PPM3_SetRatio(uint32_t ratio);
void PPM4_SetRatio(uint32_t ratio);
void Motor_Range_Set(void);
void Motor_Start_Test(void);
#ifdef __cplusplus
}
#endif
#endif
