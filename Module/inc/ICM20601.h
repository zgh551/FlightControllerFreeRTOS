#ifndef _ICM20601_H_
#define _ICM20601_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#define ICM20601_SELF_TEST_XG           ((uint8_t)0x00)
#define ICM20601_SELF_TEST_YG           ((uint8_t)0x01)
#define ICM20601_SELF_TEST_ZG           ((uint8_t)0x02)
#define ICM20601_SELF_TEST_XA           ((uint8_t)0x0D)
#define ICM20601_SELF_TEST_YA           ((uint8_t)0x0E)
#define ICM20601_SELF_TEST_ZA           ((uint8_t)0x0F)
#define ICM20601_XG_OFFSET_H            ((uint8_t)0x13)
#define ICM20601_XG_OFFSET_L            ((uint8_t)0x14)
#define ICM20601_YG_OFFSET_H            ((uint8_t)0x15)
#define ICM20601_YG_OFFSET_L            ((uint8_t)0x16)
#define ICM20601_ZG_OFFSET_H            ((uint8_t)0x17)
#define ICM20601_ZG_OFFSET_L            ((uint8_t)0x18)
#define ICM20601_SMPLRT_DIV             ((uint8_t)0x19)
#define ICM20601_CONFIG                 ((uint8_t)0x1A)  
#define ICM20601_GYRO_CONFIG            ((uint8_t)0x1B)
#define ICM20601_ACCEL_CONFIG           ((uint8_t)0x1C)
#define ICM20601_ACCEL_CONFIG2          ((uint8_t)0x1D)  
#define ICM20601_LP_ACCEL_ODR           ((uint8_t)0x1E)
#define ICM20601_MOT_THR                ((uint8_t)0x1F)
#define ICM20601_FIFO_EN                ((uint8_t)0x23)
#define ICM20601_FSYNC_INT              ((uint8_t)0x36)  
#define ICM20601_INT_PIN_CFG            ((uint8_t)0x37)
#define ICM20601_INT_ENABLE             ((uint8_t)0x38)
#define ICM20601_INT_STATUS             ((uint8_t)0x3A)
#define ICM20601_ACCEL_XOUT_H           ((uint8_t)0x3B)
#define ICM20601_ACCEL_XOUT_L           ((uint8_t)0x3C)
#define ICM20601_ACCEL_YOUT_H           ((uint8_t)0x3D)
#define ICM20601_ACCEL_YOUT_L           ((uint8_t)0x3E)
#define ICM20601_ACCEL_ZOUT_H           ((uint8_t)0x3F)
#define ICM20601_ACCEL_ZOUT_L           ((uint8_t)0x40)
#define ICM20601_TEMP_OUT_H             ((uint8_t)0x41)
#define ICM20601_TEMP_OUT_L             ((uint8_t)0x42)
#define ICM20601_GYRO_XOUT_H            ((uint8_t)0x43)
#define ICM20601_GYRO_XOUT_L            ((uint8_t)0x44)
#define ICM20601_GYRO_YOUT_H            ((uint8_t)0x45)
#define ICM20601_GYRO_YOUT_L            ((uint8_t)0x46)
#define ICM20601_GYRO_ZOUT_H            ((uint8_t)0x47)
#define ICM20601_GYRO_ZOUT_L            ((uint8_t)0x48)
#define ICM20601_SIGNAL_PATH_RESET      ((uint8_t)0x68)   
#define ICM20601_ACCEL_INTEL_CTRL       ((uint8_t)0x69)  
#define ICM20601_USER_CTRL              ((uint8_t)0x6A)   
#define ICM20601_PWR_MGMT_1             ((uint8_t)0x6B)
#define ICM20601_PWR_MGMT_2             ((uint8_t)0x6C) 
#define ICM20601_FIFO_COUNTH            ((uint8_t)0x72) 
#define ICM20601_FIFO_COUNTL            ((uint8_t)0x73)
#define ICM20601_FIFO_R_W               ((uint8_t)0x74)
#define ICM20601_WHO_AM_I               ((uint8_t)0x75)//ICM20601 ID = 0xAC
#define ICM20601_XA_OFFSET_H            ((uint8_t)0x77)  
#define ICM20601_XA_OFFSET_L            ((uint8_t)0x78)  
#define ICM20601_YA_OFFSET_H            ((uint8_t)0x7A)  
#define ICM20601_YA_OFFSET_L            ((uint8_t)0x7B)
#define ICM20601_ZA_OFFSET_H            ((uint8_t)0x7D)  
#define ICM20601_ZA_OFFSET_L            ((uint8_t)0x7E)
  
void ICM20601_WriteReg( uint8_t writeAddr, uint8_t writeData );
uint8_t ICM20601_ReadReg( uint8_t readAddr );
void ICM20601_WriteRegs(uint8_t writeAddr, uint8_t *writeData, uint8_t lens);
void ICM20601_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens);

void ICM20601_Init(void);
bool ICM20601_TestConnection(void);
void ICM20601_GetFloatData( float *dataIMU );
#ifdef __cplusplus
}
#endif
#endif
