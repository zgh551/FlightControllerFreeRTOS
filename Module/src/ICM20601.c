#include "ICM20601.h"

void ICM20601_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  MP9250_CS_LOW;
  SPI1_RW(writeAddr);
  SPI1_RW(writeData);
  MP9250_CS_HIGH;
}

uint8_t ICM20601_ReadReg( uint8_t readAddr )
{
  uint8_t readData = 0;
  MP9250_CS_LOW;
  SPI1_RW(0x80 | readAddr);
  readData = SPI1_RW(0x00);  
  MP9250_CS_HIGH;

  return readData;
}

void ICM20601_WriteRegs(uint8_t writeAddr, uint8_t *writeData, uint8_t lens)
{
  MP9250_CS_LOW;
  SPI1_RW(writeAddr);
  for(uint8_t i = 0; i < lens; i++)
    SPI1_RW(writeData[i]);
  MP9250_CS_HIGH;
}

void ICM20601_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens)
{
  MP9250_CS_LOW;
  SPI1_RW(0x80 | readAddr);
  for(uint8_t i = 0; i < lens; i++)
    readData[i] = SPI1_RW(0x00);
  MP9250_CS_HIGH;
}

bool ICM20601_TestConnection(void)
{
  uint8_t ID;
  ID = ICM20601_ReadReg(ICM20601_WHO_AM_I);
  if(ID != 0x12)
  {
    printf("ICM20601 ID=%d\n",ID);
    return ERROR;
  }
  else
  {
    return SUCCESS;
  }
}

void ICM20601_GetFloatData( float *dataIMU )
{
  uint8_t tmpRead[14];

  ICM20601_ReadRegs(ICM20601_ACCEL_XOUT_H, tmpRead, 14);

  dataIMU[0] = 25.0f + (Byte16(int16_t, tmpRead[6],  tmpRead[7]))/326.8f;    // Temp
  dataIMU[1] = (float)((Byte16(int16_t, tmpRead[0],  tmpRead[1])) /2048.0f);// Acc.X
  dataIMU[2] = (float)((Byte16(int16_t, tmpRead[2],  tmpRead[3])) /2048.0f);// Acc.Y
  dataIMU[3] = (float)((Byte16(int16_t, tmpRead[4],  tmpRead[5])) /2048.0f);// Acc.Z
  dataIMU[4] = (float)((Byte16(int16_t, tmpRead[8],  tmpRead[9])) /16.4f);//Gyr.X
  dataIMU[5] = (float)((Byte16(int16_t, tmpRead[10], tmpRead[11]))/16.4f);//Gyr.Y
  dataIMU[6] = (float)((Byte16(int16_t, tmpRead[12], tmpRead[13]))/16.4f);//Gyr.Z 
}

void ICM20601_GetData( Axis3f *gyro,Axis3f *acc )
{
  uint8_t tmpRead[14];

  ICM20601_ReadRegs(ICM20601_ACCEL_XOUT_H, tmpRead, 14);

  acc->x  = (float)((Byte16(int16_t, tmpRead[0],  tmpRead[1])) /2048.0f);// Acc.X
  acc->y  = (float)((Byte16(int16_t, tmpRead[2],  tmpRead[3])) /2048.0f);// Acc.Y
  acc->z  = (float)((Byte16(int16_t, tmpRead[4],  tmpRead[5])) /2048.0f);// Acc.Z
  gyro->x = (float)((Byte16(int16_t, tmpRead[8],  tmpRead[9])) /16.4f);//Gyr.X
  gyro->y = (float)((Byte16(int16_t, tmpRead[10], tmpRead[11]))/16.4f);//Gyr.Y
  gyro->z = (float)((Byte16(int16_t, tmpRead[12], tmpRead[13]))/16.4f);//Gyr.Z 
}
static void ICM20601_Offset_Correct(void)
{

  uint8_t reg_offset[2],i;
  
  uint8_t icm20601_offset_reg[6]={
  ICM20601_XG_OFFSET_H,
  ICM20601_YG_OFFSET_H,
  ICM20601_ZG_OFFSET_H,
  ICM20601_XA_OFFSET_H,
  ICM20601_YA_OFFSET_H,
  ICM20601_ZA_OFFSET_H
  };
  
  int16_t OffsetValue[6]={
    -50,//set gyro X axis offset
    0  ,//set gyro Y asix offset
    0  ,//set gyro Z asix offset
    2  ,//Set acc X asix offset
    2  ,//Set acc Y asix offset
    28  //Set acc Z asix offset
  };
  
  for(i=0;i<6;i++)
  {
    ICM20601_ReadRegs(icm20601_offset_reg[i],reg_offset,2);
    OffsetValue[i] += Byte16(int16_t, reg_offset[0], reg_offset[1]);
    reg_offset[0] = (uint8_t)((OffsetValue[i] >> 8)&0xff);
    reg_offset[1] = (uint8_t)(OffsetValue[i] & 0xff);
    ICM20601_WriteRegs(icm20601_offset_reg[i],reg_offset,2);
    vTaskDelay( pdMS_TO_TICKS( 1 ) );
  }
}

void ICM20601_Init(void)
{
  uint8_t i;
  uint8_t ConfigParameter[11][2]={
    {ICM20601_PWR_MGMT_1   ,0x80},//Reset Device
    {ICM20601_PWR_MGMT_1   ,0x04},//Clock Source
    {ICM20601_INT_PIN_CFG  ,0x10},//Set INT_ANYRD_2CLEAR
    {ICM20601_INT_ENABLE   ,0x01},//Set RAW_RDY_EN
    {ICM20601_PWR_MGMT_2   ,0x00},//Enable Acc & Gyro
    {ICM20601_SMPLRT_DIV   ,0x00},//Sample Rate Divider 1kHz
    {ICM20601_GYRO_CONFIG  ,0x18},//set : ¡À2000dps  250hz
    {ICM20601_ACCEL_CONFIG ,0x18},//set : ¡À16g 218.1hz
    {ICM20601_CONFIG       ,0x00},//set : 250hz for gyro
    {ICM20601_ACCEL_CONFIG2,0x00},//set : 7.8HZ rate: 1Khz
    {ICM20601_USER_CTRL    ,0x10} //Disable I2C Slave module and put the serial interface in SPI mode only.
  };

  for(i=0;i<11;i++)
  {
    ICM20601_WriteReg(ConfigParameter[i][0],ConfigParameter[i][1]);
    vTaskDelay( pdMS_TO_TICKS( 10 ) );
  }
  ICM20601_Offset_Correct();
}
//  int16_t Acc_X_Offset,Acc_Y_Offset,Acc_Z_Offset;
//  int16_t GYRO_X_Offset,GYRO_Y_Offset,GYRO_Z_Offset;
//void ICM20601_GetSelfCheckData(uint8_t *dat)
//{
////  float GYRO[3],ACCEL[3];
//  ICM20601_ReadRegs(ICM20601_SELF_TEST_XG, dat  ,3);
//  Delay(10);
//  ICM20601_ReadRegs(ICM20601_SELF_TEST_XA, &dat[3] ,3);
//  Delay(10);
//}

//static void ICM20601_Selfcheck(void)
//{
//  uint8_t SelfcheckData[14];
//  float imu_data_selfcheck[8],imu_data_temp[8];
//  ICM20601_WriteReg(ICM20601_GYRO_CONFIG,0xF8);//Enable the self check
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_ACCEL_CONFIG,0xF8);//Enable the self check
//  Delay(100);
//  ICM20601_getData(imu_data_selfcheck);
//  printf("self value:\r\n");
//  printf("%f %f %f %f %f %f\r\n", imu_data_selfcheck[1],
//                                  imu_data_selfcheck[2],
//                                  imu_data_selfcheck[3],
//                                  imu_data_selfcheck[4],
//                                  imu_data_selfcheck[5],
//                                  imu_data_selfcheck[6]);
//  Delay(10);
//  ICM20601_GetSelfCheckData(SelfcheckData);
//  printf("self register value:\r\n");
//  printf("%d %d %d %d %d %d\r\n", SelfcheckData[0],SelfcheckData[1],SelfcheckData[2],
//                                  SelfcheckData[3],SelfcheckData[4],SelfcheckData[5]);
//  Delay(10);
//  ICM20601_GetSelfCheckData(SelfcheckData);
//  printf("self register value:\r\n");
//  printf("%d %d %d %d %d %d\r\n", SelfcheckData[0],SelfcheckData[1],SelfcheckData[2],
//                                  SelfcheckData[3],SelfcheckData[4],SelfcheckData[5]);
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_GYRO_CONFIG,0x18);//Enable the self check
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_ACCEL_CONFIG,0x18);//Enable the self check
//  Delay(10); 
//  ICM20601_getData(imu_data_temp);
//  Delay(10);
//
//  printf("%f %f %f %f %f %f\r\n",   imu_data_selfcheck[1]-imu_data_temp[1],
//                                    imu_data_selfcheck[2]-imu_data_temp[2],
//                                    imu_data_selfcheck[3]-imu_data_temp[3],
//                                    imu_data_selfcheck[4]-imu_data_temp[4],
//                                    imu_data_selfcheck[5]-imu_data_temp[5],
//                                    imu_data_selfcheck[6]-imu_data_temp[6]);
//}
//  //set gyro X axis offset
//  GYRO_X_Offset = -50;
//  ICM20601_ReadRegs(ICM20601_XG_OFFSET_H,reg_offset,2);
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  GYRO_X_Offset += Byte16(int16_t, reg_offset[0], reg_offset[1]);
//  acc_offset[0] = (uint8_t)((GYRO_X_Offset >> 8)&0xff);
//  acc_offset[1] = (uint8_t)(GYRO_X_Offset & 0xff);
//  ICM20601_WriteReg(ICM20601_XG_OFFSET_H,acc_offset[0]);//acc z axis set the offset
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_XG_OFFSET_L,acc_offset[1]);//acc z axis set the offset 
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  //set gyro Y asix offset
//  GYRO_Y_Offset = 0;
//  ICM20601_ReadRegs(ICM20601_YG_OFFSET_H,reg_offset,2);
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  GYRO_Y_Offset += Byte16(int16_t, reg_offset[0], reg_offset[1]);
//  acc_offset[0] = (uint8_t)((GYRO_Y_Offset >> 8)&0xff);
//  acc_offset[1] = (uint8_t)(GYRO_Y_Offset & 0xff);
//  ICM20601_WriteReg(ICM20601_YG_OFFSET_H,acc_offset[0]);//acc z axis set the offset
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_YG_OFFSET_L,acc_offset[1]);//acc z axis set the offset 
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  //set gyro Z asix offset
//  GYRO_Z_Offset = 0;
//  ICM20601_ReadRegs(ICM20601_ZG_OFFSET_H,reg_offset,2);
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  GYRO_Z_Offset += Byte16(int16_t, reg_offset[0], reg_offset[1]);
//  acc_offset[0] = (uint8_t)((GYRO_Z_Offset >> 8)&0xff);
//  acc_offset[1] = (uint8_t)(GYRO_Z_Offset & 0xff);
//  ICM20601_WriteReg(ICM20601_ZG_OFFSET_H,acc_offset[0]);//acc z axis set the offset
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_ZG_OFFSET_L,acc_offset[1]);//acc z axis set the offset 
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  
//  //Set the acc X 
//  Acc_X_Offset  = 2;
//  ICM20601_ReadRegs(ICM20601_XA_OFFSET_H,reg_offset,2);
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  Acc_X_Offset += Byte16(int16_t, reg_offset[0], reg_offset[1])>>1;
//  acc_offset[0] = (uint8_t)((Acc_X_Offset >> 7)&0xff);
//  acc_offset[1] = (uint8_t)((Acc_X_Offset << 1)&0xff);
//  ICM20601_WriteReg(ICM20601_XA_OFFSET_H,acc_offset[0]);//acc z axis set the offset
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_XA_OFFSET_L,acc_offset[1]);//acc z axis set the offset 
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  //Set the acc Y
//  Acc_Y_Offset  = 2;
//  ICM20601_ReadRegs(ICM20601_YA_OFFSET_H,reg_offset,2);
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  Acc_Y_Offset += Byte16(int16_t, reg_offset[0], reg_offset[1])>>1;
//  acc_offset[0] = (uint8_t)((Acc_Y_Offset >> 7)&0xff);
//  acc_offset[1] = (uint8_t)((Acc_Y_Offset << 1)&0xff);
//  ICM20601_WriteReg(ICM20601_YA_OFFSET_H,acc_offset[0]);//acc z axis set the offset
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_YA_OFFSET_L,acc_offset[1]);//acc z axis set the offset 
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  //Set the acc Z
//  Acc_Z_Offset = 28;
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_ReadRegs(ICM20601_ZA_OFFSET_H,reg_offset,2);
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  Acc_Z_Offset += Byte16(int16_t, reg_offset[0], reg_offset[1])>>1;
//  acc_offset[0] = (uint8_t)((Acc_Z_Offset >> 7)&0xff);
//  acc_offset[1] = (uint8_t)((Acc_Z_Offset << 1)&0xff);
//  ICM20601_WriteReg(ICM20601_ZA_OFFSET_H,acc_offset[0]);//acc z axis set the offset
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_ZA_OFFSET_L,acc_offset[1]);//acc z axis set the offset 
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_PWR_MGMT_1,0x80);//Reset Device
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_PWR_MGMT_1,0x04);//Clock Source
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_INT_PIN_CFG,0x10);//Set INT_ANYRD_2CLEAR
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_INT_ENABLE,0x01);//Set RAW_RDY_EN
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_PWR_MGMT_2,0x00);//Enable Acc & Gyro
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_SMPLRT_DIV,0x00);//Sample Rate Divider 1kHz
//  vTaskDelay( pdMS_TO_TICKS( 10 ) );
//  ICM20601_WriteReg(ICM20601_GYRO_CONFIG,0x18);//set : ¡À2000dps  250hz
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_ACCEL_CONFIG,0x18);//set : ¡À16g 218.1hz
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_CONFIG,0x00);//set : 250hz for gyro
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_ACCEL_CONFIG2,0x00);//set : 7.8HZ rate: 1Khz
//  Delay(10);
//  ICM20601_WriteReg(ICM20601_USER_CTRL,0x10);//Disable I2C Slave module and put the serial interface in SPI mode only.
//  Delay(10);

