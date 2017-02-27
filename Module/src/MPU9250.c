#include "MPU9250.h"

#define _USE_MAG_AK8963

#define MAG_READ_DELAY 256

//static SPI_HandleTypeDef MPU_HandleStruct;
#ifdef _USE_MAG_AK8963
int16_t AK8963_ASA[3] = {0};
#endif

//static void MPU9250_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  //SPI CLK ENABLE 
//  MPU9250_SPI_INIT(MPU9250_SPI_CLK,ENABLE);
//  // ENABLE THE GPIO CLK
//  RCC_AHB1PeriphClockCmd(MPU9250_SPI_SCL_CLK | MPU9250_SPI_MISO_CLK |
//                         MPU9250_SPI_MOSI_CLK| MPU9250_SPI_nCS_CLK  , ENABLE);
//  
//  //SPI PIN CONFIGURE
//  //Connect the SPI Pin to AF
//  GPIO_PinAFConfig(MPU9250_SPI_SCL_PORT,MPU9250_SPI_SCL_SOURCE,MPU9250_SPI_SCL_AF);
//  GPIO_PinAFConfig(MPU9250_SPI_MISO_PORT,MPU9250_SPI_MISO_SOURCE,MPU9250_SPI_MISO_AF);
//  GPIO_PinAFConfig(MPU9250_SPI_MOSI_PORT,MPU9250_SPI_MOSI_SOURCE,MPU9250_SPI_MOSI_AF);
//  
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//  
//  //SPI CLK
//  GPIO_InitStructure.GPIO_Pin = MPU9250_SPI_SCL_PIN;
//  GPIO_Init(MPU9250_SPI_SCL_PORT, &GPIO_InitStructure);
//  
//  //SPI MISO
//  GPIO_InitStructure.GPIO_Pin = MPU9250_SPI_MISO_PIN;
//  GPIO_Init(MPU9250_SPI_MISO_PORT, &GPIO_InitStructure);
//  
//  //SPI MOSI
//  GPIO_InitStructure.GPIO_Pin = MPU9250_SPI_MOSI_PIN;
//  GPIO_Init(MPU9250_SPI_MOSI_PORT, &GPIO_InitStructure);
//  
//  //CONFIGUR THE nCS PIN
//  GPIO_InitStructure.GPIO_Pin   = MPU9250_SPI_nCS_PIN;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//  GPIO_Init(MPU9250_SPI_nCS_PORT, &GPIO_InitStructure);
//}
//
//void MPU9250_SPI_Init(void)
//{
//  SPI_InitTypeDef SPI_InitStructure;
//  MPU9250_GPIO_Init();
//  MP9250_CS_HIGH;
//  
//  SPI_InitStructure.SPI_Direction   = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_Mode        = SPI_Mode_Master;
//  SPI_InitStructure.SPI_DataSize    = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_CPOL        = SPI_CPOL_High;
//  SPI_InitStructure.SPI_CPHA        = SPI_CPHA_2Edge;
//  SPI_InitStructure.SPI_NSS         = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//<1MHZ
//  SPI_InitStructure.SPI_FirstBit    = SPI_FirstBit_MSB;
//  
//  SPI_Init(MPU9250_SPI,&SPI_InitStructure);
//  SPI_Cmd(MPU9250_SPI, ENABLE);
//}
//
///**
//  * @brief  Sends a byte through the SPI interface and return the byte received
//  *         from the SPI bus.
//  * @param  byte: byte to send.
//  * @retval The value of the received byte.
//  */
//uint8_t MPU9250_RW(uint8_t byte)
//{
//  /*!< Loop while DR register in not empty */
//  while (SPI_I2S_GetFlagStatus(MPU9250_SPI, SPI_I2S_FLAG_TXE) == RESET);
//
//  /*!< Send byte through the SPI1 peripheral */
//  SPI_I2S_SendData(MPU9250_SPI, byte);
//
//  /*!< Wait to receive a byte */
//  while (SPI_I2S_GetFlagStatus(MPU9250_SPI, SPI_I2S_FLAG_RXNE) == RESET);
//
//  /*!< Return the byte read from the SPI bus */
//  return SPI_I2S_ReceiveData(MPU9250_SPI);
//}

//write the mpu9250 reg
void MPU9250_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  MP9250_CS_LOW;
  SPI1_RW(writeAddr);
  SPI1_RW(writeData);
  MP9250_CS_HIGH;
}

//write serials data into the reg
void MPU9250_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  MP9250_CS_LOW;
  SPI1_RW(writeAddr);
  for(uint8_t i = 0; i < lens; i++)
    SPI1_RW(writeData[i]);
  MP9250_CS_HIGH;
}

//read one the reg value
uint8_t MPU9250_ReadReg( uint8_t readAddr )
{
  uint8_t readData = 0;

  MP9250_CS_LOW;
  SPI1_RW(0x80 | readAddr);
  readData = SPI1_RW(0x00);
  MP9250_CS_HIGH;

  return readData;
}

void MPU9250_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens)
{
  MP9250_CS_LOW;
  SPI1_RW(0x80 | readAddr);
  for(uint8_t i = 0; i < lens; i++)
    readData[i] = SPI1_RW(0x00);
  MP9250_CS_HIGH;
}


#ifdef _USE_MAG_AK8963
void MPU9250_Mag_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  uint8_t  status = 0;
  uint32_t timeout = MAG_READ_DELAY;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
  Delay(1);

  do {
    status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
    Delay(1);
  } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
}
#endif

#ifdef _USE_MAG_AK8963
void MPU9250_Mag_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  uint8_t  status = 0;
  uint32_t timeout = MAG_READ_DELAY;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  Delay(1);
  for(uint8_t i = 0; i < lens; i++) {
    MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr + i);
    Delay(1);
    MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData[i]);
    Delay(1);
    MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
    Delay(1);

    do {
      status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
  }
}
#endif

#ifdef _USE_MAG_AK8963
uint8_t MPU9250_Mag_ReadReg( uint8_t readAddr )
{
  uint8_t status = 0;
  uint8_t readData = 0;
  uint32_t timeout = MAG_READ_DELAY;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
  Delay(1);

  do {
    status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
    Delay(1);
  } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

  readData = MPU9250_ReadReg(MPU6500_I2C_SLV4_DI);

  return readData;
}
#endif

#ifdef _USE_MAG_AK8963
void MPU9250_Mag_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  uint8_t status = 0;
  uint32_t timeout = MAG_READ_DELAY;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
  Delay(1);
  for(uint8_t i = 0; i< lens; i++) {
    MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr + i);
    Delay(1);
    MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
    Delay(1);

    do {
      status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

    readData[i] = MPU9250_ReadReg(MPU6500_I2C_SLV4_DI);
    Delay(1);
  }
}
#endif


#define MPU6500_InitRegNum 11
uint8_t MPU9250_Init( MPU_InitTypeDef *MPUx )
{
  uint8_t status = ERROR;
#ifdef _USE_MAG_AK8963
  uint8_t tmpRead[3] = {0};
#endif
  uint8_t MPU6500_InitData[MPU6500_InitRegNum][2] = {
    {0x80, MPU6500_PWR_MGMT_1},     // [0]  Reset Device
    {0x04, MPU6500_PWR_MGMT_1},     // [1]  Clock Source
    {0x10, MPU6500_INT_PIN_CFG},    // [2]  Set INT_ANYRD_2CLEAR
    {0x01, MPU6500_INT_ENABLE},     // [3]  Set RAW_RDY_EN
    {0x00, MPU6500_PWR_MGMT_2},     // [4]  Enable Acc & Gyro
    {0x00, MPU6500_SMPLRT_DIV},     // [5]  Sample Rate Divider
    {0x18, MPU6500_GYRO_CONFIG},    // [6]  default : +-2000dps
    {0x08, MPU6500_ACCEL_CONFIG},   // [7]  default : +-4G
    {0x07, MPU6500_CONFIG},         // [8]  default : LPS_41Hz
    {0x03, MPU6500_ACCEL_CONFIG_2}, // [9]  default : LPS_41Hz
    {0x30, MPU6500_USER_CTRL},      // [10] Set I2C_MST_EN, I2C_IF_DIS
  };

  MPU6500_InitData[6][0] = MPUx->MPU_Gyr_FullScale;       // MPU6500_GYRO_CONFIG
  MPU6500_InitData[8][0] = MPUx->MPU_Gyr_LowPassFilter;   // MPU6500_CONFIG
  MPU6500_InitData[7][0] = MPUx->MPU_Acc_FullScale;       // MPU6500_ACCEL_CONFIG
  MPU6500_InitData[9][0] = MPUx->MPU_Acc_LowPassFilter;   // MPU6500_ACCEL_CONFIG_2

  for(uint8_t i = 0; i < MPU6500_InitRegNum; i++) {
    MPU9250_WriteReg(MPU6500_InitData[i][1], MPU6500_InitData[i][0]);
    Delay(1);
  }
  Delay(10);
  status = MPU9250_Check();
  if(status != SUCCESS)
    return ERROR;

  Delay(10);

#ifdef _USE_MAG_AK8963
  MPU9250_Mag_WriteReg(AK8963_CNTL2, 0x01);       // Reset Device
  Delay(1);
  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
  Delay(1);
  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x1F);       // Fuse ROM access mode
  Delay(1);
  MPU9250_Mag_ReadRegs(AK8963_ASAX, tmpRead, 3);  // Read sensitivity adjustment values
  Delay(1);
  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
  Delay(1);

  AK8963_ASA[0] = (int16_t)(tmpRead[0]) + 128;
  AK8963_ASA[1] = (int16_t)(tmpRead[1]) + 128;
  AK8963_ASA[2] = (int16_t)(tmpRead[2]) + 128;

  MPU9250_WriteReg(MPU6500_I2C_MST_CTRL, 0x5D);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);
  Delay(1);

  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x16);       // Continuous measurement mode 2
  Delay(1);

  MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, 0x09);
  Delay(1);
  MPU9250_WriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x81);
  Delay(100);
#endif

  return SUCCESS;
}

uint8_t MPU9250_Check( void )
{
  uint8_t deviceID = ERROR;

  deviceID = MPU9250_ReadReg(MPU6500_WHO_AM_I);
  if(deviceID != MPU6500_Device_ID)
  {
    printf("MPU9250 ERR,ID=%d\n",deviceID);
    return ERROR;
  }
#ifdef _USE_MAG_AK8963
  deviceID = MPU9250_Mag_ReadReg(AK8963_WIA);
  if(deviceID != AK8963_Device_ID)
  {
    printf("AK8963 ERR,ID=%d\n",deviceID);
    return ERROR;
  }
#endif

  return SUCCESS;
}


void MPU9250_Config( void )
{
  MPU_InitTypeDef MPU_InitStruct;
  
//  MPU9250_SPI_Init();
//  SPI1_Init();
//  Delay(10);
  
  printf("MPU9250 Init ... ");

  MPU_InitStruct.MPU_Gyr_FullScale     = MPU_GyrFS_2000dps;
  MPU_InitStruct.MPU_Gyr_LowPassFilter = MPU_GyrLPS_41Hz;
  MPU_InitStruct.MPU_Acc_FullScale     = MPU_AccFS_4g;
  MPU_InitStruct.MPU_Acc_LowPassFilter = MPU_AccLPS_41Hz;
  if(MPU9250_Init(&MPU_InitStruct) != SUCCESS) {
    printf("ERROR\r\n");
    while(1) {
      STM_LED_Toggle(LEDR);
      Delay(100);
    }
  }
  else {
    printf("SUCCESS\r\n");
  }
  Delay(100);
}

void MPU9250_getData( int16_t *dataIMU )
{
  uint8_t tmpRead[22] = {0};

#ifdef _USE_MAG_AK8963
  MPU9250_ReadRegs(MPU6500_ACCEL_XOUT_H, tmpRead, 22);
#else
  MPU9250_ReadRegs(MPU6500_ACCEL_XOUT_H, tmpRead, 14);
#endif

  dataIMU[0] = (Byte16(int16_t, tmpRead[6],  tmpRead[7]));    // Temp
  dataIMU[1] = (Byte16(int16_t, tmpRead[0],  tmpRead[1]));    // Acc.X
  dataIMU[2] = (Byte16(int16_t, tmpRead[2],  tmpRead[3]));    // Acc.Y
  dataIMU[3] = (Byte16(int16_t, tmpRead[4],  tmpRead[5]));    // Acc.Z
  dataIMU[4] = (Byte16(int16_t, tmpRead[8],  tmpRead[9]));    // Gyr.X
  dataIMU[5] = (Byte16(int16_t, tmpRead[10], tmpRead[11]));   // Gyr.Y
  dataIMU[6] = (Byte16(int16_t, tmpRead[12], tmpRead[13]));   // Gyr.Z
#ifdef _USE_MAG_AK8963
  if(!(tmpRead[14] & AK8963_STATUS_DRDY) || (tmpRead[14] & AK8963_STATUS_DOR) || (tmpRead[21] & AK8963_STATUS_HOFL))
    return;

  dataIMU[7] = (Byte16(int16_t, tmpRead[16], tmpRead[15]));   // Mag.X
  dataIMU[8] = (Byte16(int16_t, tmpRead[18], tmpRead[17]));   // Mag.Y
  dataIMU[9] = (Byte16(int16_t, tmpRead[20], tmpRead[19]));   // Mag.Z

//  dataIMU[7] = ((long)dataIMU[7] * AK8963_ASA[0]) >> 8;
//  dataIMU[8] = ((long)dataIMU[8] * AK8963_ASA[1]) >> 8;
//  dataIMU[9] = ((long)dataIMU[9] * AK8963_ASA[2]) >> 8;
#endif  
}