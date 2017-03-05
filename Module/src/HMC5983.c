#include "HMC5983.h"


//float B[6] = {1.0119836225537   ,  0.00804477285115623 ,0.00718567847934233,
//              0.980743846974053 , -0.043053987729707   ,1.00957192431739   };

//Ðý×ªÆ½Ì¨
#define MagOffsetX   -0.047030661217639
#define MagOffsetY   -0.098843728296896
#define MagOffsetZ   0

float B[6] = {0.122158190704876   , 0.0328369577517874 ,0.237010139644319,
              0.00882679899048001 , 0.0637099477107354 ,0.459844779708063};

int16_t Xoffset,Yoffset,Zoffset;
uint16_t Tag_Cnt=0;

union b2f
{
  float     dat_f;
  uint8_t   dat_b[4];
}type_d;

void HMC5983_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  HMC5983_CS_LOW;
  
  SPI1_RW(writeAddr);
  SPI1_RW(writeData);
  
  HMC5983_CS_HIGH;
}

void HMC5983_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  HMC5983_CS_LOW;
  
  SPI1_RW(writeAddr|0x40);
  for(uint8_t i = 0; i < lens; i++)
    SPI1_RW(writeData[i]);
  
  HMC5983_CS_HIGH;
}

uint8_t HMC5983_ReadReg( uint8_t readAddr )
{
  uint8_t readData = 0;
  HMC5983_CS_LOW;
  SPI1_RW(0x80 | readAddr);
  readData = SPI1_RW(0x00);  
  HMC5983_CS_HIGH;

  return readData;
}

void HMC5983_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens)
{
  HMC5983_CS_LOW;
  SPI1_RW(0xC0 | readAddr);
  for(uint8_t i = 0; i < lens; i++)
    readData[i] = SPI1_RW(0x00);
  HMC5983_CS_HIGH;
}

bool HMC5983_TestConnection( void )
{
  uint8_t deviceID[3];

  HMC5983_ReadRegs(IRA,deviceID,3);
  if(deviceID[0] != 'H' || deviceID[1] != '4' || deviceID[2] != '3')
  {
    printf("HMC5983 ID=%d %d %d\n",deviceID[0],deviceID[1],deviceID[2]);
    return ERROR;
  }
  return SUCCESS;
}
/**********************************************************************
 *@brief 
 *@param 
 *	this parameter can be one of following parameters
 *	@arg 
 *@retval return whether receive  is successful or not
 *	@arg 
 *	@arg 
***********************************************************************/
void HMC5983_Init(MAG_InitTypeDef *MAGx)
{  
  uint8_t HMC5983_InitData[3];
  
  HMC5983_InitData[0] = (uint8_t)((MAGx->MAG_TC << 7) | (MAGx->MAG_SampAverage << 5) |(MAGx->MAG_Data_Rate<< 2)| MAGx->MAG_Measure_Mode );
  HMC5983_InitData[1] = (uint8_t)(MAGx->MAG_Gain << 5);
  HMC5983_InitData[2] = MAGx->MAG_Operate_Mode;
  
  HMC5983_WriteRegs(CRA,HMC5983_InitData,3);
}

void HMC5983_Configure(void)
{
  MAG_InitTypeDef MAG_Init_Struct;

  MAG_Init_Struct.MAG_TC            = TMP_ENABLE;
  MAG_Init_Struct.MAG_SampAverage   = One;
  MAG_Init_Struct.MAG_Data_Rate     = DOR_220HZ;
  MAG_Init_Struct.MAG_Measure_Mode  = Normal;
  MAG_Init_Struct.MAG_Gain          = Mag_Gain_2_5G;
  MAG_Init_Struct.MAG_Operate_Mode  = ContinuousMeasurementMode;
  HMC5983_Init(&MAG_Init_Struct);
  printf("HMC5983 Init ...");
}

void HMC5983_GetInt16Data( int16_t *dataIMU )
{
  uint8_t temp_data[6];
  if(HMC5983_ReadReg(SR)&0x01)
  {
    HMC5983_ReadRegs(DOMR_X,temp_data,6);
    dataIMU[0] = Byte16(int16_t, temp_data[0],  temp_data[1]); // MAG.X
    dataIMU[1] = Byte16(int16_t, temp_data[4],  temp_data[5]); // MAG.Y
    dataIMU[2] = Byte16(int16_t, temp_data[2],  temp_data[3]); // MAG.Z
  }
}

void HMC5983_GetFloatData( float *dataIMU )
{
  uint8_t temp_data[6];
  uint8_t status;
  status = HMC5983_ReadReg(SR);
  if(status & 0x01)
  {
    HMC5983_ReadRegs(DOMR_X,temp_data,6);
    dataIMU[0] = Byte16(int16_t, temp_data[0],  temp_data[1])*0.00152 ;  //MAG.X
    dataIMU[1] = Byte16(int16_t, temp_data[4],  temp_data[5])*0.00152 ;  //MAG.Y
    dataIMU[2] = Byte16(int16_t, temp_data[2],  temp_data[3])*0.00152 ;  //MAG.Z
  }
}

void HMC5983GetTreeAxisData( int16_t *mx,int16_t *my,int16_t *mz )
{
  uint8_t temp_data[6];
  uint8_t status;
  status = HMC5983_ReadReg(SR);
  if(status & 0x01)
  {
    HMC5983_ReadRegs(DOMR_X,temp_data,6);
    *mx = Byte16(int16_t, temp_data[0],  temp_data[1]);  //MAG.X*0.00152 
    *my = Byte16(int16_t, temp_data[4],  temp_data[5]);  //MAG.Y*0.00152 
    *mz = Byte16(int16_t, temp_data[2],  temp_data[3]);  //MAG.Z*0.00152 
  }
}

void MagCorrect( float *dataIMU , float *CorrectDataIMU )
{
  float temp[3];
  
  temp[0] = dataIMU[0]- MagOffsetX;
  temp[1] = dataIMU[1]- MagOffsetY;
  temp[2] = dataIMU[2]- MagOffsetZ;
  
  CorrectDataIMU[0] = B[0]*temp[0] + B[1]*temp[1] + B[2]*temp[2];
  CorrectDataIMU[1] = B[1]*temp[0] + B[3]*temp[1] + B[4]*temp[2];
  CorrectDataIMU[2] = B[2]*temp[0] + B[4]*temp[1] + B[5]*temp[2];
}

void MagCorrectOffset( float *dataIMU , float *CorrectDataIMU )
{ 
  CorrectDataIMU[0] = dataIMU[0] - MagOffsetX;
  CorrectDataIMU[1] = dataIMU[1] - MagOffsetY;
  CorrectDataIMU[2] = dataIMU[2] - MagOffsetZ;
}

void HMC5983_SendData(float *dat)
{
  uint8_t CRC_Sum,i,j;
  CRC_Sum = 0;
  Tag_Cnt++;
  SendByte(0xaa);
  SendByte(0x55);
  for(j=0;j<3;j++)
  {
    type_d.dat_f = dat[j];
    for(i=0;i<4;i++)
    {
      CRC_Sum += type_d.dat_b[i];
      SendByte(type_d.dat_b[i]);
    }
  }
  CRC_Sum += (uint8_t)((Tag_Cnt>>8)&0xff);
  SendByte((Tag_Cnt>>8)&0xff);
  CRC_Sum += (uint8_t)(Tag_Cnt&0xff);
  SendByte(Tag_Cnt&0xff);
  SendByte(CRC_Sum);
}
//  uint8_t i=0,j=0;
//  int16_t POS_Dat[3],NEG_Dat[3];
//  int16_t POS_Sum[3],NEG_Sum[3];
//  for(j=0;j<3;j++)
//  {
//    POS_Sum[j] = 0;
//    NEG_Sum[j] = 0;
//  }
//  for(i=0;i<10;i++)
//  {
//    Delay(10);
//    HMC5983GetInt16Data(POS_Dat);
//    for(j=0;j<3;j++)
//    {
//      POS_Sum[j] += POS_Dat[j];
//    }
//  }
//  for(j=0;j<3;j++)
//  {
//    POS_Dat[j] = POS_Sum[j] / 10;
//  }
//  printf("X_POS = %d\r\nY_POS = %d\r\nZ_POS = %d\r\n",POS_Dat[0],POS_Dat[1],POS_Dat[2]);
//  
//  MAG_Init_Struct.MAG_Measure_Mode = Negative;
//  HMC5983_Init(&MAG_Init_Struct);
//
//  for(i=0;i<10;i++)
//  {
//    Delay(10);
//    HMC5983GetInt16Data(NEG_Dat);
//    for(j=0;j<3;j++)
//    {
//      NEG_Sum[j] += NEG_Dat[j];
//    }
//  }
//  for(j=0;j<3;j++)
//  {
//    NEG_Dat[j] = NEG_Sum[j] / 10;
//  }
//  printf("X_NEG = %d\r\nY_NEG = %d\r\nZ_NEG = %d\r\n",NEG_Dat[0],NEG_Dat[1],NEG_Dat[2]);
//  Xoffset = (POS_Dat[0] + NEG_Dat[0])/2;
//  Yoffset = (POS_Dat[1] + NEG_Dat[1])/2;
//  Zoffset = (POS_Dat[2] + NEG_Dat[2])/2;
//  printf("X_Offset = %d\r\nY_Offset = %d\r\nZ_Offset = %d\r\n",Xoffset,Yoffset,Zoffset);
//  
//  MAG_Init_Struct.MAG_Measure_Mode = Normal;
//  if(HMC5983_Init(&MAG_Init_Struct) != SUCCESS){
//    printf("ERROR\r\n");
//    while(1) {
//      STM_LED_Toggle(LEDR);
//      Delay(100);
//    }
//  }
//  else {
//    printf("SUCCESS\r\n");
//  }
//  Delay(100);