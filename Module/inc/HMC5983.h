#ifndef _HMC5983_H_
#define _HMC5983_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define CRA       0x00
#define CRB       0x01
#define MR        0x02
#define DOMR_X    0x03
#define DOLR_X    0x04  
#define DOMR_Z    0x05
#define DOLR_Z    0x06
#define DOMR_Y    0x07
#define DOLR_Y    0x08
#define SR        0x09
#define IRA       0x0A
#define IRB       0x0B
#define IRC       0x0C
#define TOMR      0x31
#define TOLR      0x32
 
typedef enum {
  TMP_DISABLE    = 0x00,
  TMP_ENABLE     = 0x01
} Tmp_Cmps_TypeDef;

typedef enum {
  One      = 0x00,
  Two      = 0x01,
  Four     = 0x02,
  Eight    = 0x03
} Sample_Averaged_TypeDef;

typedef enum {
  DOR_0_75HZ    = 0x00,
  DOR_1_5HZ     = 0x01,
  DOR_3HZ       = 0x02,
  DOR_7_5HZ     = 0x03,
  DOR_15HZ      = 0x04,
  DOR_30HZ      = 0x05,
  DOR_75HZ      = 0x06,
  DOR_220HZ     = 0x07
} Data_Out_Rate_TypeDef;

typedef enum {
  Normal        = 0x00,
  Positive      = 0x01,
  Negative      = 0x02,
  TempSenOnly   = 0x03
} Measure_Mode_TypeDef;



typedef enum {
  Mag_Gain_0_88G    = 0x00,
  Mag_Gain_1_3G     = 0x01,
  Mag_Gain_1_9G     = 0x02,
  Mag_Gain_2_5G     = 0x03,
  Mag_Gain_4_0G     = 0x04,
  Mag_Gain_4_7G     = 0x05,
  Mag_Gain_5_6G     = 0x06,
  Mag_Gain_8_1G     = 0x07
} Mag_Gain_TypeDef;

typedef enum {
  ContinuousMeasurementMode     = 0x00,
  SingleMeasurementMode         = 0x01
} Operating_Mode_TypeDef;

typedef struct {
  Tmp_Cmps_TypeDef          MAG_TC;             //bit 7
  Sample_Averaged_TypeDef   MAG_SampAverage;    //bit 6-5
  Data_Out_Rate_TypeDef     MAG_Data_Rate;      //bit 4-2
  Measure_Mode_TypeDef      MAG_Measure_Mode;   //bit 1-0
  Mag_Gain_TypeDef          MAG_Gain;
  Operating_Mode_TypeDef    MAG_Operate_Mode;
} MAG_InitTypeDef;

bool HMC5983_TestConnection( void );

void HMC5983_Configure(void);
void HMC5983_GetData( int16_t *dataIMU );
void HMC5983_GetFloatData( float *dataIMU );
void HMC5983GetTreeAxisData( int16_t *mx,int16_t *my,int16_t *mz );
void MagCorrect( float *dataIMU , float *CorrectDataIMU );
void MagCorrectOffset( float *dataIMU , float *CorrectDataIMU );
void HMC5983_SendData(float *dat);
#ifdef __cplusplus
}
#endif
#endif
