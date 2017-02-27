#include "IMU.h"

#define IMU_STARTUP_TIME_MS   1000
#define IMU_ENABLE_MAG_HMC5983
//#define IMU_ENABLE_PRESSURE_MS5611

#define IMU_NBR_OF_BIAS_SAMPLES  128
typedef struct
{
  Axis3i16   bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static BiasObj    gyroBias;
static BiasObj    accelBias;
//static int32_t    varianceSampleTime;
//
//static Axis3i16   gyroMpu;
//static Axis3i16   accelMpu;
//
//static Axis3i16   accelLPF;
//static Axis3i16   accelLPFAligned;
//static Axis3i16   mag;
//static Axis3i32   accelStoredFilterValues;
//static uint8_t    imuAccLpfAttFactor;
static bool       isHmc5983lPresent;
static bool       isMs5611Present;
//
//static bool isMpu6050TestPassed;
//static bool isHmc5883lTestPassed;
//static bool isMs5611TestPassed;
//
//// Pre-calculated values for accelerometer alignment
//static float cosPitch;
//static float sinPitch;
//static float cosRoll;
//static float sinRoll;

static bool isInit = false;

void IMU_Init(void)
{
  if(isInit)
    return;

 isHmc5983lPresent = false;
 isMs5611Present   = false;

  // Wait for sensors to startup,about one second startup
  while (xTaskGetTickCount() < pdMS_TO_TICKS(IMU_STARTUP_TIME_MS));
  //hardware Inintial the io port
  SPI1_Init(); 
  printf("SPI  Init Finish\n");
  
  //use the SPI bus to connect the ICM20602
  if (ICM20601_TestConnection() == SUCCESS)
  {		
    LedseqRun(LEDR,seq_linkup);
  }
  else
  {
    LedseqRun(LEDL,seq_linkup);
  }
  ICM20601_Init();
  
#ifdef IMU_ENABLE_MAG_HMC5983
  HMC5983_Configure();
  if (HMC5983_TestConnection() == true)
  {
    isHmc5983lPresent = true;
    LedseqRun(LEDR,seq_linkup);
  }
  else
  {
    LedseqRun(LEDL, seq_linkup);
  }
  vTaskDelay(pdMS_TO_TICKS(100));
#endif

#ifdef IMU_ENABLE_PRESSURE_MS5611
  if (ms5611Init(I2C2) == TRUE)
  {
    isMs5611Present = TRUE;
    LedseqRun(LEDR,seq_linkup);
  }
  else
  {
    LedseqRun(LEDL, seq_linkup);
  }
#endif

//  imuBiasInit(&gyroBias);
//  imuBiasInit(&accelBias);
//  varianceSampleTime = (int32_t)(-GYRO_MIN_BIAS_TIMEOUT_MS + 1);
//  imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

//  cosPitch = cos(configblockGetCalibPitch() * M_PI/180);
//  sinPitch = sin(configblockGetCalibPitch() * M_PI/180);
//  cosRoll = cos(configblockGetCalibRoll() * M_PI/180);
//  sinRoll = sin(configblockGetCalibRoll() * M_PI/180);
  /**/
//  cosPitch = cos(0.0 * M_PI/180);
//  sinPitch = sin(0.0 * M_PI/180);
//  cosRoll  = cos(0.0 * M_PI/180);
//  sinRoll  = sin(0.0 * M_PI/180);
	
  isInit = true;
}
