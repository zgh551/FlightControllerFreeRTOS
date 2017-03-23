#include "IMU.h"


#define IMU_STARTUP_TIME_MS   1000
#define IMU_ENABLE_MAG_HMC5983
//#define IMU_ENABLE_PRESSURE_MS5611

//#define IMU_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   ICM20602_DEG_PER_LSB_2000
//#define IMU_ACCEL_FS_CFG      MPU6500_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG     ICM20602_G_PER_LSB_16
#define IMU_1G_RAW            (int16_t)(1.0f / ICM20602_G_PER_LSB_16)

#define IMU_VARIANCE_MAN_TEST_TIMEOUT M2T(1000) // Timeout in ms
#define IMU_MAN_TEST_LEVEL_MAX        5.0f      // Max degrees off

#define MAG_GAUSS_PER_LSB     666.7f

#define IMU_STARTUP_TIME_MS   1000

#define GYRO_NBR_OF_AXES 3
#define GYRO_X_SIGN      (-1)
#define GYRO_Y_SIGN      (-1)
#define GYRO_Z_SIGN      (-1)
#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    pdMS_TO_TICKS(1*1000)

//#define IMU_TAKE_ACCEL_BIAS   
#define IMU_NBR_OF_BIAS_SAMPLES  1024

#define GYRO_VARIANCE_BASE        3000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

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
static int32_t    varianceSampleTime;
static Axis3i16   gyroMpu;
static Axis3i16   accelMpu;

static Axis3i16   accelLPF;
static Axis3i16   accelLPFAligned;
//static Axis3i16   mag;
static Axis3i32   accelStoredFilterValues;
static uint8_t    imuAccLpfAttFactor;
static bool       isHmc5983lPresent;
static bool       isMs5611Present;

//Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static bool isInit = false;
/**
 * MPU6500 selt test function. If the chip is moved to much during the self test
 * it will cause the test to fail.
 */
static void imuBiasInit(BiasObj* bias);
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                              Axis3i32* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);

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
  printf("Sensors SPI Init Finish\n");
  
  //Use the SPI Bus to connect the ICM20601
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
  if (HMC5983_TestConnection() == true)
  {
    isHmc5983lPresent = true;
    LedseqRun(LEDR,seq_linkup);
  }
  else
  {
    LedseqRun(LEDL, seq_linkup);
  }
  HMC5983_Configure();
  vTaskDelay(pdMS_TO_TICKS(100));
#endif

#ifdef IMU_ENABLE_PRESSURE_MS5611
  MS5611_Init();
  if (MS5611_SelfTest() == true)
  {
    isMs5611Present = true;
    LedseqRun(LEDR,seq_linkup);
  }
  else
  {
    LedseqRun(LEDL, seq_linkup);
  }
#endif
  imuBiasInit(&gyroBias);
#ifdef IMU_TAKE_ACCEL_BIAS
  imuBiasInit(&accelBias);
#endif
  
  varianceSampleTime = (int32_t)(-GYRO_MIN_BIAS_TIMEOUT_MS + 1);
  imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;
  
  cosPitch = 0.0f;//cos(configblockGetCalibPitch() * M_PI/180);
  sinPitch = 0.0f;//sin(configblockGetCalibPitch() * M_PI/180);
  cosRoll  = 0.0f;//cos(configblockGetCalibRoll() * M_PI/180);
  sinRoll  = 0.0f;//sin(configblockGetCalibRoll() * M_PI/180);

  isInit = true;
}

bool IMU_Test(void)
{
  return isInit;
}

void imu6Read(Axis3f *gyro,Axis3f *acc)
{
  ICM20601GetSixAxisData(&accelMpu.x,&accelMpu.y,&accelMpu.z,&gyroMpu.x,&gyroMpu.y,&gyroMpu.z);
  imuAddBiasValue(&gyroBias, &gyroMpu);
#ifdef IMU_TAKE_ACCEL_BIAS
  if (!accelBias.isBiasValueFound)
  {
    imuAddBiasValue(&accelBias, &accelMpu);
  }
#endif
  if (!gyroBias.isBiasValueFound)
  {
    //decide the quad whether in the static state
    imuFindBiasValue(&gyroBias);
    if (gyroBias.isBiasValueFound)
    {
      LedseqRun(LEDR, seq_calibrated);
    }
  }

#ifdef IMU_TAKE_ACCEL_BIAS
  if (gyroBias.isBiasValueFound &&
      !accelBias.isBiasValueFound)
  {
    Axis3i32 mean;

    imuCalculateBiasMean(&accelBias, &mean);
    accelBias.bias.x = mean.x;
    accelBias.bias.y = mean.y;
    accelBias.bias.z = mean.z - IMU_1G_RAW;
    accelBias.isBiasValueFound = true;
  }
#endif

  imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,
                    (int32_t)imuAccLpfAttFactor);
//
//  imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

  // Re-map outputs
  gyro->x =  (gyroMpu.y - gyroBias.bias.x) * IMU_DEG_PER_LSB_CFG;
  gyro->y =  (gyroMpu.x - gyroBias.bias.y) * IMU_DEG_PER_LSB_CFG;
  gyro->z =  -(gyroMpu.z - gyroBias.bias.z) * IMU_DEG_PER_LSB_CFG;
#ifdef IMU_TAKE_ACCEL_BIAS
  acc->x = (accelMpu.x - accelBias.bias.x) * IMU_G_PER_LSB_CFG;
  acc->y = (accelMpu.y - accelBias.bias.y) * IMU_G_PER_LSB_CFG;
  acc->z = (accelMpu.z - accelBias.bias.z) * IMU_G_PER_LSB_CFG;
#else
  acc->x =  (accelLPF.y) * IMU_G_PER_LSB_CFG;
  acc->y =  (accelLPF.x) * IMU_G_PER_LSB_CFG;
  acc->z =  -(accelLPF.z) * IMU_G_PER_LSB_CFG;
#endif
}

bool imu6IsCalibrated(void)
{
  bool status;

  status = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
  status &= accelBias.isBiasValueFound;
#endif

  return status;
}

void imu9Read(Axis3f *gyro,Axis3f *acc,Axis3f *mag)
{
  float magDateTemp[3];
  imu6Read(gyro,acc);
  if(isHmc5983lPresent){
    HMC5983_GetFloatData( magDateTemp );
    mag->x = magDateTemp[0];
    mag->y = magDateTemp[1];
    mag->z = magDateTemp[2];
  }else{
    mag->x = 0;
    mag->y = 0;
    mag->z = 0;
  }
}

bool imuHasBarometer(void)
{
  return isMs5611Present;
}

bool imuHasMangnetometer(void)
{
  return isHmc5983lPresent;
}

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

  isInit = true;
}

/**
 * Calculates the mean for the bias buffer.__attribute__((used))
 */
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool imuFindBiasValue(BiasObj* bias)
{
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    Axis3f variance;
    Axis3f mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = (int16_t)mean.x;
      bias->bias.y = (int16_t)mean.y;
      bias->bias.z = (int16_t)mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out)
{
  Axis3i16 rx;
  Axis3i16 ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = (int16_t)(in->y * cosRoll - in->z * sinRoll);
  rx.z = (int16_t)(in->y * sinRoll + in->z * cosRoll);

  // Rotate around y-axis
  ry.x = (int16_t)(rx.x * cosPitch - rx.z * sinPitch);
  ry.y = rx.y;
  ry.z = (int16_t)(-rx.x * sinPitch + rx.z * cosPitch);

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}