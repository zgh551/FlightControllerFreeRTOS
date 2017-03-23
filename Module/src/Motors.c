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
#include "motors.h"

static uint16_t motorsBLConvBitsTo16(uint16_t bits);
static uint16_t motorsBLConv16ToBits(uint16_t bits);
static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);

//#include "motors_def.c"
// Connector M1, PA1, TIM2_CH2
static const MotorPerifDef CONN_M1 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// Connector M2, PB11, TIM2_CH4
static const MotorPerifDef CONN_M2 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Connector M3, PA15, TIM2_CH1
static const MotorPerifDef CONN_M3 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// Connector M4, PB9, TIM4_CH4
static const MotorPerifDef CONN_M4 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM4,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Connector M1, PA1, TIM2_CH2, Brushless config
static const MotorPerifDef CONN_M1_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = PPM1_GPIO_CLK,
    .gpioPort      = PPM1_GPIO_PORT,
    .gpioPin       = PPM1_PIN,
    .gpioPinSource = PPM1_SOURCE,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = PPM1_AF,
    .timPerif      = PPM_TIM_CLK,
    .tim           = PPM_TIM,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// Connector M2, PB11, TIM2_CH4, Brushless config
static const MotorPerifDef CONN_M2_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = PPM2_GPIO_CLK,
    .gpioPort      = PPM2_GPIO_PORT,
    .gpioPin       = PPM2_PIN,
    .gpioPinSource = PPM2_SOURCE,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = PPM2_AF,
    .timPerif      = PPM_TIM_CLK,
    .tim           = PPM_TIM,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// Connector M3, PA15, TIM2_CH1, Brushless config
static const MotorPerifDef CONN_M3_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = PPM3_GPIO_CLK,
    .gpioPort      = PPM3_GPIO_PORT,
    .gpioPin       = PPM3_PIN,
    .gpioPinSource = PPM3_SOURCE,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = PPM3_AF,
    .timPerif      = PPM_TIM_CLK,
    .tim           = PPM_TIM,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

// Connector M4, PB9, TIM4_CH4, Brushless config
static const MotorPerifDef CONN_M4_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = PPM4_GPIO_CLK,
    .gpioPort      = PPM4_GPIO_PORT,
    .gpioPin       = PPM4_PIN,
    .gpioPinSource = PPM4_SOURCE,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = PPM4_AF,
    .timPerif      = PPM_TIM_CLK,
    .tim           = PPM_TIM,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Deck TX2, PA2, TIM2_CH3
static const MotorPerifDef DECK_TX2_TIM2 =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_2,
    .gpioPinSource = GPIO_PinSource2,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

// Deck RX2, PA3, TIM2_CH4
static const MotorPerifDef DECK_RX2_TIM2 =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_3,
    .gpioPinSource = GPIO_PinSource3,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Deck IO2, PB5, TIM3_CH2
static const MotorPerifDef DECK_IO2 =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_5,
    .gpioPinSource = GPIO_PinSource5,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// Deck IO3, PB4, TIM3_CH1
static const MotorPerifDef DECK_IO3 =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_4,
    .gpioPinSource = GPIO_PinSource4,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};



/**
 * Default brushed mapping to M1-M4 connectors.
 */
const MotorPerifDef* motorMapDefaultBrushed[NBR_OF_MOTORS] =
{
  &CONN_M1,
  &CONN_M2,
  &CONN_M3,
  &CONN_M4
};

/**
 * Brushless motors mapped as on the Big-Quad deck
 * M1 -> TX2
 * M2 -> IO3
 * M3 -> IO2
 * M4 -> RX2
 */
const MotorPerifDef* motorMapBigQuadDeck[NBR_OF_MOTORS] =
{
  &DECK_TX2_TIM2,
  &DECK_IO3,
  &DECK_IO2,
  &DECK_RX2_TIM2
};

/**
 * Brushless motors mapped to the standard motor connectors with pull-ups (~1K) to VBAT soldered.
 */
const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS] =
{
  &CONN_M1_BL,
  &CONN_M2_BL,
  &CONN_M3_BL,
  &CONN_M4_BL
};
///////////////////////////////
uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsDeInit(const MotorPerifDef** motorMapSelect);

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

static const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

static bool isInit = false;

/* Private functions */

static uint16_t motorsBLConvBitsTo16(uint16_t bits)
{
  return (0xFFFF * (bits - MOTORS_BL_PWM_CNT_FOR_1MS) / MOTORS_BL_PWM_CNT_FOR_1MS);
}

static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_1MS + ((bits * MOTORS_BL_PWM_CNT_FOR_1MS) / 0xFFFF));
}

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
  return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
  return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if (isInit)
  {
    motorsDeInit(motorMap);
  }

  motorMap = motorMapSelect;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = (GPIOOType_TypeDef)motorMap[i]->gpioOType;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);

    MOTORS_TIM_DBG_CFG(motorMap[i]->timDbgStop, ENABLE);
    //Enable the timer PWM outputs
    TIM_CtrlPWMOutputs(motorMap[i]->tim, ENABLE);
  }

  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }

  isInit = true;
}

void motorsDeInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  GPIO_InitTypeDef GPIO_InitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);
    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);

    //Deinit timer
    TIM_DeInit(motorMap[i]->tim);
  }
}

// Ithrust is thrust mapped for 65536 <==> 60g
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  uint16_t ratio;

  ratio = ithrust;

  if (motorMap[id]->drvType == BRUSHLESS)
  {
    motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));
  }
  else
  {
    motorMap[id]->setCompare(motorMap[id]->tim, motorsConv16ToBits(ratio));
  }
}

int motorsGetRatio(uint32_t id)
{
  int ratio;

//  ASSERT(id < NBR_OF_MOTORS);
  if (motorMap[id]->drvType == BRUSHLESS)
  {
    ratio = motorsBLConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }
  else
  {
    ratio = motorsConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }

  return ratio;
}

bool motorsTest(void)
{
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    if (motorMap[i]->drvType == BRUSHED)
    {
#ifdef ACTIVATE_STARTUP_SOUND
      motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
      vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME_MS));
      motorsBeep(MOTORS[i], false, 0, 0);
      vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_DELAY_TIME_MS));
#else
      motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
      vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME_MS));
      motorsSetRatio(MOTORS[i], 0);
      vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_DELAY_TIME_MS));
#endif
    }
  }

  return isInit;
}
/* Set PWM frequency for motor controller
 * This function will set all motors into a "beep"-mode,
 * each of the motor will turned on with a given ratio and frequency.
 * The higher the ratio the higher the given power to the motors.
 * ATTENTION: To much ratio can push your crazyflie into the air and hurt you!
 * Example:
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//  ASSERT(id < NBR_OF_MOTORS);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  if (enable)
  {
    TIM_TimeBaseStructure.TIM_Prescaler = (5 - 1);
    TIM_TimeBaseStructure.TIM_Period = (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency);
  }
  else
  {
    TIM_TimeBaseStructure.TIM_Period = motorMap[id]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[id]->timPrescaler;
  }

  // Timer configuration
  TIM_TimeBaseInit(motorMap[id]->tim, &TIM_TimeBaseStructure);
  motorMap[id]->setCompare(motorMap[id]->tim, ratio);
}


// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
  motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  vTaskDelay(pdMS_TO_TICKS(duration_msec));
  motorsBeep(MOTOR_M1, false, frequency, 0);
  motorsBeep(MOTOR_M2, false, frequency, 0);
  motorsBeep(MOTOR_M3, false, frequency, 0);
  motorsBeep(MOTOR_M4, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
  int i = 0;
  uint16_t note;      // Note in hz
  uint16_t duration;  // Duration in ms

  do
  {
    note = notes[i++];
    duration = notes[i++];
    motorsPlayTone(note, duration);
  } while (duration != 0);
}