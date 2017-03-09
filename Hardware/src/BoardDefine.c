#include "BoardDefine.h"

GPIO_TypeDef * GPIO_PORT[LEDn] = {LED1_GPIO_PORT,LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn]  = {LED1_PIN,LED2_PIN};
const uint16_t GPIO_CLK[LEDn]  = {LED1_GPIO_CLK,LED2_GPIO_CLK};

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void STM_LED_Init(Led_TypeDef led)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(GPIO_CLK[led], ENABLE);

  /* Configure PG6 and PG8 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_PORT[led], &GPIO_InitStructure);
  GPIO_PORT[led]->BSRRL = GPIO_PIN[led];
}

void STM_LED_Toggle(Led_TypeDef led)
{
  GPIO_PORT[led]->ODR ^= GPIO_PIN[led];
}

void STM_LED_Set(Led_TypeDef led,uint8_t state)
{
  if(state)
  {
    /* Open LED */
    GPIO_ResetBits(GPIO_PORT[led], GPIO_PIN[led]);
  }
  else
  {
    /* Close LED */
    GPIO_SetBits(GPIO_PORT[led], GPIO_PIN[led]);
  }
}

static void SENSOR_POWER_EN_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(SENSOR_POWER_EN_GPIO_CLK, ENABLE);

  /* Configure PG6 and PG8 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = SENSOR_POWER_EN_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(SENSOR_POWER_EN_GPIO_PORT, &GPIO_InitStructure);
  SENSOR_POWER_EN_GPIO_PORT->BSRRH = SENSOR_POWER_EN_PIN;
}

void SENSOR_POWER_ENABLE(void)
{
   SENSOR_POWER_EN_GPIO_PORT->BSRRL = SENSOR_POWER_EN_PIN;
}

void SENSOR_POWER_DISABLE(void)
{
   SENSOR_POWER_EN_GPIO_PORT->BSRRH = SENSOR_POWER_EN_PIN;
}

static void VBUS_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(VBUS_GPIO_CLK, ENABLE);

  /* Configure PG6 and PG8 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = VBUS_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(VBUS_GPIO_PORT, &GPIO_InitStructure);
  VBUS_GPIO_PORT->BSRRH = VBUS_PIN;
}

static void SPI1_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //SPI CLK ENABLE 
  MPU9250_SPI_INIT(MPU9250_SPI_CLK,ENABLE);
  // ENABLE THE GPIO CLK
  RCC_AHB1PeriphClockCmd(MPU9250_SPI_SCL_CLK | MPU9250_SPI_MISO_CLK |
                         MPU9250_SPI_MOSI_CLK| MPU9250_SPI_nCS_CLK  | 
                         HMC5983_SPI_nCS_CLK , ENABLE);
  
  //SPI PIN CONFIGURE
  //Connect the SPI Pin to AF
  GPIO_PinAFConfig(MPU9250_SPI_SCL_PORT,MPU9250_SPI_SCL_SOURCE,MPU9250_SPI_SCL_AF);
  GPIO_PinAFConfig(MPU9250_SPI_MISO_PORT,MPU9250_SPI_MISO_SOURCE,MPU9250_SPI_MISO_AF);
  GPIO_PinAFConfig(MPU9250_SPI_MOSI_PORT,MPU9250_SPI_MOSI_SOURCE,MPU9250_SPI_MOSI_AF);
  
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  
  //SPI CLK
  GPIO_InitStructure.GPIO_Pin = MPU9250_SPI_SCL_PIN;
  GPIO_Init(MPU9250_SPI_SCL_PORT, &GPIO_InitStructure);
  
  //SPI MISO
  GPIO_InitStructure.GPIO_Pin = MPU9250_SPI_MISO_PIN;
  GPIO_Init(MPU9250_SPI_MISO_PORT, &GPIO_InitStructure);
  
  //SPI MOSI
  GPIO_InitStructure.GPIO_Pin = MPU9250_SPI_MOSI_PIN;
  GPIO_Init(MPU9250_SPI_MOSI_PORT, &GPIO_InitStructure);
  
  //CONFIGUR THE nCS PIN
  GPIO_InitStructure.GPIO_Pin   = MPU9250_SPI_nCS_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(MPU9250_SPI_nCS_PORT, &GPIO_InitStructure);
  
  //HMC5983 nCS Pin
  GPIO_InitStructure.GPIO_Pin   = HMC5983_SPI_nCS_PIN;
  GPIO_Init(HMC5983_SPI_nCS_PORT, &GPIO_InitStructure);
}

void SPI1_Init(void)
{
  SPI_InitTypeDef SPI_InitStructure;
  SPI1_GPIO_Init();
  MP9250_CS_HIGH;
  HMC5983_CS_HIGH;
  
  SPI_InitStructure.SPI_Direction   = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode        = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize    = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL        = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA        = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS         = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//<1MHZ
  SPI_InitStructure.SPI_FirstBit    = SPI_FirstBit_MSB;
  
  SPI_Init(MPU9250_SPI,&SPI_InitStructure);
  SPI_Cmd(MPU9250_SPI, ENABLE);
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t SPI1_RW(uint8_t byte)
{
  /*!< Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(MPU9250_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(MPU9250_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(MPU9250_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(MPU9250_SPI);
}




static void STM_Printf_COMInit(USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable UART clock */
  RCC_APB1PeriphClockCmd(PRINTF_COM1_CLK , ENABLE);

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(PRINTF_COM1_TX_GPIO_CLK | PRINTF_COM1_RX_GPIO_CLK, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(PRINTF_COM1_TX_GPIO_PORT, PRINTF_COM1_TX_SOURCE, PRINTF_COM1_TX_AF);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(PRINTF_COM1_RX_GPIO_PORT, PRINTF_COM1_RX_SOURCE, PRINTF_COM1_RX_AF);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin   = PRINTF_COM1_TX_PIN; 
  GPIO_Init(PRINTF_COM1_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin   = PRINTF_COM1_RX_PIN;
  GPIO_Init(PRINTF_COM1_RX_GPIO_PORT, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(PRINTF_COM1, USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(PRINTF_COM1, ENABLE);
}

static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  
  /* USARTx configured as follows:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate    = 115200;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity      = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  STM_Printf_COMInit(&USART_InitStructure);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(PRINTF_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(PRINTF_COM1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

void SendByte(uint8_t dat)
{
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(PRINTF_COM1, USART_FLAG_TC) == RESET);
  USART_SendData(PRINTF_COM1, (uint8_t) dat);
}

/********************Time Configure ****************************/
void Tim1Configure(void)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStruct;
  NVIC_InitTypeDef          NVIC_InitStruct;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  
  TIM_TimeBaseInitStruct.TIM_ClockDivision   = TIM_CKD_DIV2;
  TIM_TimeBaseInitStruct.TIM_Prescaler       = 180;
  TIM_TimeBaseInitStruct.TIM_Period          = 4999;
  TIM_TimeBaseInitStruct.TIM_CounterMode     = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel                       =   TIM1_UP_TIM10_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority     =   0x01;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority            =   0x01;
  NVIC_InitStruct.NVIC_IRQChannelCmd                    =   ENABLE;
  NVIC_Init(&NVIC_InitStruct);
          
  TIM_ITConfig(TIM1, TIM_IT_Update , ENABLE);
  TIM_Cmd(TIM1 , DISABLE);
}

/**************PPM Configure *****************/
/**
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
//static void PPM_GPIO_Config(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//
//
//  /* GPIOA, GPIOB and GPIOE Clocks enable */
//  RCC_AHB1PeriphClockCmd( PPM1_GPIO_CLK | PPM2_GPIO_CLK | PPM3_GPIO_CLK |PPM4_GPIO_CLK , ENABLE);
//  
//  /* GPIOD Configuration: Channel 1,2,3,4 as alternate function push-pull */
//  GPIO_InitStructure.GPIO_Pin = PPM1_PIN | PPM2_PIN | PPM3_PIN | PPM4_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(PPM1_GPIO_PORT, &GPIO_InitStructure);
//  
//  GPIO_PinAFConfig(PPM1_GPIO_PORT, PPM1_SOURCE, PPM1_AF);
//  GPIO_PinAFConfig(PPM2_GPIO_PORT, PPM2_SOURCE, PPM2_AF);
//  GPIO_PinAFConfig(PPM3_GPIO_PORT, PPM3_SOURCE, PPM3_AF);
//  GPIO_PinAFConfig(PPM4_GPIO_PORT, PPM4_SOURCE, PPM4_AF);  
//}

//static void PPM_TIM_Config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  TIM_OCInitTypeDef  TIM_OCInitStructure;
//  uint16_t TimerPeriod = 0;
//  uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
//
//  /* TIM4 clock enable */
//  RCC_APB1PeriphClockCmd(PPM_TIM_CLK , ENABLE);
//  
//  /* Compute the value to be set in ARR register to generate signal frequency at 50hz */
//  TimerPeriod = (1000000 / 50 ) - 1;//19999
//  /* Compute CCR1 value to generate a duty cycle at 5% for channel 1 and 1N */
//  Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 100);
//  /* Compute CCR2 value to generate a duty cycle at 5%  for channel 2 and 2N */
//  Channel2Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 100);
//  /* Compute CCR3 value to generate a duty cycle at 5%  for channel 3 and 3N */
//  Channel3Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 100);
//  /* Compute CCR4 value to generate a duty cycle at 5%  for channel 4 */
//  Channel4Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod- 1)) / 100);
//  
//  /* Time Base configuration */
//  TIM_TimeBaseStructure.TIM_Prescaler = 90;//1MHZ
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//
//  TIM_TimeBaseInit(PPM_TIM, &TIM_TimeBaseStructure);
//  
//  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
//  TIM_OCInitStructure.TIM_OCMode        = TIM_OCMode_PWM2;
//  TIM_OCInitStructure.TIM_OutputState   = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_OutputNState  = TIM_OutputNState_Enable;
//  TIM_OCInitStructure.TIM_Pulse         = Channel1Pulse;
//  TIM_OCInitStructure.TIM_OCPolarity    = TIM_OCPolarity_Low;
//  TIM_OCInitStructure.TIM_OCNPolarity   = TIM_OCNPolarity_High;
//  TIM_OCInitStructure.TIM_OCIdleState   = TIM_OCIdleState_Set;
//  TIM_OCInitStructure.TIM_OCNIdleState  = TIM_OCIdleState_Reset;
//
//  TIM_OC1Init(PPM_TIM, &TIM_OCInitStructure);
//
//  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
//  TIM_OC2Init(PPM_TIM, &TIM_OCInitStructure);
//
//  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
//  TIM_OC3Init(PPM_TIM, &TIM_OCInitStructure);
//
//  TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
//  TIM_OC4Init(PPM_TIM, &TIM_OCInitStructure);
//
//  /* TIM1 counter enable */
//  TIM_Cmd(PPM_TIM, ENABLE);
//
//  /* TIM1 Main Output Enable */
//  TIM_CtrlPWMOutputs(PPM_TIM, ENABLE);
//}

/**
  * @brief  DeInitializes the SDIO interface.
  * @param  None
  * @retval None
  */
void SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*!< Disable SDIO Clock */
  SDIO_ClockCmd(DISABLE);
  
  /*!< Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  /*!< DeInitializes the SDIO peripheral */
  SDIO_DeInit();
  
  /* Disable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);

  GPIO_PinAFConfig(SD_D0_GPIO_PORT , SD_D0_SOURCE , SD_D0_AF);
  GPIO_PinAFConfig(SD_D1_GPIO_PORT , SD_D1_SOURCE , SD_D1_AF);
  GPIO_PinAFConfig(SD_D2_GPIO_PORT , SD_D2_SOURCE , SD_D2_AF);
  GPIO_PinAFConfig(SD_D3_GPIO_PORT , SD_D3_SOURCE , SD_D3_AF);
  GPIO_PinAFConfig(SD_CK_GPIO_PORT , SD_CK_SOURCE , SD_CK_AF);
  GPIO_PinAFConfig(SD_CMD_GPIO_PORT, SD_CMD_SOURCE, SD_CMD_AF);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = SD_D0_PIN | SD_D1_PIN | SD_D2_PIN | SD_D3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = SD_CMD_PIN;
  GPIO_Init(SD_CMD_GPIO_PORT, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = SD_CK_PIN;
  GPIO_Init(SD_CK_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | SD_DETECT_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(SD_D0_GPIO_PORT , SD_D0_SOURCE , SD_D0_AF);
  GPIO_PinAFConfig(SD_D1_GPIO_PORT , SD_D1_SOURCE , SD_D1_AF);
  GPIO_PinAFConfig(SD_D2_GPIO_PORT , SD_D2_SOURCE , SD_D2_AF);
  GPIO_PinAFConfig(SD_D3_GPIO_PORT , SD_D3_SOURCE , SD_D3_AF);
  GPIO_PinAFConfig(SD_CK_GPIO_PORT , SD_CK_SOURCE , SD_CK_AF);
  GPIO_PinAFConfig(SD_CMD_GPIO_PORT, SD_CMD_SOURCE, SD_CMD_AF);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = SD_D0_PIN | SD_D1_PIN | SD_D2_PIN | SD_D3_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = SD_CMD_PIN;
  GPIO_Init(SD_CMD_GPIO_PORT, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = SD_CK_PIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SD_CK_GPIO_PORT, &GPIO_InitStructure);
  
  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);
  
  /* Enable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

  /* Enable the DMA2 Clock */
  RCC_AHB1PeriphClockCmd(SD_SDIO_DMA_CLK, ENABLE);
}

/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval None
  */
void SDIO_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);  
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3  or Stream6 Config */
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3  or Stream6 enable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
    
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3 or Stream6 Config */
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3 or Stream6 enable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

/*************Hardware Configure*******************/
void HardwarePeripheralInit(void)
{
  USART_Config();
  printf("UART Init Finish\n");
  
//  STM_LED_Init(LED1);
//  STM_LED_Init(LED2);
  SENSOR_POWER_EN_Init();
  SENSOR_POWER_ENABLE();
  VBUS_Init();
  printf("LED Init Finish\n");
//  
//  SPI1_Init();
//  printf("SPI  Init Finish\n");
//  
//  PPM_GPIO_Config();
//  PPM_TIM_Config();
//  printf("PPM  Init Finish\n");
//  
//  Tim1Configure();
//  printf("TIM1  Init Finish\n");
}

/* Public functions */
//void systemLaunch(void)
//{
//	xTaskCreate(systemTask,( signed portCHAR* )"SYSTEM",
//              configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
//}
