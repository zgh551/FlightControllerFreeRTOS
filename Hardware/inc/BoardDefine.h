#ifndef __BOARDDEFINE_H__
#define __BOARDDEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "BoardDefineType.h"
  
///****LED Define ****/

#define  ON     1
#define  OFF    0

#define LEDn 2

#define LED1_PIN        GPIO_Pin_7
#define LED1_GPIO_PORT  GPIOB
#define LED1_GPIO_CLK   RCC_AHB1Periph_GPIOB
  
#define LED2_PIN        GPIO_Pin_6
#define LED2_GPIO_PORT  GPIOB
#define LED2_GPIO_CLK   RCC_AHB1Periph_GPIOB


/***********Power IO Pin****************/
#define SENSOR_POWER_EN_PIN        GPIO_Pin_14
#define SENSOR_POWER_EN_GPIO_PORT  GPIOE
#define SENSOR_POWER_EN_GPIO_CLK   RCC_AHB1Periph_GPIOE
     
#define VBUS_PIN        GPIO_Pin_3
#define VBUS_GPIO_PORT  GPIOE
#define VBUS_GPIO_CLK   RCC_AHB1Periph_GPIOE

/****SPI Define *****/
#define SPIn                1
#define MPU9250_SPI             SPI1
#define MPU9250_SPI_CLK         RCC_APB2Periph_SPI1
#define MPU9250_SPI_INIT        RCC_APB2PeriphClockCmd
//clk
#define MPU9250_SPI_SCL_PIN     GPIO_Pin_5
#define MPU9250_SPI_SCL_PORT    GPIOA
#define MPU9250_SPI_SCL_CLK     RCC_AHB1Periph_GPIOA
#define MPU9250_SPI_SCL_SOURCE  GPIO_PinSource5
#define MPU9250_SPI_SCL_AF      GPIO_AF_SPI1
//MOSI
#define MPU9250_SPI_MISO_PIN     GPIO_Pin_6
#define MPU9250_SPI_MISO_PORT    GPIOA
#define MPU9250_SPI_MISO_CLK     RCC_AHB1Periph_GPIOA
#define MPU9250_SPI_MISO_SOURCE  GPIO_PinSource6
#define MPU9250_SPI_MISO_AF      GPIO_AF_SPI1
//MISO
#define MPU9250_SPI_MOSI_PIN     GPIO_Pin_7
#define MPU9250_SPI_MOSI_PORT    GPIOA
#define MPU9250_SPI_MOSI_CLK     RCC_AHB1Periph_GPIOA
#define MPU9250_SPI_MOSI_SOURCE  GPIO_PinSource7
#define MPU9250_SPI_MOSI_AF      GPIO_AF_SPI1

/* nCS GPIO define*/
#define MPU9250_SPI_nCS_PIN     GPIO_Pin_2
#define MPU9250_SPI_nCS_PORT    GPIOC
#define MPU9250_SPI_nCS_CLK     RCC_AHB1Periph_GPIOC

#define MP9250_CS_LOW    GPIO_ResetBits(MPU9250_SPI_nCS_PORT,MPU9250_SPI_nCS_PIN)
#define MP9250_CS_HIGH   GPIO_SetBits(MPU9250_SPI_nCS_PORT,MPU9250_SPI_nCS_PIN)
#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))


/**********************HMC5983******************************/
#define HMC5983_SPI_nCS_PIN     GPIO_Pin_12
#define HMC5983_SPI_nCS_PORT    GPIOE
#define HMC5983_SPI_nCS_CLK     RCC_AHB1Periph_GPIOE

#define HMC5983_CS_LOW    GPIO_ResetBits(HMC5983_SPI_nCS_PORT,HMC5983_SPI_nCS_PIN)
#define HMC5983_CS_HIGH   GPIO_SetBits(HMC5983_SPI_nCS_PORT,HMC5983_SPI_nCS_PIN)

/**********************nRF24L01******************************/
//SPI
#define nRF24L01_SPI                        SPI2
#define nRF24L01_SPI_CLK                    RCC_APB1Periph_SPI2

#define nRF24L01_SPI_SCK_PIN                GPIO_Pin_13                 /* PB.13 */
#define nRF24L01_SPI_SCK_GPIO_PORT          GPIOB                       /* GPIOB */
#define nRF24L01_SPI_SCK_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define nRF24L01_SPI_SCK_SOURCE             GPIO_PinSource13
#define nRF24L01_SPI_SCK_AF                 GPIO_AF_SPI2

#define nRF24L01_SPI_MISO_PIN               GPIO_Pin_14                 /* PB.14 */
#define nRF24L01_SPI_MISO_GPIO_PORT         GPIOB                       /* GPIOB */
#define nRF24L01_SPI_MISO_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define nRF24L01_SPI_MISO_SOURCE            GPIO_PinSource14
#define nRF24L01_SPI_MISO_AF                GPIO_AF_SPI2
  
#define nRF24L01_SPI_MOSI_PIN               GPIO_Pin_15                 /* PB.15 */
#define nRF24L01_SPI_MOSI_GPIO_PORT         GPIOB                       /* GPIOB */
#define nRF24L01_SPI_MOSI_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define nRF24L01_SPI_MOSI_SOURCE            GPIO_PinSource15
#define nRF24L01_SPI_MOSI_AF                GPIO_AF_SPI2
  
#define nRF24L01_SPI_PORT                   GPIOB                       //All SPI Pin in GPIOB 

#define nRF24L01_SPI_CSN_PIN                GPIO_Pin_9                  /* PD.09 */
#define nRF24L01_SPI_CSN_GPIO_PORT          GPIOD                       /* GPIOD */
#define nRF24L01_SPI_CSN_GPIO_CLK           RCC_AHB1Periph_GPIOD

#define nRF24L01_SPI_CE_PIN                 GPIO_Pin_8                  /* PD.08 */
#define nRF24L01_SPI_CE_GPIO_PORT           GPIOD                       /* GPIOD */
#define nRF24L01_SPI_CE_GPIO_CLK            RCC_AHB1Periph_GPIOD

#define nRF24L01_SPI_IRQ_PIN                GPIO_Pin_12                 /* PB.12 */
#define nRF24L01_SPI_IRQ_GPIO_PORT          GPIOB                       /* GPIOB */
#define nRF24L01_SPI_IRQ_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define nRF24L01_SPI_IRQ_SRC_PORT	    EXTI_PortSourceGPIOB
#define nRF24L01_SPI_IRQ_SRC_PIN	    EXTI_PinSource12
#define nRF24L01_SPI_IRQ_LINE               EXTI_Line12
/***************USART Communication*****************/
#define COMn                             1

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define PRINTF_COM1                        USART2
#define PRINTF_COM1_CLK                    RCC_APB1Periph_USART2

#define PRINTF_COM1_TX_PIN                 GPIO_Pin_2
#define PRINTF_COM1_TX_GPIO_PORT           GPIOA
#define PRINTF_COM1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define PRINTF_COM1_TX_SOURCE              GPIO_PinSource2
#define PRINTF_COM1_TX_AF                  GPIO_AF_USART1

#define PRINTF_COM1_RX_PIN                 GPIO_Pin_3
#define PRINTF_COM1_RX_GPIO_PORT           GPIOA
#define PRINTF_COM1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define PRINTF_COM1_RX_SOURCE              GPIO_PinSource3
#define PRINTF_COM1_RX_AF                  GPIO_AF_USART1
  
#define PRINTF_COM1_IRQn                   USART1_IRQn
/**************PPM Configure *****************/
#define PPM_TIM                  TIM4   
#define PPM_TIM_CLK              RCC_APB1Periph_TIM4

#define PPM1_PIN                 GPIO_Pin_12
#define PPM1_GPIO_PORT           GPIOD
#define PPM1_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define PPM1_SOURCE              GPIO_PinSource12
#define PPM1_AF                  GPIO_AF_TIM4

#define PPM2_PIN                 GPIO_Pin_13
#define PPM2_GPIO_PORT           GPIOD
#define PPM2_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define PPM2_SOURCE              GPIO_PinSource13
#define PPM2_AF                  GPIO_AF_TIM4

#define PPM3_PIN                 GPIO_Pin_14
#define PPM3_GPIO_PORT           GPIOD
#define PPM3_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define PPM3_SOURCE              GPIO_PinSource14
#define PPM3_AF                  GPIO_AF_TIM4

#define PPM4_PIN                 GPIO_Pin_15
#define PPM4_GPIO_PORT           GPIOD
#define PPM4_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define PPM4_SOURCE              GPIO_PinSource15
#define PPM4_AF                  GPIO_AF_TIM4

/**
  * @brief  SD FLASH SDIO Interface
  */
#define SD_D0_PIN                       GPIO_Pin_8                 /* PC.8 */
#define SD_D0_GPIO_PORT                 GPIOC                      /* GPIOC */
#define SD_D0_GPIO_CLK                  RCC_AHB1Periph_GPIOC
#define SD_D0_SOURCE                    GPIO_PinSource8
#define SD_D0_AF                        GPIO_AF_SDIO

#define SD_D1_PIN                       GPIO_Pin_9                 /* PC.9 */
#define SD_D1_GPIO_PORT                 GPIOC                      /* GPIOC */
#define SD_D1_GPIO_CLK                  RCC_AHB1Periph_GPIOC
#define SD_D1_SOURCE                    GPIO_PinSource9
#define SD_D1_AF                        GPIO_AF_SDIO

#define SD_D2_PIN                       GPIO_Pin_10                 /* PC.10 */
#define SD_D2_GPIO_PORT                 GPIOC                      /* GPIOC */
#define SD_D2_GPIO_CLK                  RCC_AHB1Periph_GPIOC
#define SD_D2_SOURCE                    GPIO_PinSource10
#define SD_D2_AF                        GPIO_AF_SDIO

#define SD_D3_PIN                       GPIO_Pin_11                 /* PC.11 */
#define SD_D3_GPIO_PORT                 GPIOC                      /* GPIOC */
#define SD_D3_GPIO_CLK                  RCC_AHB1Periph_GPIOC
#define SD_D3_SOURCE                    GPIO_PinSource11
#define SD_D3_AF                        GPIO_AF_SDIO

#define SD_CK_PIN                       GPIO_Pin_12                 /* PC.12 */
#define SD_CK_GPIO_PORT                 GPIOC                      /* GPIOC */
#define SD_CK_GPIO_CLK                  RCC_AHB1Periph_GPIOC
#define SD_CK_SOURCE                    GPIO_PinSource12
#define SD_CK_AF                        GPIO_AF_SDIO

#define SD_CMD_PIN                      GPIO_Pin_2                 /* PD.2 */
#define SD_CMD_GPIO_PORT                GPIOD                      /* GPIOD */
#define SD_CMD_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define SD_CMD_SOURCE                   GPIO_PinSource2
#define SD_CMD_AF                       GPIO_AF_SDIO

#define SD_DETECT_PIN                   GPIO_Pin_8                 /* PH.8 */
#define SD_DETECT_GPIO_PORT             GPIOA                      /* GPIOA */
#define SD_DETECT_GPIO_CLK              RCC_AHB1Periph_GPIOA
   
#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40012C80)
/** 
  * @brief  SDIO Intialization Frequency (400KHz max)
  */
#define SDIO_INIT_CLK_DIV                ((uint8_t)0x76)
/** 
  * @brief  SDIO Data Transfer Frequency (25MHz max) //9MHz
  */
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x03) 

#define SD_SDIO_DMA                   DMA2
#define SD_SDIO_DMA_CLK               RCC_AHB1Periph_DMA2
 
#define SD_SDIO_DMA_STREAM3	          3
//#define SD_SDIO_DMA_STREAM6           6

#ifdef SD_SDIO_DMA_STREAM3
 #define SD_SDIO_DMA_STREAM            DMA2_Stream3
 #define SD_SDIO_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF         DMA_FLAG_FEIF3
 #define SD_SDIO_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF3
 #define SD_SDIO_DMA_FLAG_TEIF         DMA_FLAG_TEIF3
 #define SD_SDIO_DMA_FLAG_HTIF         DMA_FLAG_HTIF3
 #define SD_SDIO_DMA_FLAG_TCIF         DMA_FLAG_TCIF3 
 #define SD_SDIO_DMA_IRQn              DMA2_Stream3_IRQn
 #define SD_SDIO_DMA_IRQHANDLER        DMA2_Stream3_IRQHandler 
#elif defined SD_SDIO_DMA_STREAM6
 #define SD_SDIO_DMA_STREAM            DMA2_Stream6
 #define SD_SDIO_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF         DMA_FLAG_FEIF6
 #define SD_SDIO_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF6
 #define SD_SDIO_DMA_FLAG_TEIF         DMA_FLAG_TEIF6
 #define SD_SDIO_DMA_FLAG_HTIF         DMA_FLAG_HTIF6
 #define SD_SDIO_DMA_FLAG_TCIF         DMA_FLAG_TCIF6 
 #define SD_SDIO_DMA_IRQn              DMA2_Stream6_IRQn
 #define SD_SDIO_DMA_IRQHANDLER        DMA2_Stream6_IRQHandler
#endif /* SD_SDIO_DMA_STREAM3 */

/*Led Set Function*/
void STM_LED_Init(Led_TypeDef led);
void STM_LED_Toggle(Led_TypeDef led);
void STM_LED_Set(Led_TypeDef led,uint8_t state);

/* Serial communication Function */
void SendByte(uint8_t dat);

/* SPI function */
void SPI1_Init(void);
uint8_t SPI1_RW(uint8_t byte);

/* SD Card Function Init */
void SD_LowLevel_DeInit(void);
void SD_LowLevel_Init(void);
void SDIO_NVIC_Configuration(void);
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);

void HardwarePeripheralInit(void);

#ifdef __cplusplus
}
#endif
  
#endif
