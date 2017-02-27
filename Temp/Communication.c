#include "Communication.h"
#include <stdio.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


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

void USART_Config(void)
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
