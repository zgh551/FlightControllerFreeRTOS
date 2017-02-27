/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FAT_ENABLE  0
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if FAT_ENABLE 
FIL *fil;
FATFS *fs;
FRESULT res;
char my_char[20];
char buf[512];
#endif
/* Private function prototypes -----------------------------------------------*/
//portTASK_FUNCTION(LedTest,pvParameters)
//{
//  	for( ;; )
//	{
//      STM_LED_Toggle(LEDR);
//      vTaskDelay( pdMS_TO_TICKS( 100 ) );
//    }
//}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Ensure all priority bits are assigned as preemption priority bits. */
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
//  xTaskCreate( LedTest, "LED", configMINIMAL_STACK_SIZE , NULL, 1, NULL );
  systemLaunch();
  
  /* Start the scheduler. */
  vTaskStartScheduler();
  
  /* Infinite loop */   
  for( ;; );

}
//  /* SysTick end of count event each 10ms */
//  RCC_GetClocksFreq(&RCC_Clocks);
//  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
//  Delay(5);
//  HardwarePeripheralInit();
//  ICM20601_Init();
//  Delay(10); 
//  HMC5983_Configure();
//  Delay(10);
//  nRF24L01_Configure();
//  Motor_Range_Set();
//  Motor_Start_Test();

//  TIM_Cmd(TIM1 , ENABLE);
//#if FAT_ENABLE  
//  /* NVIC Configuration */
//  SDIO_NVIC_Configuration();
//  mem_init(SRAMIN);
//  res = f_mkfs ("",FM_FAT,0,buf,sizeof buf);
//  fs  = (FATFS*)mymalloc(SRAMIN,sizeof(FATFS));
//  fil = (FIL*)mymalloc(SRAMIN,sizeof(FIL));
//  res = f_mount (fs,"0:",1);
//  res = f_open (fil,"0:zgh.txt", FA_CREATE_ALWAYS | FA_WRITE);	
//  if(res == FR_OK)
//  {
//    res = f_lseek(fil,f_size(fil));
//    res = f_close (fil);
//  }
//#endif
//  while (1)
//  {
//    #if FAT_ENABLE
//    res = f_open (fil,"0:zgh.txt", FA_OPEN_APPEND | FA_WRITE);	
//    if(res == FR_OK)
//    {
//      res = f_lseek(fil,f_size(fil));
//      sprintf(my_char,"%f %f %f",m_PITCH,m_ROLL,m_YAW);
//      res = f_printf(fil, "%s\n", my_char);            /* "1234" */
//      res = f_close (fil);
//    }
//    #endif
//    Delay(100);
//    STM_LED_Toggle(LED2);  
//
//  }
    //    nrfRxPayload(rx_test,1);//read the RX payload 
//    status = nrfReadReg(REG_FIFO_STATUS);
//    status = nrfReadReg(REG_STATUS);
//    dataLen = nrfRxLength();
//    nrfRxPayload(RX_Buffer1,dataLen);//read the RX payload 
//    Delay(10);
//    PPM1_SetRatio(0);

void vApplicationTickHook( void )
{
//	#if ( mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY == 0 )
//	{
//		/* Just to verify that the interrupt nesting behaves as expected,
//		increment ulFPUInterruptNesting on entry, and decrement it on exit. */
//		ulFPUInterruptNesting++;
//
//		/* Fill the FPU registers with 0. */
//		vRegTestClearFlopRegistersToParameterValue( 0UL );
//
//		/* Trigger a timer 2 interrupt, which will fill the registers with a
//		different value and itself trigger a timer 3 interrupt.  Note that the
//		timers are not actually used.  The timer 2 and 3 interrupt vectors are
//		just used for convenience. */
//		NVIC_SetPendingIRQ( TIM2_IRQn );
//
//		/* Ensure that, after returning from the nested interrupts, all the FPU
//		registers contain the value to which they were set by the tick hook
//		function. */
//		configASSERT( ulRegTestCheckFlopRegistersContainParameterValue( 0UL ) );
//
//		ulFPUInterruptNesting--;
//	}
//	#endif
}
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
