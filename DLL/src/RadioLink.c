#include "RadioLink.h"

#define RADIO_CONNECTED_TIMEOUT   pdMS_TO_TICKS(2000)

static bool isInit;

/* Synchronisation */
static xSemaphoreHandle dataRdy;
/* Data queue */
static xQueueHandle txQueue;
static xQueueHandle rxQueue;

static uint32_t LastPacketTick;

//Union used to efficiently handle the packets (Private type)
typedef union
{
  CRTPPacket crtp;
  struct {
    uint8_t size;
    uint8_t data[32];
  } __packed raw;
} RadioPacket;

RadioPacket pk;

static struct {
 bool enabled;
}state;


enum {
  RX_1=0,
  TX_1,
  RX_2,
  TX_2
};

#define RX_PLOAD_WIDTH  32  	
#define TX_PLOAD_WIDTH  32

uint8_t myrxtest[32]={0};
unsigned char dataLen;
uint8_t reg_state;

/**************************************************
 *@brief set the radio enable
 *@param enable:input true or false 
 *@retval None 
**/
int  radioSetEnable(bool enable)
{
  nrfSetEnable(enable);
  state.enabled = enable;
  return 0;
}

/*****************************************************************************
 *@brief	send the packet data to txQueue
 *@param[in]*pk: the packet pointer with information
 *@retval	if state is not enable then return ENETDOWN,otherwise return zero.
 *****************************************************************************/
int radioSendPacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return 1;
  xQueueSend(txQueue, pk, portMAX_DELAY);//111portMAX_DELAY

  return 0;
}

/*********************************************************************************************
 *@brief	  receive the packet data from the txQueue and put the packet into the pk pointer 
 *@param[out] *pk: the packet pointer with information
 *@retval	  if state is not enable then return ENETDOWN,otherwise return zero.
 *********************************************************************************************/
int radioReceivePacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return 1;
  xQueueReceive(rxQueue, pk, portMAX_DELAY);//portMAX_DELAY

  return 0;
}

/*****************************************************************************
 *@brief  judge the radio whether is connected or not
 *@param  None
 *@retval if radio is connected then return true,otherwise return false
 *****************************************************************************/
static bool radioIsConnected(void)
{
  if ((xTaskGetTickCount() - LastPacketTick) > RADIO_CONNECTED_TIMEOUT)
    return false;

  return true;
}

/********************************************************************************
 *@brief	reset the txQueue and clear the TX FIFO register
 *@param	None
 *@retval	None
 ********************************************************************************/
void radioReset(void)
{
  xQueueReset(txQueue);
  nrfFlushTx();
}

static struct crtpLinkOperations radioOp =
{
  radioSetEnable,                     //int (*setEnable)(bool enable);
  radioSendPacket,                    //int (*sendPacket)(CRTPPacket *pk);
  radioReceivePacket,                 //int (*receivePacket)(CRTPPacket *pk);
  radioIsConnected,                   //bool (*isConnected)(void);
  radioReset                          //int (*reset)(void);
};

/*******************************************************************************************
 *@brief  Radio task handles the CRTP packet transfers as well as the radio link
 *        specific communications (eg. Scann and ID ports, communication error handling
 *        and so much other cool things that I don't have time for it ...)
 *@param  NUll
 *@retval None 
 *******************************************************************************************/
static void RadioTask(void * arg)
{
//  LastPacketTick = xTaskGetTickCount();
  //Packets handling loop
//  xSemaphoreTake( dataRdy, 0 );
//  //clear the interruptions flags
//  nrfWriteReg(REG_STATUS, 0x70);
  while(1)
  {
//    vTaskDelayUntil((portTickType *)&LastPacketTick,pdMS_TO_TICKS(30));//20ms 
//    EXTI_GenerateSWInterrupt(nRF24L01_SPI_IRQ_LINE);
    xSemaphoreTake(dataRdy, portMAX_DELAY);//interrupt way
//    LastPacketTick = xTaskGetTickCount();
//    reg_state = nrfReadReg(REG_RPD);//
//    reg_state = nrfReadReg(REG_STATUS); //REG_FIFO_STATUS nrfReadReg(REG_FIFO_STATUS)&0x02
//    if(reg_state&0x40)
//    {	
      nrfSetEnable(false);
      //Fetch all the data (Loop until the RX Fifo is NOT empty)
      while( !(nrfReadReg(REG_FIFO_STATUS)&0x01) )
      {
        dataLen = nrfRxLength();
        if (dataLen>32)          //If a packet has a wrong size it is dropped
            nrfFlushRx();		   //clear the RX FIFO register
        else                     //Else, it is processed
        {
          //Fetch the data
          pk.raw.size = dataLen-1;//-->crtp one is flag other is data
          nrfRxPayload((uint8_t*)pk.raw.data,dataLen);//read the RX payload 
          xQueueSend(rxQueue, &pk, 0);
        }
      }
    
      //Push the data to send (Loop until the TX Fifo is full or there is no more data to send)
      while( (uxQueueMessagesWaiting((xQueueHandle)txQueue) > 0) && !(nrfReadReg(REG_FIFO_STATUS)&0x20) )
      {
        xQueueReceive(txQueue, &pk, 0);//receive from thr type of crtp,so datalength is low
        pk.raw.size++;
        nrfWriteAck(0, (uint8_t*) pk.raw.data, pk.raw.size);
      }//
      
      //Re-enable the radio
      nrfSetEnable(true);			
//    }
      //clear the interruptions flags
      nrfWriteReg(REG_STATUS, 0x70);
//    if(reg_state&0x10)//
//    {
//      if(reg_state & 0x01)//TX FIFO
//      {
//        nrfFlushTx();
//      }
//    }	
//    nrfWriteReg(REG_STATUS,reg_state);
//    reg_state = nrfReadReg(REG_STATUS);
  }
}

/************************************************************************
 *@brief  initialize the nRF24L01 build the connected
 *@param  None
 *@retval None
 ************************************************************************/
static void radiolinkInitNRF24L01P(uint8_t modle)
{
  int i;
  uint8_t radioAddress[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	
  RADIO_DIS_CE;
  
  //Set radio RX P0 address
  nrfSetRxAddress(0, radioAddress);
	
  //Set the TX address
  nrfSetTxAddress(radioAddress);
  
  //Enable the receive channal 0
  nrfWriteReg(REG_EN_RXADDR,0x01);
	
  //Set the radio channel 40
  nrfSetChannel(40);
	
  //Set the radio data rate 
  nrfSetDateRate(RADIO_RATE_1M);
	
  //Set the channal 0 Auto Ack
  nrfWriteReg(REG_EN_AA, 0x01);
	
//  Set 500us and 10 times
  nrfWriteReg(REG_SETUP_RETR, 0x1a);
	
	
  switch(modle)
  {
      case RX_1:
          //Set the Rx data width
          nrfWriteReg(REG_RX_PW_P0,RX_PLOAD_WIDTH);	
          //Power the radio, Enable the RX_RD AND MAX_RT interruption, set the radio in PRX mode
          nrfWriteReg(REG_CONFIG, 0x0F);
          break;
      case TX_1:
          //Set the Tx data width
          nrfWriteReg(REG_RX_PW_P0,TX_PLOAD_WIDTH);	
          //Power the radio, Enable the TX_DS AND MAX_RT interruption, set the radio in PRX mode
          nrfWriteReg(REG_CONFIG, 0x0E);
          break;
      case RX_2:
          //Flush RX
          for(i=0;i<3;i++)
          nrfFlushRx();
          //Flush TX
          for(i=0;i<3;i++)
          nrfFlushTx();
          //Power the radio, Enable the RX_RD AND MAX_RT interruption, set the radio in PRX mode
          nrfWriteReg(REG_CONFIG, 0x0F);//2015.7.9
          //active the regist
          nrfActivate();
          // Enable the dynamic payload size and the ack payload for the pipe 0
          nrfWriteReg(REG_DYNPD,   0x01);
          nrfWriteReg(REG_FEATURE, 0x06);
          
          break;
      case TX_2:
          //Flush RX
          for(i=0;i<3;i++)
          nrfFlushRx();
          //Flush TX
          for(i=0;i<3;i++)
          nrfFlushTx();
          //Power the radio, Enable the TX_DS AND MAX_RT interruption, set the radio in PRX mode
          nrfWriteReg(REG_CONFIG, 0x0E);
          //active the regist
          nrfActivate();
          // Enable the dynamic payload size and the ack payload for the pipe 0
          nrfWriteReg(REG_DYNPD,   0x01);
          nrfWriteReg(REG_FEATURE, 0x06);
          
          break;
          default:
          
          break;
  }
  RADIO_EN_CE;

  vTaskDelay( pdMS_TO_TICKS( 100 ) ); //Wait for the chip to be ready
}

/****************************************************************************************
 *@brief check the nRF24L01 whether exist
 *@param None
 *@retval if one then successful,otherwise if zero then false
 ***************************************************************************************/  
static uint8_t nrf24l01Check(void)
{
    uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
    uint8_t i;

    nrfWriteBuf(REG_TX_ADDR,buf,5);//Write the five byte data into the address 
    vTaskDelay( pdMS_TO_TICKS( 1 ) );
    nrfReadBuf (REG_TX_ADDR,buf,5);//Read five byte data fron the address
    for(i=0;i<5;i++)if(buf[i]!=0xA5) return false;  //check the nRF24L01 is err                                           
    return true;                                    //check the nRF24L01 is right
}  

/************************************************************************
 *@brief	if interruption happen then run the radiolinkTask functions 
 *@param	None
 *@retval	None
 ************************************************************************/
static void interruptCallback(void)
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
   
  //To unlock RadioTask
  xSemaphoreGiveFromISR(dataRdy, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*************************************************************************
 *@brief  initialize the radio 
 *@param  None
 *@retval None
 *************************************************************************/
void radiolinkInit(void)
{
  if(isInit)
    return;
  //hardware initiallise
  nrfSpiInit();
  
  //check the hardware whether existence
  isInit = nrf24l01Check();
  if(!isInit)LedseqRun(LEDR,seq_armed);//seq_linkup
 
  //set the interrupt function
  nrfSetInterruptCallback(interruptCallback);

  vTaskSetApplicationTaskTag(0, (pdTASK_HOOK_CODE)TASK_RADIO_ID_NBR);

  // Initialise the semaphores 
  vSemaphoreCreateBinary(dataRdy);
  // Queue init 
  rxQueue = xQueueCreate(3, sizeof(RadioPacket));
  txQueue = xQueueCreate(3, sizeof(RadioPacket));
  //init the nrf24l01
  radiolinkInitNRF24L01P(RX_2);	//6.24
  // Launch the Radio link task 		
  xTaskCreate(RadioTask,(  char* )"RadioTask",
			configMINIMAL_STACK_SIZE,NULL, /*priority*/2, NULL);
  isInit = true;
}

/**********************************************************************
 *@brief test the radiolink whether is initialised
 *@param None
 *@retval if it is initialise then return one,otherwise return zero.
 **********************************************************************/
bool radiolinkTest(void)
{
  return isInit;
}

/********************************************************************************
 *@brief	reinitialise the radio
 *@param	None
 *@retval	None
 ********************************************************************************/
void radiolinkReInit(void)
{
  if (!isInit)
  return;
  radiolinkInitNRF24L01P(RX_2);
}

struct crtpLinkOperations * radiolinkGetLink(void)
{
  return &radioOp;
}
