#include "LedSeq.h"

/* Led sequence priority */
static ledseq_t * sequences[] = {
  seq_testPassed,   
  seq_lowbat,      
  seq_charged,
  seq_charging,
  seq_bootloader,
  seq_armed,
  seq_calibrated,
  seq_alive,
  seq_linkup,
};

/* Led sequences */
ledseq_t seq_lowbat[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_armed[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(250)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_calibrated[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(450)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_alive[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(1950)},
  {    0, LEDSEQ_LOOP},
};

//TODO: Change, right now is called so fast it looks like seq_lowbat
ledseq_t seq_altHold[] = {
  { true, LEDSEQ_WAITMS(1)},
  {false, LEDSEQ_WAITMS(50)},
  {    0, LEDSEQ_STOP},
};

ledseq_t seq_linkup[] = {
  { true, LEDSEQ_WAITMS(10)},
  {false, LEDSEQ_WAITMS(10)},
  {    0, LEDSEQ_STOP},
};


ledseq_t seq_charged[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_charging[] = {
  { true, LEDSEQ_WAITMS(200)},
  {false, LEDSEQ_WAITMS(800)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_bootloader[] = {
  { true, LEDSEQ_WAITMS(500)},
  {false, LEDSEQ_WAITMS(500)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_testPassed[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_STOP},
};

/* Led sequence handling machine implementation */
#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

static void RunLedseq(xTimerHandle xTimer);
static int  GetPrio(ledseq_t *seq);
static void UpdateActive(Led_TypeDef led);

//State of every sequence for every led: LEDSEQ_STOP if stopped or the current step
static int state[LEDn][SEQ_NUM];
//Active sequence for each led
static int activeSeq[LEDn];

static xTimerHandle timer[LEDn];

static xSemaphoreHandle LedseqSem;

static bool isInit = false;

/*****************************************
 *@brief  initialise the led sequence
 *@param  None
 *@retval None
******************************************/
void LedseqInit(void)
{
  int i,j;
  
  if(isInit)
    return;
  //Hardware initialise
  STM_LED_Init(LEDL);
  STM_LED_Init(LEDR);
  
  //Initialise the sequences state
  for(i=0; i<LEDn; i++) 
  {
      activeSeq[i] = LEDSEQ_STOP;
	  for(j=0; j<SEQ_NUM; j++)
	  state[i][j] = LEDSEQ_STOP;
  }
  
  //Initialise the soft timers that runs the led sequences for each leds.
  for(i=0; i<LEDn; i++)
    timer[i] = xTimerCreate((const char *)"ledseqTimer", pdMS_TO_TICKS(1000), pdFALSE, (void*)i, RunLedseq);
  
  vSemaphoreCreateBinary(LedseqSem);
  
  isInit = true;
}

/**********************************************************************
 *@brief test the ledseq whether is initialised
 *@param None
 *@retval if it is initialise then return one,otherwise return zero.
 **********************************************************************/
bool LedseqTest(void)
{
  return isInit;// & ledTest();
}

/********************************************************************
 *@brief 
 */
void LedseqRun(Led_TypeDef led, ledseq_t *sequence)
{
  int prio = GetPrio(sequence);
  
  if(prio<0) return;
  
  xSemaphoreTake(LedseqSem, portMAX_DELAY);
  state[led][prio] = 0;                     //Reset the seq. to its first step
  UpdateActive(led);                        //set the activeSeq = prio
  xSemaphoreGive(LedseqSem);//Binary Semaphore 
  
  //Run the first step if the new seq is the active sequence
  if(activeSeq[led] == prio)
    RunLedseq(timer[led]);
}

void LedseqStop(Led_TypeDef led, ledseq_t *sequence)
{
  int prio = GetPrio(sequence);
  
  if(prio<0) return;
  
  xSemaphoreTake(LedseqSem, portMAX_DELAY);
  state[led][prio] = LEDSEQ_STOP;  //Stop the seq.
  UpdateActive(led);
  xSemaphoreGive(LedseqSem);
  
  //Run the next active sequence (if any...)
  RunLedseq(timer[led]);
}

/************************************************************************
 *@brief set the led sequence of the time of on and off
 *@param *sequence:the led sequence
 *@param onTime:led open time;
 *@param offTime:the led close time
 *@retval None
 ************************************************************************/
void LedseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime)
{
  sequence[0].action = onTime;
  sequence[1].action = offTime;
}

/******************************************************************************************* 
 *@brief the Center of the led sequence machine. This function is executed by the FreeRTOS
 *       timer and runs the sequences
 *@param xTimer:xTimerHandle
 *@retval None
 ********************************************************************************************/
static void RunLedseq( xTimerHandle xTimer )
{
  Led_TypeDef led = pvTimerGetTimerID(xTimer) ? LEDR : LEDL;
  ledseq_t *step;
  bool leave=false;
	
  while(!leave) 
  {
    int prio = activeSeq[led];
  
    if (prio == LEDSEQ_STOP)return;
    
    step = &sequences[prio][state[led][prio]];

    state[led][prio]++;
    
    xSemaphoreTake(LedseqSem, portMAX_DELAY);
    switch(step->action)
    {
      case LEDSEQ_LOOP:
        state[led][prio] = 0;
        break;
      case LEDSEQ_STOP:
        state[led][prio] = LEDSEQ_STOP;
        UpdateActive(led);
        break;
      default:  //The step is a LED action and a time
        STM_LED_Set(led,step->value);
        if (step->action == 0)
        break;
        xTimerChangePeriod(xTimer, pdMS_TO_TICKS(step->action), 0);
        xTimerStart(xTimer, 0);
        leave=true;
        break;
    }
    xSemaphoreGive(LedseqSem);
  }
}

//Utility functions
/***********************************************************************************
 *@brief get the led sequence priority
 *@param *seq:the led sequence 
 *@retval if the sequence is right then return the led sequence,otherwise return -1
 ***********************************************************************************/
static int GetPrio(ledseq_t *seq)
{
  int prio;

  //Find the priority of the sequence
  for(prio=0; prio<SEQ_NUM; prio++)
    if(sequences[prio]==seq) return prio;
  
  return -1; //Invalid sequence
}
/***************************************************
 *@brief  update the activeSeq to the priority
 *@param  led:input the LEDL or LEDR
 *@retval None
 ***************************************************/
static void UpdateActive(Led_TypeDef led)
{
  int prio;
  
  activeSeq[led]=LEDSEQ_STOP;
  STM_LED_Set(led,OFF);
  
  for(prio=0;prio<SEQ_NUM;prio++)
  {
    if (state[led][prio] != LEDSEQ_STOP)
    {
      activeSeq[led]=prio;
      break;
    }
  }
}
