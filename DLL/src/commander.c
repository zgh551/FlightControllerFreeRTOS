#include "commander.h"

#define MIN_THRUST  0       //10000
#define MAX_THRUST  65500

struct CommanderCrtpValues
{
  uint16_t thrust;// 
  int8_t roll;    // 
  int8_t pitch;   //
  int8_t yaw;     //      
};

static struct CommanderCrtpValues __packed targetVal[2];
//static struct CommanderCrtpValues ActualVal[2];

static bool isInit;
static int  side=0;
static uint32_t lastUpdate;

static bool altHoldMode    = false;
static bool altHoldModeOld = false;

static void mydebugCrtpCB(CRTPPacket* pk);
static void commanderCrtpCB(CRTPPacket* pk);
static void paramCrtpCB(CRTPPacket* pk);

static void commanderWatchdogReset(void);

void commanderInit(void)
{
  if(isInit)
  return;

//  crtpInit();
//  isInit = crtpTest();
//  if(!isInit)LedseqRun(LEDR,seq_armed);//seq_linkup
  
  crtpInitTaskQueue(CRTP_PORT_DEBUG);
  crtpInitTaskQueue(CRTP_PORT_COMMANDER);  
  crtpInitTaskQueue(CRTP_PORT_PARAM);
  
  crtpRegisterPortCB(CRTP_PORT_DEBUG,       mydebugCrtpCB);	
  crtpRegisterPortCB(CRTP_PORT_COMMANDER,   commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_PARAM,       paramCrtpCB);
  

  lastUpdate = xTaskGetTickCount();
  isInit = true;
}

bool commanderTest(void)
{
  return isInit;
}

/******************************************************************
 *@brief  debug function for crtp callback function
 *@retval NULL
 ******************************************************************/
static void mydebugCrtpCB(CRTPPacket* pk)
{
  LedseqRun(LEDR, seq_linkup);
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  targetVal[!side] = *((struct CommanderCrtpValues*)pk->data);
  side = !side;
  commanderWatchdogReset();
}

static void paramCrtpCB(CRTPPacket* pk)
{
  altHoldMode = (bool)pk->data[0];
  commanderWatchdogReset();
}

/******************************************************************
 *@brief  Get the current tick count
 *@retval NULL
 ******************************************************************/
static void commanderWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

/******************************************************************
 *@brief  Get the tick count interval
 *@retval interval value
 ******************************************************************/
static uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

/******************************************************************
 *@brief  when Radio link Fail,then set the target angle to zero,
          after that,set the thrust to the zero.
 *@retval NULL
 ******************************************************************/
static void commanderWatchdog(void)
{
  int usedSide = side;
  uint32_t ticktimeSinceUpdate;

  ticktimeSinceUpdate = commanderGetInactivityTime();

  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_STABALIZE)
  {
    targetVal[usedSide].roll = 0;
    targetVal[usedSide].pitch = 0;
    targetVal[usedSide].yaw = 0;
  }
  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    targetVal[usedSide].thrust = 0;
    altHoldMode = false;
  }
  else
  {
    
  }
}

/******************************************************************
 *@brief  if the Rock set into Holdmode,then setAltHold will be set high 
          at one pulse,this pulse used to initial the current posture and
          the altHoldChange surport the change value for the holdmode
 *@retval NULL
 ******************************************************************/
void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange)
{
  *altHold       =  altHoldMode;                    // Still in altitude hold mode
  *setAltHold    =  altHoldMode && !altHoldModeOld; // Hover just activated
  *altHoldChange =  altHoldMode ? ((float) targetVal[side].thrust - 32767.) / 32767. : 0.0; //32767 Amount to change altitude hold target
  altHoldModeOld =  altHoldMode;
}

/******************************************************************
 *@brief  Set the angle type
 *@retval NULL
 ******************************************************************/
void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = ANGLE;
  *pitchType = ANGLE;
  *yawType   = RATE;
}

/******************************************************************
 *@brief  Get the euler angle from the Rock Target Value
 *@retval NULL
 ******************************************************************/
void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired)
{
  int usedSide = side;
	
  *eulerRollDesired  = targetVal[usedSide].roll;
  *eulerPitchDesired = targetVal[usedSide].pitch;
  *eulerYawDesired   = targetVal[usedSide].yaw;
}

/******************************************************************
 *@brief  Get the Thrust Value from the Rock 
 *@retval NULL
 ******************************************************************/
void commanderGetThrust(uint16_t* thrust)
{
  int usedSide = side;
  uint16_t rawThrust = targetVal[usedSide].thrust;

  if (rawThrust > MIN_THRUST)
  {
    *thrust = rawThrust;
  }
  else
  {
    *thrust = 0;
  }

  if (rawThrust > MAX_THRUST)
  {
    *thrust = MAX_THRUST;
  }

  commanderWatchdog();
}

//void commanderPutRPY(float* eulerRollActual, float* eulerPitchActual, float* eulerYawActual,CRTPPacket* pk)
//{
//  int usedSide = side;
//  uint8_t* temp; 
//  uint8_t  Dat[30];
//  uint8_t i=0;
//
//  ActualVal[usedSide].roll  = *eulerRollActual ;
//  ActualVal[usedSide].pitch = *eulerPitchActual;
//  ActualVal[usedSide].yaw   = *eulerYawActual  ;
////  temp = (uint8_t*)(&ActualVal[usedSide]);
////  for(i=0;i<12;i++)
////  {
////    pk->data[i]= temp[i];
////  }
//  pk->data = (uint8_t*)(&ActualVal[usedSide]);
//  
//  
//  //Dat = temp;
//  //&((*pk).data[0]) = Dat;
//  //(pk->data) = Dat;
//}
