#include "system.h"

/* Private variable */
bool canFly;
static bool isInit;

/* System wide synchronisation */
xSemaphoreHandle StartMutex;

/* Private functions */
static void systemInit(void);
static bool systemTest(void);
static void systemTask(void *arg);
static void systemStart(void);

/* Public functions */
void systemLaunch(void)
{
	xTaskCreate(systemTask,( portCHAR* )"SYSTEM",
              configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
}

portTASK_FUNCTION( systemTask , pvParameters )
{
  bool pass = true;
  //Init the high-levels modules
  systemInit();
  
  //Test the modules
  pass &= systemTest();/////////////////

  //Start the firmware
  if(pass)
  {
    systemStart();
    LedseqRun(LEDL,seq_testPassed);//seq_linkup
  }
  else
  {
    if (systemTest())
    {
      while(1)
      {
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    }
    else
    {
      
    }
  }
	
  while(1)
  {
    vTaskDelay(portMAX_DELAY);
  }
}

//This must be the first module to be initialized!
static void systemInit(void)
{
  if(isInit)
    return;
  StartMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(StartMutex, portMAX_DELAY);
  
  LedseqInit();
  ComModuleInit();
  isInit = true;
}

static bool systemTest(void)
{
  bool pass=isInit;
  
  pass &= LedseqTest();
  pass &= ComModuleTest();
  return pass;
}

/* Global system variables */
static void systemStart()
{
  xSemaphoreGive(StartMutex);
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(StartMutex, portMAX_DELAY);
  xSemaphoreGive(StartMutex);
}

void systemSetCanFly(bool val)
{
  canFly = val;
}

bool systemCanFly(void)
{
  return canFly;
}