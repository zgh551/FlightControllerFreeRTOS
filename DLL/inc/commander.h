#ifndef _COMMANDER_H_
#define _COMMANDER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "BoardDefineType.h"  
#include "stabilizer_types.h"
#include "commander_type.h"
#include "CRTP_Type.h"
  
#ifdef PLATFORM_CF1
  #define DEFAULT_YAW_MODE  PLUSMODE
#else
  #define DEFAULT_YAW_MODE  XMODE
#endif
  
#define COMMANDER_WDT_TIMEOUT_STABILIZE  pdMS_TO_TICKS(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   pdMS_TO_TICKS(800)
        


void commanderInit(void);
bool commanderTest(void);

uint32_t commanderGetInactivityTime(void);
void commanderExtrxSet(const CommanderCrtpValues* val);

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);

void commanderSendStateRemote(state_t state);
#ifdef __cplusplus
}
#endif
#endif
