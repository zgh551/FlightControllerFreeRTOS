#ifndef _COMMANDER_H_
#define _COMMANDER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "BoardDefineType.h"  
  
#define COMMANDER_WDT_TIMEOUT_STABALIZE  pdMS_TO_TICKS(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   pdMS_TO_TICKS(800)
  
typedef enum
{
  RATE,
  ANGLE
} RPYType;

//typedef struct _CRTPPacket CRTPPacket;

void commanderInit(void);
bool commanderTest(void);

void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired);
void commanderGetThrust(uint16_t* thrust);
void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType);
void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange);


#ifdef __cplusplus
}
#endif
#endif
