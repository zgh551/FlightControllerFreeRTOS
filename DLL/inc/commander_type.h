#ifndef _COMMANDER_TYPE_H_
#define _COMMANDER_TYPE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

  /**
* CRTP commander data struct
*/
typedef struct _CommanderCrtpValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;   
}__packed CommanderCrtpValues;

#ifdef __cplusplus
}
#endif
#endif
