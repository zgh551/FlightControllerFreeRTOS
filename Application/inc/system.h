#ifndef _SYSTEM_H_
#define _SYSTEM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
	
void systemLaunch(void);

void systemStart(void);
void systemWaitStart(void);
void systemSetCanFly(bool val);
bool systemCanFly(void);


#ifdef __cplusplus
}
#endif
#endif
