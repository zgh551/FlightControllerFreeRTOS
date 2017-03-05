#ifndef _EULERANGLE_H_
#define _EULERANGLE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifndef M_PI 
#define M_PI 3.1415926535897932384626433832795
#endif

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);

#ifdef __cplusplus
}
#endif
#endif
