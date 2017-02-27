/**
******************************************************************************
* @file    ComModule.c
* @author  Zhu Guohua
* @version V1.0
* @date    07-March-2015
* @brief   This file is high level communication module
******************************************************************************
* @attention
* Quadcopter = qdcpt
* COPYRIGHT 2015 
******************************************************************************  
ComModule.c - High level communication module
*/
#include "ComModule.h"

static bool isInit =0;

void ComModuleInit(void)
{
  if(isInit)
  return;
  
  radiolinkInit();
  crtpInit();
  crtpSetLink(radiolinkGetLink()); 
  commanderInit();
  
  isInit = true;
}
 
bool ComModuleTest(void)
{
  isInit &= radiolinkTest();
  isInit &= crtpTest();
  isInit &= commanderTest();
  
  return isInit;
}
