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

static bool isInit = false;

void ComModuleInit(void)
{
  if(isInit)
  return;
  //nRF24L01 confiure
  radiolinkInit();
  //base on the CRTP type to decode the data
  crtpInit();
  //reister the callback function,progress the remote data
  commanderInit();
  
  //link the channel to the radio module nRF24L01
  crtpSetLink(radiolinkGetLink());

  isInit = true;
}
 
bool ComModuleTest(void)
{
  isInit &= radiolinkTest();
  isInit &= crtpTest();
  isInit &= commanderTest();
  
  return isInit;
}
