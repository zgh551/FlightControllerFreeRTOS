#ifndef _RADIOLINK_H_
#define _RADIOLINK_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "CRTP_Type.h"

/*************************************************************************
 *@brief  initialize the radio 
 *@param  None
 *@retval None
 *************************************************************************/
void radiolinkInit(void);

/********************************************************************************
 *@brief	reinitialise the radio
 *@param	None
 *@retval	None
 ********************************************************************************/
void radiolinkReInit(void);

/**********************************************************************
 *@brief test the radiolink whether is initialised
 *@param None
 *@retval if it is initialise then return one,otherwise return zero.
 **********************************************************************/
bool radiolinkTest(void);

/************************************************************************
 *@brief	if interruption happen then run the radiolinkTask functions 
 *@param	None
 *@retval	None
 ************************************************************************/
static void interruptCallback(void);

/*********************************************************************************************
 *@brief	  receive the packet data from the txQueue and put the packet into the pk pointer 
 *@param[out] *pk: the packet pointer with information
 *@retval	  if state is not enable then return ENETDOWN,otherwise return zero.
 *********************************************************************************************/
int radioReceivePacket(CRTPPacket * pk);

/*****************************************************************************
 *@brief	send the packet data to txQueue
 *@param[in]*pk: the packet pointer with information
 *@retval	if state is not enable then return ENETDOWN,otherwise return zero.
 *****************************************************************************/
int radioSendPacket(CRTPPacket * pk);
	
struct crtpLinkOperations * radiolinkGetLink(void);

#ifdef __cplusplus
}
#endif
#endif
