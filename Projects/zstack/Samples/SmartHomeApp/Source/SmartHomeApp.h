/**************************************************************************************************
  Filename:       SmartHomeApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Sample Application definitions.


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef SMARTHOMEAPP_H
#define SMARTHOMEAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "Hal_timer.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define SMARTHOMEAPP_ENDPOINT           20

#define SMARTHOMEAPP_PROFID             0x0F08
#define SMARTHOMEAPP_DEVICEID           0x0001
#define SMARTHOMEAPP_DEVICE_VERSION     0
#define SMARTHOMEAPP_FLAGS              0

#define SMARTHOMEAPP_MAX_CLUSTERS       2

#define SMARTHOMEAPP_P2P_CLUSTERID      100
#define SMARTHOMEAPP_PERIODIC_CLUSTERID 101
#define SMARTHOMEAPP_FLASH_CLUSTERID    102

  
// Send Message Timeout
#define SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT   5000
#define SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT_DHT11   20000     // Every 20 seconds
#define SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT_SMOKEALARM   3000     // Every 3 seconds
#define SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT_FIREALARM   3000     // Every 3 seconds


// Application Events (OSAL) - These are bit weighted definitions.
#define SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT       0x0001
  
// Group ID for Flash Command
#define SMARTHOMEAPP_FLASH_GROUP                 0x0001
  
// Flash Command Duration - in milliseconds
#define SMARTHOMEAPP_FLASH_DURATION              1000
/*********************************************************************
 * SMARTHOMEDEFINE
 */
typedef enum
{
	SMARTHOME_MACADDRMSG = 0,
	SMARTHOME_TEMPHUMIMSG = 1,
	SMARTHOME_AIRCONDITIONMSG = 2,
	SMARTHOME_FIREALARM = 3,
	SMARTHOME_SMOKEALARM = 4,
	SMARTHOME_TEST
} msgType_t;


typedef struct 
{
    msgType_t msgType;
    uint8 len;
    uint8 data[10];
}uartMsgBody;

typedef struct 
{
	uint8 header;
	uint16      shortAddr;
	uint8 extAddr[8];
	uartMsgBody body;
	uint16 tailer;
}uartSendMsg;

typedef struct 
{
	uint8 header;
	uint16      shortAddr;
	uartMsgBody body;
	uint16 tailer;
}uartReceiveMsg;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void SmartHomeApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 SmartHomeApp_ProcessEvent( uint8 task_id, uint16 events );

extern  void callBackFunction(uint8 timerId, uint8 channel, uint8 channelMode);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SMARTHOMEAPP_H */
