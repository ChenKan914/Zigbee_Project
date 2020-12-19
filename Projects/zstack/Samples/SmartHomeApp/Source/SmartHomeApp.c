/**************************************************************************************************
  Filename:       SmartHomeApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SmartHomeApp.h"
#include "SmartHomeAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "stdio.h"
#include "string.h"
#include "DHT11.h"
#include "hal_types.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SmartHomeApp_ClusterList[SMARTHOMEAPP_MAX_CLUSTERS] =
{
  SMARTHOMEAPP_PERIODIC_CLUSTERID,
  SMARTHOMEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SmartHomeApp_SimpleDesc =
{
  SMARTHOMEAPP_ENDPOINT,              //  int Endpoint;
  SMARTHOMEAPP_PROFID,                //  uint16 AppProfId[2];
  SMARTHOMEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SMARTHOMEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SMARTHOMEAPP_FLAGS,                 //  int   AppFlags:4;
  SMARTHOMEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SmartHomeApp_ClusterList,  //  uint8 *pAppInClusterList;
  SMARTHOMEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SmartHomeApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SmartHomeApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SmartHomeApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SmartHomeApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SmartHomeApp_Init() is called.
devStates_t SmartHomeApp_NwkState;

uint8 SmartHomeApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SmartHomeApp_Periodic_DstAddr; //广播
afAddrType_t SmartHomeApp_Flash_DstAddr;    //组播
afAddrType_t SmartHomeApp_P2P_DstAddr;      //点播


afAddrType_t SmartHomeApp_AirContidion_DstAddr;

aps_Group_t SmartHomeApp_Group;

uint8 SmartHomeAppPeriodicCounter = 0;
uint8 SmartHomeAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SmartHomeApp_HandleKeys( uint8 shift, uint8 keys );
void SmartHomeApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SmartHomeApp_SendPeriodicMessage( void );
void SmartHomeApp_SendFlashMessage( uint16 flashTime );
void SmartHomeApp_Send_P2P_Message(void);
/*********************************************************************
 * SMARTHOME FUNCTIONS
 */

void SHCoordSendUartMsg(uartSendMsg msg);
void SHCoordReveiceUartMsg(uint8 *receiveMsg,uint16 len);



 void SHEndPointSendMacAddrMsg(void);
 void SHEndPointSendDHT11Msg(void);
 void SHEndPointSendFireAlarmMsg(void);
 void SHEndPointSendSmokeAlarmMsg(void);
 
 void SHCoordSendMacAddrMsg(afIncomingMSGPacket_t *pkt);
 void SHCoordSendDHT11Msg(afIncomingMSGPacket_t *pkt);
 void SHCoordSendFireAlarmMsg(afIncomingMSGPacket_t *pkt);
 void SHCoordSendSmokeAlarmMsg(afIncomingMSGPacket_t *pkt);

 void SHEndPointReveiceAirConMsg(afIncomingMSGPacket_t *pkt);

 void SHIRSendInit();
 void SHIR_Sending(uint8 *Sendcode35,uint8 *Sendcode32);

#define SHIRSendPin P0_7
#define SHSMOKEANDFIREPin P0_5            //定义P0.5口为传感器的输入端


void  (*p)(uint8,uint8,uint8);
   

   
void SHIRSendDelay_10us(uint16 usx10)
{                         
    unsigned int a=0,i=0;
	for(i=0;i<usx10;i++)
	{
		a = 12;
		while(a)
		{
			a--;
		}			
	}
}

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SmartHomeApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SmartHomeApp_Init( uint8 task_id )
{ 
  SmartHomeApp_TaskID = task_id;
  SmartHomeApp_NwkState = DEV_INIT;
  SmartHomeApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
  
  HalTimerInit();

  p = callBackFunction;//(HAL_TIMER_0,HAL_TIMER_CHANNEL_SINGLE,HAL_TIMER_CH_MODE_OUTPUT_COMPARE)
     
   HalTimerConfig(HAL_TIMER_3,HAL_TIMER_MODE_CTC,HAL_TIMER_CHANNEL_SINGLE,
   HAL_TIMER_CH_MODE_OUTPUT_COMPARE,TRUE,p);

   	P0DIR |= 0x80;           //P0.7定义为输出
	P0INP &=  ~0x80;  
  	SHIRSendPin = 0;
    HalTimerStart(HAL_TIMER_3,13);
    T1IE = 0;	

   
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SmartHomeAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SmartHomeApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;//广播
  SmartHomeApp_Periodic_DstAddr.endPoint = SMARTHOMEAPP_ENDPOINT;
  SmartHomeApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SmartHomeApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;//组播
  SmartHomeApp_Flash_DstAddr.endPoint = SMARTHOMEAPP_ENDPOINT;
  SmartHomeApp_Flash_DstAddr.addr.shortAddr = SMARTHOMEAPP_FLASH_GROUP;
  
  SmartHomeApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
  SmartHomeApp_P2P_DstAddr.endPoint = SMARTHOMEAPP_ENDPOINT; 
  SmartHomeApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器

  // Fill out the endpoint description.
  SmartHomeApp_epDesc.endPoint = SMARTHOMEAPP_ENDPOINT;
  SmartHomeApp_epDesc.task_id = &SmartHomeApp_TaskID;
  SmartHomeApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SmartHomeApp_SimpleDesc;
  SmartHomeApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SmartHomeApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SmartHomeApp_TaskID );

  // By default, all devices start out in Group 1
  SmartHomeApp_Group.ID = 0x0001;
  osal_memcpy( SmartHomeApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SMARTHOMEAPP_ENDPOINT, &SmartHomeApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SmartHomeApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SmartHomeApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SmartHomeApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SmartHomeApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SmartHomeApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SmartHomeApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SmartHomeApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SmartHomeApp_NwkState == DEV_ZB_COORD) ||
                 (SmartHomeApp_NwkState == DEV_ROUTER)
              || (SmartHomeApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
        	SHEndPointSendMacAddrMsg();
            osal_start_timerEx( SmartHomeApp_TaskID,
                              SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT,
                              SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SmartHomeApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SmartHomeApp_Init()).
  if ( events & SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
#if defined ( ENDDEVICE_DHT11 ) 
	SHEndPointSendDHT11Msg();
	osal_start_timerEx( SmartHomeApp_TaskID, SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT,
	(SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT_DHT11+ (osal_rand() & 0x00FF)) );
#endif

#if defined ( ENDDEVICE_FIREALARM )
	SHEndPointSendFireAlarmMsg();
	osal_start_timerEx( SmartHomeApp_TaskID, SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT,
	(SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT_FIREALARM+ (osal_rand() & 0x00FF)) );
#endif

#if defined ( ENDDEVICE_SMOKEALARM )
	SHEndPointSendSmokeAlarmMsg();
	osal_start_timerEx( SmartHomeApp_TaskID, SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT,
	(SMARTHOMEAPP_SEND_PERIODIC_MSG_TIMEOUT_SMOKEALARM+ (osal_rand() & 0x00FF)) );
#endif



    // return unprocessed events
    return (events ^ SMARTHOMEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SmartHomeApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SmartHomeApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    SHEndPointSendMacAddrMsg();
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SMARTHOMEAPP_ENDPOINT, SMARTHOMEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SMARTHOMEAPP_ENDPOINT, SMARTHOMEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SMARTHOMEAPP_ENDPOINT, &SmartHomeApp_Group );
    }
  }
  if ( keys & HAL_KEY_SW_6 )
  {
	SHEndPointSendDHT11Msg();
  }
}

void callBackFunction(uint8 timerId, uint8 channel, uint8 channelMode)
{
	//SHIRSendPin=~SHIRSendPin;  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SmartHomeApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SmartHomeApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  //HalUARTWrite(0,"REVC:\n",6);
  uint16 flashTime;

  switch ( pkt->clusterId )
  {
    case SMARTHOMEAPP_P2P_CLUSTERID:
      HalUARTWrite(0, "Rx:", 3);       //提示接收到数据
      HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //串口输出接收到的数据
      HalUARTWrite(0, "\n", 1);         // 回车换行
      break;    
    case SMARTHOMEAPP_PERIODIC_CLUSTERID:
      break;

    case SMARTHOMEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
    case SMARTHOME_MACADDRMSG:
	SHCoordSendMacAddrMsg(pkt);
	break;
    case SMARTHOME_TEMPHUMIMSG:
	SHCoordSendDHT11Msg(pkt);
	break;
	case SMARTHOME_FIREALARM:
	SHCoordSendFireAlarmMsg(pkt);
	break;
	case SMARTHOME_SMOKEALARM:
	SHCoordSendSmokeAlarmMsg(pkt);
	break;
	case SMARTHOME_AIRCONDITIONMSG:
	SHEndPointReveiceAirConMsg(pkt);
	break;
	default : break;
  }
}

/*********************************************************************
 * @fn      SmartHomeApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SmartHomeApp_SendPeriodicMessage( void )
{
}

/*********************************************************************
 * @fn      SmartHomeApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SmartHomeApp_SendFlashMessage( uint16 flashTime )
{
}

/*********************************************************************
 * @fn      SmartHomeApp_Send_P2P_Message
 *
 * @brief   point to point.
 *
 * @param   none
 *
 * @return  none
 */
void SmartHomeApp_Send_P2P_Message( void )   ///////
{
}


void SHEndPointSendMacAddrMsg()
{
  uint8 *macAddr = aExtendedAddress;
  
  if ( AF_DataRequest( &SmartHomeApp_P2P_DstAddr, &SmartHomeApp_epDesc,
                       SMARTHOME_MACADDRMSG,
                       8,
                       macAddr,
                       &SmartHomeApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}
void SHCoordSendMacAddrMsg(afIncomingMSGPacket_t *pkt)
{

  uartSendMsg msg;
  
  msg.header = 0xff;
  msg.shortAddr = pkt->srcAddr.addr.shortAddr;
  for(int i=0;i<8;i++)
 {
	msg.extAddr[i] =  pkt->cmd.Data[7-i];
 }
  msg.body.msgType = SMARTHOME_MACADDRMSG;
  msg.body.len = 0;

  for(int i=0;i<msg.body.len;i++)
  {
	  msg.body.data[i] = pkt->cmd.Data[8+i];
  }
  msg.tailer = 0x11;
  
  SHCoordSendUartMsg(msg);
}

void SHEndPointSendDHT11Msg()
{
  P0SEL &= 0x7f;                  //P0_7配置成通用io
  byte temp, humidity, data[12];
  DHT11(); 
  uint8 *macAddr = aExtendedAddress;
  temp = wendu_shi<<4 |wendu_ge;
  humidity = shidu_shi<<4 |shidu_ge;

  osal_memcpy(data, macAddr, 8); 
  osal_memcpy(&data[8],&temp, 1);
  osal_memcpy(&data[9],&humidity, 1);
  //HalUARTWrite(0,data,10);

    if ( AF_DataRequest( &SmartHomeApp_P2P_DstAddr, &SmartHomeApp_epDesc,
                       SMARTHOME_TEMPHUMIMSG,
                       10,
                       data,
                       &SmartHomeApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SHEndPointSendFireAlarmMsg()
{
	P0DIR &= ~0x20; 
	if(SHSMOKEANDFIREPin == 0)         //当浓度高于设定值时 ，执行条件函数        
        {
            DelayMS(10);          //延时抗干扰
            if(SHSMOKEANDFIREPin == 0)     //确定 浓度高于设定值时 ，执行条件函数
            {
              	byte data[8];
				uint8 *macAddr = aExtendedAddress;

				osal_memcpy(data, macAddr, 8);
				//HalUARTWrite(0,data,8);

				if ( AF_DataRequest( &SmartHomeApp_P2P_DstAddr, &SmartHomeApp_epDesc,
				                   SMARTHOME_FIREALARM,
				                   8,
				                   data,
				                   &SmartHomeApp_TransID,
				                   AF_DISCV_ROUTE,
				                   AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
				{
				}
				else
				{
				// Error occurred in request to send.
				}
            }
        }
}

void SHEndPointSendSmokeAlarmMsg()
{
	P0DIR &= ~0x20; 
	if(SHSMOKEANDFIREPin == 0)         //当浓度高于设定值时 ，执行条件函数        
        {
            DelayMS(10);          //延时抗干扰
            if(SHSMOKEANDFIREPin == 0)     //确定 浓度高于设定值时 ，执行条件函数
            {
              	byte data[8];
				uint8 *macAddr = aExtendedAddress;

				osal_memcpy(data, macAddr, 8);
				//HalUARTWrite(0,data,8);

				if ( AF_DataRequest( &SmartHomeApp_P2P_DstAddr, &SmartHomeApp_epDesc,
				                   SMARTHOME_SMOKEALARM,
				                   8,
				                   data,
				                   &SmartHomeApp_TransID,
				                   AF_DISCV_ROUTE,
				                   AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
				{
				}
				else
				{
				// Error occurred in request to send.
				}
            }
        }

}

void SHCoordSendDHT11Msg(afIncomingMSGPacket_t *pkt)
{
  //HalUARTWrite(0,pkt->cmd.Data,pkt->cmd.DataLength);
  uartSendMsg msg;
  
  msg.header = 0xff;
  msg.shortAddr = pkt->srcAddr.addr.shortAddr;
  for(int i=0;i<8;i++)
 {
	msg.extAddr[i] =  pkt->cmd.Data[7-i];
 }

  msg.body.msgType = SMARTHOME_TEMPHUMIMSG;
  msg.body.len = 2;

  for(int i=0;i<msg.body.len;i++)
  {
	  msg.body.data[i] = pkt->cmd.Data[8+i];
  }
  msg.tailer = 0x11;
  
SHCoordSendUartMsg(msg); 
}

void SHCoordSendFireAlarmMsg(afIncomingMSGPacket_t *pkt)
{
	  uartSendMsg msg;
	  
	  msg.header = 0xff;
	  msg.shortAddr = pkt->srcAddr.addr.shortAddr;
	  for(int i=0;i<8;i++)
	 {
		msg.extAddr[i] =  pkt->cmd.Data[7-i];
	 }
	
	  msg.body.msgType = SMARTHOME_FIREALARM;
	  msg.body.len = 0;
	
	  for(int i=0;i<msg.body.len;i++)
	  {
		  msg.body.data[i] = pkt->cmd.Data[8+i];
	  }
	  msg.tailer = 0x11;
	  
	SHCoordSendUartMsg(msg); 
}

void SHCoordSendSmokeAlarmMsg(afIncomingMSGPacket_t *pkt)
{
	  uartSendMsg msg;
	  
	  msg.header = 0xff;
	  msg.shortAddr = pkt->srcAddr.addr.shortAddr;
	  for(int i=0;i<8;i++)
	 {
		msg.extAddr[i] =  pkt->cmd.Data[7-i];
	 }
	
	  msg.body.msgType = SMARTHOME_SMOKEALARM;
	  msg.body.len = 0;
	
	  for(int i=0;i<msg.body.len;i++)
	  {
		  msg.body.data[i] = pkt->cmd.Data[8+i];
	  }
	  msg.tailer = 0x11;
	  
	SHCoordSendUartMsg(msg); 
}

void SHCoordSendUartMsg(uartSendMsg msg)
{
	uint8 data[50]={0};
     data[0] = msg.header;
     data[1] = msg.shortAddr>>8;
     data[2] = msg.shortAddr;
      for(int i=0;i<8;i++)
     {
    	data[3+i] =  msg.extAddr[i];
     }
     data[11] = msg.body.msgType;
     data[12] = msg.body.len;
	for(int i=0;i<msg.body.len;i++)
	 {
	  	data[13 + i] = msg.body.data[i];
	 }
     data[13+msg.body.len] = msg.tailer;

     HalUARTWrite(0,data,14+msg.body.len);
}

void SHCoordReveiceUartMsg(uint8 *receiveMsg,uint16 len)
{
	uartReceiveMsg msgData;
	
	msgData.header = receiveMsg[0];
	msgData.tailer= receiveMsg[len-1];
	uint16 shortAdr[2]; 
	shortAdr[0] = receiveMsg[1];
	shortAdr[1] = receiveMsg[2];
	msgData.shortAddr= (shortAdr[0] << 8) | shortAdr[1];
	msgData.body.msgType = receiveMsg[3];
	msgData.body.len = receiveMsg[4];
	for(int i = 0;i<msgData.body.len;i++)
	{
		msgData.body.data[i] = receiveMsg[5+i];
	}

	if(msgData.body.msgType == SMARTHOME_AIRCONDITIONMSG)
	{
	  	SmartHomeApp_AirContidion_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;//广播
	  	SmartHomeApp_AirContidion_DstAddr.endPoint = SMARTHOMEAPP_ENDPOINT;
	 	SmartHomeApp_AirContidion_DstAddr.addr.shortAddr = msgData.shortAddr;

	    if ( AF_DataRequest( &SmartHomeApp_AirContidion_DstAddr, &SmartHomeApp_epDesc,
	                       SMARTHOME_AIRCONDITIONMSG,
	                       msgData.body.len ,
	                       msgData.body.data,
	                       &SmartHomeApp_TransID,
	                       AF_DISCV_ROUTE,
	                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	  {
	  }
	  else
	  {
	    // Error occurred in request to send.
	  }
	}
}

 void SHEndPointReveiceAirConMsg(afIncomingMSGPacket_t *pkt)
 {
 	uint16 lenHex = pkt->cmd.DataLength;
	
 	uint8 binStr[72];
	memset(binStr,'0',72);
	uint8 hexChar[2];    
	for (int i = 0; i < lenHex; i++)    
	{        
		hexChar[0] = (pkt->cmd.Data[i] & 0xF0) >> 4;        
		hexChar[1] = pkt->cmd.Data[i] & 0x0F;        
		for (int j = 0; j < 2; j++)        
		{           
			 for (int k = 0; k < 4; k++)            
			{               
				if (hexChar[j] & (0x08 >> k))                
				{                    
					binStr[8 * i + 4 * j + k] = '1';                
				}            
			}        
		}    
	}

	uint8 Sendcode35[35],Sendcode32[32];
	for(int i = 0;i<35;i++)
	{
		Sendcode35[i] = binStr[i];
	}
	
	for(int i = 0;i<32;i++)
	{
		Sendcode32[i] = binStr[40 + i];
	}
	
	SHIR_Sending(Sendcode35,Sendcode32);	
 }
 
 void SHIR_Sending(uint8 *Sendcode35,uint8 *Sendcode32)
 {

      uint16 i=0;	
	
	SHIRSendPin = 1;
   	T1IE = 1;
	SHIRSendDelay_10us(6800);
	T1IE = 0;	
	SHIRSendPin = 1;
	SHIRSendDelay_10us(3850);

	for(i=0;i<35;i++)//发送地址码两个字节 先发低位
	{
		if(Sendcode35[i] == '1')					//发射逻辑1
		{
			SHIRSendPin = 1;	
			T1IE = 1;	//打开中断和T3中断
			SHIRSendDelay_10us(450);

			SHIRSendPin = 1;
			T1IE = 0;
			SHIRSendDelay_10us(1420);
		}
		else						//发射逻辑0
		{
			SHIRSendPin = 1;
			T1IE = 1;
			SHIRSendDelay_10us(450);

			SHIRSendPin = 1;
			T1IE = 0;
			SHIRSendDelay_10us(450);					
		}
	}
        for(i=0;i<1;i++)//发送地址码两个字节 先发低位
        {
        	SHIRSendPin = 1;	
             T1IE = 1;
		SHIRSendDelay_10us(450);
		
		SHIRSendPin = 1;
		T1IE = 0;
		SHIRSendDelay_10us(17000);
        }
	for(i=0;i<32;i++)//发送指令码两个字节 先发低位
	{
		if(Sendcode32[i] == '1')					//发射逻辑1
		{
			SHIRSendPin = 1;	
			T1IE = 1;
			SHIRSendDelay_10us(450);
			
			SHIRSendPin = 1;
			T1IE = 0;
			SHIRSendDelay_10us(1420);
		}
		else						//发射逻辑0
		{
			SHIRSendPin = 1;	
			T1IE = 1;
			SHIRSendDelay_10us(450);
			
			SHIRSendPin = 1;
			T1IE = 0;
			SHIRSendDelay_10us(450);					
		}
	}	
	
	SHIRSendPin = 1;	
	T1IE = 1;
	SHIRSendDelay_10us(450);	//协议尾
	T1IE = 0;   
	SHIRSendPin = 0;	
}
/*********************************************************************
*********************************************************************/
