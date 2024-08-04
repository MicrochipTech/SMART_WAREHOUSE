// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2023 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <string.h>
#include "app.h"
#include "definitions.h"
#include "configuration.h"

#include "thread_demo.h"
#include "timers.h"

#include "openthread/udp.h"
#include "openthread/message.h"
#include "openthread/ip6.h"
#include "openthread/instance.h"
#include "openthread/error.h"
#include "openthread/thread.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define LED_BLINK_TIME_MS               (150)

#define ALIVE_LED_BLINK_TIME_MS         (500)

devMsgType_t demoCommand;

otUdpSocket aSocket;

/* The timer created for LED that blinks when it receives the data from the Leader */
static TimerHandle_t Data_sent_LED_Timer_Handle = NULL;
/* The timer created for LED that blinks when it sends data to the Leader*/
static TimerHandle_t Data_receive_LED_Timer_Handle = NULL;
/* The timer created for LED that blinks to show heartbeat of MCU*/
static TimerHandle_t Alive_LED_Timer_Handle = NULL;

/******************************************************************************
                        Definitions section
******************************************************************************/

/******************************************************************************
                        external variables section
******************************************************************************/
extern demoDevice_t demoDevices[];

extern otInstance *instance;



// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

extern otInstance *instance;
extern demoDevice_t demoDevices[TOTAL_DEMO_DEVICES];

static void processHelpCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processAutoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processDiscoverCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processGetDeviceInfoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processGetDeviceAddrCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processLightSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processLightGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoSensorGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoSensorSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoHVACSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoHVACGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processSolarSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processSolarGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processAccessSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processAccessGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
static void Data_sent_LED_Timer_Callback(TimerHandle_t xTimer)
{
    RGB_LED_GREEN_Off();    
    /* Keep compiler happy. */
     (void)xTimer;    
}
static void Data_receive_LED_Timer_Callback(TimerHandle_t xTimer)
{
    RGB_LED_BLUE_Off();
    /* Keep compiler happy. */
     (void)xTimer;    
}

static void Alive_LED_Timer_Callback(TimerHandle_t xTimer)
{
    USER_LED_Toggle();
    /* Keep compiler happy. */
     (void)xTimer;    
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

static const SYS_CMD_DESCRIPTOR gatewayCmdsTbl[]=
{
   {"help", processHelpCmd, "help->Shows help you're reading now."},
   {"auto", processAutoCmd, "auto->To check baudrate and working of serial"},
   {"discover", processDiscoverCmd, "getDeviceInfo->Gets Device Info."},
   {"getDeviceInfo", processGetDeviceInfoCmd, "getDeviceInfo->Gets Device Info."},
   {"getDeviceAddr", processGetDeviceAddrCmd, "getDeviceInfo->Gets Device Info."},
   {"lightSet", processLightSetCmd, "[index][on/off][hue][saturation][level]"},
   {"lightGet", processLightGetCmd, "[index][on/off][hue][saturation][level]"},
   {"thermoSensorSet", processthermoSensorSetCmd, "get->Gets Device Payload."},
   {"thermoSensorGet", processthermoSensorGetCmd, "get->Gets Device Payload."},
   {"thermoHVACSet", processthermoHVACSetCmd, "set->Sets Device Payload."},
   {"thermoHVACGet", processthermoHVACGetCmd, "set->Sets Device Payload."},
   {"solarSet", processSolarSetCmd, "set->Sets Device Payload."},
   {"solarGet", processSolarGetCmd, "set->Sets Device Payload."},
   {"accessSet", processAccessSetCmd, "set->Sets Device Payload."},
   {"accessGet", processAccessGetCmd, "set->Sets Device Payload."},
};

static void processHelpCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    uint8_t index;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Commands: \r");
    for (index = 0; index < (sizeof(gatewayCmdsTbl)/sizeof(*gatewayCmdsTbl)); index++)
    {
        app_printf("%s\r", gatewayCmdsTbl[index].cmdStr);
    }
}

static void processAutoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
	(*pCmdIO->pCmdApi->msg)(cmdIoParam, "auto\r");
}

static void processLightSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    devTypeRGBLight_t *rgbLight = (devTypeRGBLight_t *)demoCommand.msg;

    if(argc == 6)
    {
        rgbLight->onOff = atoi(argv[2]);
        rgbLight->hue = atoi(argv[3]);
        rgbLight->saturation = atoi(argv[4]);
        rgbLight->level = atoi(argv[5]);
        demoCommand.msgType = MSG_TYPE_LIGHT_SET;
        threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 4 + sizeof(devTypeRGBLight_t), (uint8_t *)&demoCommand);
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid Command\r");
    }
}

static void processLightGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
//    demoCommand.msgType = MSG_TYPE_THERMO_SENSOR_GET;
//    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 5, (uint8_t *)&demoCommand);
    devTypeRGBLight_t *lightReport = (devTypeRGBLight_t *)demoDevices[atoi(argv[1])].devMsg;
    app_printf("On/Off-%d, H - %03d, S - %03d, V - %03d\r", lightReport->onOff, lightReport->hue, lightReport->saturation, lightReport->level);
}

static void processthermoSensorSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    devTypeThermostatSensorSet_t *tempSensorSet = (devTypeThermostatSensorSet_t *)demoCommand.msg;
    memcpy(&tempSensorSet->reportedThermostatHVAC, &demoDevices[atoi(argv[2])].devAddr, OT_IP6_ADDRESS_SIZE);
    tempSensorSet->reportInterval = atoi(argv[3]);

    demoCommand.msgType = MSG_TYPE_THERMO_SENSOR_SET;
    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 4 + sizeof(devTypeThermostatSensorSet_t), (uint8_t *)&demoCommand);
}

static void processthermoSensorGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
//    demoCommand.msgType = MSG_TYPE_THERMO_SENSOR_GET;
//    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 5, (uint8_t *)&demoCommand);
    devTypeThermostatSensorReport_t *tempReport = (devTypeThermostatSensorReport_t *)demoDevices[atoi(argv[1])].devMsg;
    app_printf("Temp-%0.2f\r", tempReport->temperature);
}

static void processthermoHVACSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    devTypeThermostatHVACSet_t *tempHVACSet = (devTypeThermostatHVACSet_t *)demoCommand.msg;
    tempHVACSet->setPoint = (float)(atoi(argv[2]) / 10);

    demoCommand.msgType = MSG_TYPE_THERMO_HVAC_SET;
    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 4 + sizeof(devTypeThermostatHVACSet_t), (uint8_t *)&demoCommand);
}

static void processthermoHVACGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
//    demoCommand.msgType = MSG_TYPE_THERMO_HVAC_GET;
//    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 5, (uint8_t *)&demoCommand);
    devTypeThermostatHVACReport_t *hvacReport = (devTypeThermostatHVACReport_t *)demoDevices[atoi(argv[1])].devMsg;
    app_printf("Set Temp-%0.2f, On/Off-%d\r", (float)hvacReport->setPoint, hvacReport->onOffStatus);
}

static void processSolarSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    devTypeSolarSet_t *solarSet = (devTypeSolarSet_t *)demoCommand.msg;
    solarSet->position = (atoi(argv[2]));

    demoCommand.msgType = MSG_TYPE_SOLAR_SET;
    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 4 + sizeof(devTypeSolarSet_t), (uint8_t *)&demoCommand);
}

static void processSolarGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
//    demoCommand.msgType = MSG_TYPE_SOLAR_GET;
//    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 5, (uint8_t *)&demoCommand);
    
    devTypeSolarReport_t *solar = (devTypeSolarReport_t *)demoDevices[atoi(argv[1])].devMsg;
    app_printf("Solar: Volt-%0.2f, Intensity-%0.2f\r", (float)solar->voltage, (float)solar->lightIntensity);
}

static void processAccessSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    devTypeAccessControlSet_t *accessSet = (devTypeAccessControlSet_t *)demoCommand.msg;
    accessSet->garageDoor = (atoi(argv[2]));

    demoCommand.msgType = MSG_TYPE_ACCESS_CONTROL_SET;
    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 4 + sizeof(devTypeAccessControlSet_t), (uint8_t *)&demoCommand);
}

static void processAccessGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    //demoCommand.msgType = MSG_TYPE_ACCESS_CONTROL_GET;
    //threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 5, (uint8_t *)&demoCommand);
    devTypeAccessControlReport_t *accessReport = (devTypeAccessControlReport_t *)demoDevices[atoi(argv[1])].devMsg;
    app_printf("Garage Door - %d\r", accessReport->garageDoor);
}

static void processGetDeviceInfoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
	app_printf("Type-%d Name - %s\r", demoDevices[atoi(argv[1])].devType, demoDevices[atoi(argv[1])].devName);
}

static void processGetDeviceAddrCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    char string[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&demoDevices[atoi(argv[1])].devAddr, string, OT_IP6_ADDRESS_STRING_SIZE);
	app_printf("Device Addr - %s\r", string);
}

static void processDiscoverCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const otIp6Address *mPeerAddr;
    const otIp6Address *mSockAddr;
    mSockAddr = otThreadGetMeshLocalEid(instance);
    demoCommand.msgType = MSG_TYPE_GATEWAY_DISCOVER_REQ;
    memcpy(&demoCommand.msg, mSockAddr, OT_IP6_ADDRESS_SIZE);
    mPeerAddr = otThreadGetRealmLocalAllThreadNodesMulticastAddress(instance);
    threadUdpSend((otIp6Address *)mPeerAddr, 4 + sizeof(otIp6Address), (uint8_t *)&demoCommand);
}


void printIpv6Address(void)
{
//    APP_Msg_T    appMsg;

    const otNetifAddress *unicastAddrs = otIp6GetUnicastAddresses(instance);
    //app_printf("Unicast Address :\r");
    
//    char string[OT_IP6_ADDRESS_STRING_SIZE];
//    otIp6AddressToString(&(unicastAddrs->mAddress), string, OT_IP6_ADDRESS_STRING_SIZE);
//    app_printf("Unicast Address :\r%s\r", string);

    for (const otNetifAddress *addr = unicastAddrs; addr; addr = addr->mNext)
    {
        char string[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&(addr->mAddress), string, OT_IP6_ADDRESS_STRING_SIZE);
        //app_printf("%s\r\n", string);
    }
//    appMsg.msgId = ;
//    OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
}

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    appData.appQueue = xQueueCreate( 64, sizeof(APP_Msg_T) );
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    Alive_LED_Timer_Handle = xTimerCreate("Milli_Timer",pdMS_TO_TICKS(ALIVE_LED_BLINK_TIME_MS),pdTRUE, ( void * ) 0, Alive_LED_Timer_Callback);
    (void)xTimerStart(Alive_LED_Timer_Handle,pdMS_TO_TICKS(ALIVE_LED_BLINK_TIME_MS));
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    APP_Msg_T   appMsg;
    APP_Msg_T   *p_appMsg;
    p_appMsg = &appMsg;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            //appData.appQueue = xQueueCreate( 10, sizeof(APP_Msg_T) );

            threadAppinit();
            //app_printf("App_Log: Thread Network is getting initialized\n");


            SYS_CMD_ADDGRP(gatewayCmdsTbl, sizeof(gatewayCmdsTbl)/sizeof(*gatewayCmdsTbl), "Gateway", ": Gateway commands");
            
            if (appInitialized)
            {
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }
        case APP_STATE_SERVICE_TASKS:
        {
            if (OSAL_QUEUE_Receive(&appData.appQueue, &appMsg, OSAL_WAIT_FOREVER))
            {
                if(p_appMsg->msgId == APP_MSG_OT_NW_CONFIG_EVT)
                {
                    threadConfigNwParameters();
                }
                else if(p_appMsg->msgId == APP_MSG_OT_NWK_START_EVT)
                {
                    threadNwStart();
                }
                else if(p_appMsg->msgId == APP_MSG_OT_PRINT_IP_EVT)
                {
                    printIpv6Address();
                }
                else if(p_appMsg->msgId == APP_MSG_OT_STATE_HANDLE_EVT)
                {
                    threadHandleStateChange();
                }
                else if(p_appMsg->msgId == APP_MSG_OT_RECV_CB)
                {
                    otMessageInfo *aMessageInfo;
                    otMessage *aMessage;
                    
                    uint8_t aMessageInfoLen = p_appMsg->msgData[0];
                    uint16_t aMessageLen = 0;
                    
                    aMessageLen = (uint16_t)p_appMsg->msgData[aMessageInfoLen + 2];
                    
                    aMessageInfo = (otMessageInfo *)&p_appMsg->msgData[MESSAGE_INFO_INDEX];
                    aMessage = (otMessage *)&p_appMsg->msgData[aMessageInfoLen + 2 + 1];
                    
                    threadReceiveData(aMessageInfo, aMessageLen, (uint8_t *)aMessage);
                }
                else if(p_appMsg->msgId == APP_MSG_OT_SEND_ADDR_TMR_EVT)
                {
#ifndef GATEWAY_HOST_CONNECTED
                    app_printf("Discovering Devices...\r\n");
#endif
                    threadSendPeriodicMsg();
                }
                
            }
            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void otUdpReceiveCb(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    APP_Msg_T appMsg_otUdpReceiveCb;
    memset(&appMsg_otUdpReceiveCb, 0, sizeof(APP_Msg_T));
    appMsg_otUdpReceiveCb.msgId = APP_MSG_OT_RECV_CB;
    
    uint16_t aMessageLen = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
    uint8_t aMessageInfoLen = sizeof(otMessageInfo);
    
    appMsg_otUdpReceiveCb.msgData[0] = aMessageInfoLen;
    memcpy(&appMsg_otUdpReceiveCb.msgData[MESSAGE_INFO_INDEX], aMessageInfo, sizeof(otMessageInfo));

    appMsg_otUdpReceiveCb.msgData[aMessageInfoLen + MESSAGE_INFO_INDEX + 1] = (uint8_t)aMessageLen;

    otMessageRead(aMessage,otMessageGetOffset(aMessage), &appMsg_otUdpReceiveCb.msgData[aMessageInfoLen + MESSAGE_INFO_INDEX + 2], aMessageLen);
    
//    app_printf("App_Log: Data Received\r\n");
    OSAL_QUEUE_SendISR(&appData.appQueue, &appMsg_otUdpReceiveCb);
//    OSAL_QUEUE_Send(&appData.appQueue, &appMsg_otUdpReceiveCb, 0);
}

void threadUdpOpen()
{
   otError err;
   //app_printf("App_log: UDP Open\n");
   err = otUdpOpen(instance, &aSocket, otUdpReceiveCb, NULL);
   if (err != OT_ERROR_NONE)
   {
      app_printf("App_Err: UDP Open failed\n");
       //print error code
       assert(err);
   }
    /* The timer created for LED that blinks when it receives the data from the Leader */
    Data_sent_LED_Timer_Handle = xTimerCreate("Milli_Timer",pdMS_TO_TICKS(LED_BLINK_TIME_MS),pdFALSE, ( void * ) 0, Data_sent_LED_Timer_Callback);
    /* The timer created for LED that blinks when it sends data to the Leader*/
    Data_receive_LED_Timer_Handle = xTimerCreate("Milli_Timer",pdMS_TO_TICKS(LED_BLINK_TIME_MS),pdFALSE, ( void * ) 0, Data_receive_LED_Timer_Callback);

}

void threadUdpSend(otIp6Address *mPeerAddr, uint8_t msgLen, uint8_t* msg)
{
    otError err = OT_ERROR_NONE;
    otMessageInfo msgInfo;
//    const otIp6Address *mPeerAddr;
    const otIp6Address *mSockAddr;
    memset(&msgInfo,0,sizeof(msgInfo));
//    otIp6AddressFromString("ff03::1",&msgInfo.mPeerAddr);
    mSockAddr = otThreadGetMeshLocalEid(instance);
//    mPeerAddr = otThreadGetRealmLocalAllThreadNodesMulticastAddress(instance);
    memcpy(&msgInfo.mSockAddr, mSockAddr, OT_IP6_ADDRESS_SIZE);
    memcpy(&msgInfo.mPeerAddr, mPeerAddr, OT_IP6_ADDRESS_SIZE);
    
    msgInfo.mPeerPort = UDP_PORT_NO;
    
    do {
        otMessage *udp_msg = otUdpNewMessage(instance,NULL);
        err = otMessageAppend(udp_msg,msg,msgLen);
        if(err != OT_ERROR_NONE)
        {
            //app_printf("App_Err: UDP Message Add fail\n");
            break;
        }
        
        err = otUdpSend(instance,&aSocket,udp_msg,&msgInfo);
        if(err != OT_ERROR_NONE)
        {
            //app_printf("App_Err: UDP Send fail\n");
            break;
        }
//        app_printf("App_Log: UDP Sent data: %d\r\n",err);
        RGB_LED_GREEN_On();
        if( xTimerIsTimerActive( Data_sent_LED_Timer_Handle ) != pdFALSE )
        {
            /* xTimer is active, do something. */
            (void)xTimerStop( Data_sent_LED_Timer_Handle, pdMS_TO_TICKS(0) );
        }
        (void)xTimerStart(Data_sent_LED_Timer_Handle,pdMS_TO_TICKS(0));

    }while(false);
    
}

void threadUdpBind()
{
   otError err;
   otSockAddr addr;
   memset(&addr,0,sizeof(otSockAddr));
   addr.mPort = UDP_PORT_NO;
   do
   {
        err = otUdpBind(instance, &aSocket, &addr, OT_NETIF_THREAD);
        if (err != OT_ERROR_NONE) {
            app_printf("App_Err: UDP Bind fail Err:%d\n",err);
            break;
        }
        //app_printf("App_Log: UDP Listening on port %d\n",UDP_PORT_NO);
   }while(false);
}

void threadReceiveData(const otMessageInfo *aMessageInfo, uint16_t length, uint8_t *msgPayload)
{
    char string[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&(aMessageInfo->mPeerAddr), string, OT_IP6_ADDRESS_STRING_SIZE);
    
    RGB_LED_BLUE_On();
    if( xTimerIsTimerActive( Data_receive_LED_Timer_Handle ) != pdFALSE )
    {
        /* xTimer is active, do something. */
        (void)xTimerStop( Data_receive_LED_Timer_Handle, pdMS_TO_TICKS(0) );
    }
    (void)xTimerStart(Data_receive_LED_Timer_Handle,pdMS_TO_TICKS(0));

    devMsgType_t *rxMsg;
    rxMsg = (devMsgType_t *)msgPayload;
    
//    app_printf("App_Log: UDP Received from [%s] len:[%d] type:[%d]\r\n", string, length, rxMsg->msgType);
    
    if(MSG_TYPE_GATEWAY_DISCOVER_RESP == rxMsg->msgType)
    {
        devDetails_t *devDetails;
        devDetails = (devDetails_t *)rxMsg->msg;
        addDemoDevice(&(aMessageInfo->mPeerAddr), devDetails->devNameSize, devDetails->devName, devDetails->devType);
    }
    else if(MSG_TYPE_LIGHT_REPORT == rxMsg->msgType)
    {
        updateDemoStatus(&(aMessageInfo->mPeerAddr), rxMsg->msg);
#ifndef GATEWAY_HOST_CONNECTED
        devTypeRGBLight_t *lightReport = (devTypeRGBLight_t *)rxMsg->msg;
        app_printf("On/Off-%d, H - %03d, S - %03d, V - %03d\r", lightReport->onOff, lightReport->hue, lightReport->saturation, lightReport->level);
#endif
    }
    else if(MSG_TYPE_THERMO_SENSOR_REPORT == rxMsg->msgType)
    {
        updateDemoStatus(&(aMessageInfo->mPeerAddr), rxMsg->msg);
#ifndef GATEWAY_HOST_CONNECTED
        devTypeThermostatSensorReport_t *tempReport = (devTypeThermostatSensorReport_t *)rxMsg->msg;
        app_printf("Temp-%0.2f\r", tempReport->temperature);
#endif
    }
    else if(MSG_TYPE_THERMO_HVAC_REPORT == rxMsg->msgType)
    {
        updateDemoStatus(&(aMessageInfo->mPeerAddr), rxMsg->msg);
#ifndef GATEWAY_HOST_CONNECTED
        devTypeThermostatHVACReport_t *hvacReport = (devTypeThermostatHVACReport_t *)rxMsg->msg;
        app_printf("Set Temp-%0.2f, On/Off-%d\r", (float)hvacReport->setPoint, hvacReport->onOffStatus);
#endif
    }
    else if(MSG_TYPE_SOLAR_REPORT == rxMsg->msgType)
    {
        updateDemoStatus(&(aMessageInfo->mPeerAddr), rxMsg->msg);
#ifndef GATEWAY_HOST_CONNECTED
        devTypeSolarReport_t *solar = (devTypeSolarReport_t *)rxMsg->msg;
        app_printf("Solar: Volt-%0.2f, Intensity-%0.2f\r", (float)solar->voltage, (float)solar->lightIntensity);
#endif
    }
    else if(MSG_TYPE_ACCESS_CONTROL_REPORT == rxMsg->msgType)
    {
        updateDemoStatus(&(aMessageInfo->mPeerAddr), rxMsg->msg);
#ifndef GATEWAY_HOST_CONNECTED
        devTypeAccessControlReport_t *accessReport = (devTypeAccessControlReport_t *)rxMsg->msg;
        app_printf("Garage Door - %d\r", accessReport->garageDoor);
#endif
    }
}

/*******************************************************************************
 End of File
 */
