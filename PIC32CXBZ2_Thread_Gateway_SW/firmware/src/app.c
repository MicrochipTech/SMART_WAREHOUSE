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
#include "thread_demo.h"
#include "udp_demo.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

devMsgType_t demoCommand;

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
devDetails_t threadDevice;

extern otInstance *instance;
extern demoDevice_t demoDevices[TOTAL_DEMO_DEVICES];

static void processHelpCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processAutoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processDiscoverCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processGetDeviceInfoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processGetDeviceAddrCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processLightCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoSensorGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoSensorSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoHVACSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void processthermoHVACGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
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


/* TODO:  Add any necessary local functions.
*/



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
   {"light", processLightCmd, "[index][on/off][hue][saturation][level]"},
   {"thermoSensorSet", processthermoSensorSetCmd, "get->Gets Device Payload."},
   {"thermoSensorGet", processthermoSensorGetCmd, "get->Gets Device Payload."},
   {"thermoHVACSet", processthermoHVACSetCmd, "set->Sets Device Payload."},
   {"thermoHVACGet", processthermoHVACGetCmd, "set->Sets Device Payload."},
};

//static void processDiscoverCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
//{
//    const void* cmdIoParam = pCmdIO->cmdIoParam;
//    devMsgType_t demoCommand;
//    const otIp6Address *mPeerAddr;
//    const otIp6Address *mSockAddr;
//    mSockAddr = otThreadGetMeshLocalEid(instance);
//    demoCommand.msgType = MSG_TYPE_GATEWAY_DISCOVER_REQ;
//    memcpy(&demoCommand.msg, mSockAddr, OT_IP6_ADDRESS_SIZE);
//    mPeerAddr = otThreadGetRealmLocalAllThreadNodesMulticastAddress(instance);
//    threadUdpSend((otIp6Address *)mPeerAddr, 4 + sizeof(otIp6Address), (uint8_t *)&demoCommand);
//    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "AOK\r\n");
//}
//
//static void processTempSensorGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
//{
//    const void* cmdIoParam = pCmdIO->cmdIoParam;
//    if(argc == 1)
//    {
//        uint8_t index = 0xFF;
//        devMsgType_t demoCommand;
//        
//        for(uint8_t indx = 0; indx < TOTAL_DEMO_DEVICES; indx++)
//        {
//            if(demoDevices[indx].isAvailable && (demoDevices[indx].devType == DEVICE_TYPE_TEMP_SENSOR))
//            {
//                index = indx;
//                break;
//            }
//        }
//        if(index < TOTAL_DEMO_DEVICES)
//        {
//            //index = atoi(argv[1]);
//            demoCommand.msgType = MSG_TYPE_TEMP_SENSOR_GET;
//            threadUdpSend((otIp6Address *)&demoDevices[index].devAddr, sizeof(demoCommand), (uint8_t *)&demoCommand);
//            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "AOK\r\n");
//        }
//        else
//        {
//            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Device not Found\r\n");
//        }
//        
//    }
//    else
//    {
//        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid Command\r\n");
//    }
//    
//}
//
//static void processGetDeviceInfoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
//{
//    const void* cmdIoParam = pCmdIO->cmdIoParam;
//    if(argc == 2)
//    {
//        uint8_t index = atoi(argv[1]);
//        app_printf("Type-%d Name - %s\r\n", demoDevices[index].devType, demoDevices[index].devName);
//    }
//    else
//    {
//        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid Command\r\n");
//    }
//}
//
//static void processGetDeviceAddrCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
//{
//    const void* cmdIoParam = pCmdIO->cmdIoParam;
//    if(argc == 2)
//    {
//        uint8_t index = atoi(argv[1]);
//        char string[OT_IP6_ADDRESS_STRING_SIZE];
//        otIp6AddressToString(&demoDevices[index].devAddr, string, OT_IP6_ADDRESS_STRING_SIZE);
//        app_printf("Device Addr - %s\r\n", string);
//    }
//    else
//    {
//        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid Command\r\n");
//    }
//}

static void processHelpCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    uint8_t index;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Commands: \r\n");
    for (index = 0; index < (sizeof(gatewayCmdsTbl)/sizeof(*gatewayCmdsTbl)); index++)
    {
        app_printf("%s\r\n", gatewayCmdsTbl[index].cmdStr);
    }
}

static void processAutoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
	(*pCmdIO->pCmdApi->msg)(cmdIoParam, "auto\r\n");
}

static void processLightCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
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
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Invalid Command\r\n");
    }
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
    app_printf("Temp-%0.1f\r", tempReport->temperature);
}

static void processthermoHVACSetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    devTypeThermostatHVACSet_t *tempHVACSet = (devTypeThermostatHVACSet_t *)demoCommand.msg;
    tempHVACSet->setPoint = (float)(atoi(argv[2]) / 10);
    tempHVACSet->onOffStatus = atoi(argv[3]);

    demoCommand.msgType = MSG_TYPE_THERMO_HVAC_SET;
    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 4 + sizeof(devTypeThermostatHVACSet_t), (uint8_t *)&demoCommand);
}

static void processthermoHVACGetCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    demoCommand.msgType = MSG_TYPE_THERMO_HVAC_GET;
    threadUdpSend((otIp6Address *)&demoDevices[atoi(argv[1])].devAddr, 5, (uint8_t *)&demoCommand);
}

static void processGetDeviceInfoCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
	app_printf("Type-%d Name - %s\r\n", demoDevices[atoi(argv[1])].devType, demoDevices[atoi(argv[1])].devName);
}

static void processGetDeviceAddrCmd(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    char string[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&demoDevices[atoi(argv[1])].devAddr, string, OT_IP6_ADDRESS_STRING_SIZE);
	app_printf("Device Addr - %s\r\n", string);
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
    //app_printf("Unicast Address :\r\n");
    
//    char string[OT_IP6_ADDRESS_STRING_SIZE];
//    otIp6AddressToString(&(unicastAddrs->mAddress), string, OT_IP6_ADDRESS_STRING_SIZE);
//    app_printf("Unicast Address :\r\n%s\r\n", string);

    for (const otNetifAddress *addr = unicastAddrs; addr; addr = addr->mNext)
    {
        char string[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&(addr->mAddress), string, OT_IP6_ADDRESS_STRING_SIZE);
        //app_printf("%s\r\n\n", string);
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

            threadDevice.devType = DEVICE_TYPE_GATEWAY;
            threadDevice.devNameSize = sizeof(DEMO_DEVICE_NAME);
            memcpy(&threadDevice.devName, DEMO_DEVICE_NAME, sizeof(DEMO_DEVICE_NAME));

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
                    threadSendIPAddr();
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


/*******************************************************************************
 End of File
 */
