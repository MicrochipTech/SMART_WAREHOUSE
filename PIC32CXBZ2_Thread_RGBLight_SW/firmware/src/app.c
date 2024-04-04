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
#include "timers.h"
#include "thread_demo.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define APP_TEMP_TIMER_INTERVAL_MS     5000


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

static TimerHandle_t tempTimerHandle = NULL;
void tempTmrCb(TimerHandle_t pxTimer);
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

devDetails_t threadDevice;
volatile uint16_t temperature_value;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

//void tempTmrCb(TimerHandle_t pxTimer)
//{
//    //temperature_value = temphum13_get_temperature();
//}

void printIpv6Address(void)
{
//    APP_Msg_T    appMsg;

    const otNetifAddress *unicastAddrs = otIp6GetUnicastAddresses(instance);
    app_printf("Unicast Address :\r\n");
    
//    char string[OT_IP6_ADDRESS_STRING_SIZE];
//    otIp6AddressToString(&(unicastAddrs->mAddress), string, OT_IP6_ADDRESS_STRING_SIZE);
//    app_printf("Unicast Address :\r\n%s\r\n", string);

    for (const otNetifAddress *addr = unicastAddrs; addr; addr = addr->mNext)
    {
        char string[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&(addr->mAddress), string, OT_IP6_ADDRESS_STRING_SIZE);
        app_printf("%s\r\n", string);
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
            app_printf("App_Log: Thread Network is getting initialized\n");
            
            threadDevice.devType = DEVICE_TYPE_LIGHT;
            threadDevice.devNameSize = sizeof(DEMO_DEVICE_NAME);
            memcpy(&threadDevice.devName, DEMO_DEVICE_NAME, sizeof(DEMO_DEVICE_NAME));

            //tempTimerHandle = xTimerCreate("temp app tmr", (APP_TEMP_TIMER_INTERVAL_MS / portTICK_PERIOD_MS), true, ( void * ) 0, tempTmrCb);
            //xTimerStart(tempTimerHandle, 0);
            if(tempTimerHandle == NULL)
            {
                app_printf("App_Err: App Timer creation failed\n");
            }
                
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
