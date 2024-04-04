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
//#include "app_tempHum13/app_temphum13.h"

#include "app_timer/app_timer.h"
#include "config/default/peripheral/sercom/i2c_master/plib_sercom2_i2c_master.h"
#include "stdio.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define APP_TEMP_TIMER_INTERVAL_MS     15000


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

uint8_t sp_position = 0;
float SP_Volt = 0.00;
float SP_ldr = 0.00;

uint8_t dutyValue1 = 0x23U;
uint8_t dutyValue2 = 0x3CU; 
uint16_t ldr_add = 0x23U;
uint8_t H_res = 0x10U; // Continuously H-Resolution Mode
uint8_t ldr_data[2];


//static TimerHandle_t tempTimerHandle = NULL;
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
//volatile uint16_t temperature_value;

void adchs_ch1_callback(ADCHS_CHANNEL_NUM channel,uintptr_t context)
{
    SP_Volt = ((ADCHS_ChannelResultGet(ADCHS_CH1)*3.3)/4095)*3.0;      //voltage = (ADC value * reference voltage) / 4095     
    app_printf(" SP Channel 1 PB5:%.2f Volt\r\n",SP_Volt);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

//void tempTmrCb(TimerHandle_t pxTimer)
//{
//    temperature_value = temphum13_get_temperature();
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
            
            threadDevice.devType = DEVICE_TYPE_SOLAR;
            threadDevice.devNameSize = sizeof(DEMO_DEVICE_NAME);
            memcpy(&threadDevice.devName, DEMO_DEVICE_NAME, sizeof(DEMO_DEVICE_NAME));

//            tempTimerHandle = xTimerCreate("temp app tmr", (APP_TEMP_TIMER_INTERVAL_MS / portTICK_PERIOD_MS), true, ( void * ) 0, tempTmrCb);
//            xTimerStart(tempTimerHandle, 0);
//            if(tempTimerHandle == NULL)
//            {
//                app_printf("App_Err: App Timer creation failed\n");
//            }
            TCC0_PWMStart();
            APP_TIMER_SetTimer(APP_TIMER_ID_0,APP_TIMER_5S,true);
            APP_TIMER_SetTimer(APP_TIMER_ID_1,APP_TIMER_5S,true);
            APP_TIMER_SetTimer(APP_TIMER_ID_2,APP_TIMER_5S,true);
            ADCHS_CallbackRegister(ADCHS_CH1,adchs_ch1_callback,0);
            SERCOM2_I2C_Write(ldr_add,&H_res,1); // CONFIGURE THE LDR
            
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
                else if(p_appMsg->msgId == APP_ADC_SP)
                {
                        ADCHS_ChannelConversionStart(ADCHS_CH1);
                }  
                else if(p_appMsg->msgId == APP_I2C_LDR)
                {
                    SERCOM2_I2C_Read(ldr_add, (uint8_t *)ldr_data, 2);
                    // Convert little-endian bytes to a 16-bit unsigned integer
                    uint16_t result = (uint16_t)(((uint16_t)ldr_data[0] << 8) | ldr_data[1]);
                    // Divide the result by 1.2
                    SP_ldr = result / 1.2;                    
                    app_printf("Light: %.2f lx\r\n",SP_ldr);                    
                }                
                else if(p_appMsg->msgId == APP_PWM_SERVO)
                {
                    switch(sp_position)
                    {
                        case 0:
                        {
                            printf("M1\r\n");
                            if(dutyValue1 > 0x23U){                                
                                for(dutyValue1; dutyValue1 >= 0x23; --dutyValue1) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue1; dutyValue1 <= 0x23; ++dutyValue1) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue2 > 0x5AU){                                
                                for(dutyValue2; dutyValue2 >= 0x5AU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x5AU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }                       
                                                       
//                            sp_position=1;
                        }
                        break;
                        
                        case 1:
                        {
                            printf("M2\r\n");
                            
                            if(dutyValue1 > 0x60U){                                
                                for(dutyValue1; dutyValue1 >= 0x60U; --dutyValue1) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue1; dutyValue1 <= 0x60U; ++dutyValue1) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue2 > 0x3CU){                                
                                for(dutyValue2; dutyValue2 >= 0x3CU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x3CU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
//                            sp_position=2;
                        }
                        break;
                        
                        case 2:
                        {
                            printf("M3\r\n");
                            if(dutyValue1 > 0x9DU){                                
                                for(dutyValue1; dutyValue1 >= 0x9DU; --dutyValue1) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue1; dutyValue1 <= 0x9DU; ++dutyValue1) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue2 > 0x5AU){                                
                                for(dutyValue2; dutyValue2 >= 0x5AU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x5AU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(40 / portTICK_PERIOD_MS);
                                }  
                            }
//                            sp_position=0;
                        }
                        break;                        
                    }                    
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
