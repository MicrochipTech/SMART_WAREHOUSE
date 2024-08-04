// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
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
#include "app_timer/app_timer.h"
#include "config/default/peripheral/sercom/i2c_master/plib_sercom2_i2c_master.h"
#include "stdio.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************


#define LED_BLINK_TIME_MS               (150)

otIp6Address gatewayAddr;
extern devDetails_t threadDevice;
extern otInstance *instance;
otUdpSocket aSocket;

extern float SP_Volt;
extern float SP_ldr;
extern uint8_t sp_position;
uint32_t wbz451_silicon_revision = 0x00;

/* The timer created for LED that blinks when it receives the data from the Leader */
static TimerHandle_t Data_sent_LED_Timer_Handle = NULL;
/* The timer created for LED that blinks when it sends data to the Leader*/
static TimerHandle_t Data_receive_LED_Timer_Handle = NULL;

extern volatile uint16_t temperature_value;

static void Data_sent_LED_Timer_Callback(TimerHandle_t xTimer)
{
//    RGB_LED_GREEN_Off();    
    /* Keep compiler happy. */
     (void)xTimer;    
}
static void Data_receive_LED_Timer_Callback(TimerHandle_t xTimer)
{
    USER_LED_On();   //off
    /* Keep compiler happy. */
     (void)xTimer;    
}


// *****************************************************************************
// *****************************************************************************
float temp_demo_value = 25.0;
devMsgType_t demoMsg;

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

void adchs_ch1_callback(ADCHS_CHANNEL_NUM channel,uintptr_t context)
{
    SP_Volt = ((ADCHS_ChannelResultGet(ADCHS_CH5)*3.3)/4095)*3.0;      //voltage = (ADC value * reference voltage) / 4095     
    app_printf(" SP Channel 1 PB1:%.2f Volt\r\n",SP_Volt);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************


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
            
            wbz451_silicon_revision = 	DSU_REGS->DSU_DID;	
            SYS_CONSOLE_PRINT("\n\r[Device DID] 0x%x  \n\r", (DSU_REGS->DSU_DID)); 

            if((wbz451_silicon_revision >> 28) ==  (0x00)) // A0 silicon
            {
              app_printf("!!! Use A2 version of the board for this application !!!\r\n");
              while((wbz451_silicon_revision >> 28) ==  (0x00)){                  
                }
            }	

            threadAppinit();
            app_printf("App_Log: Thread Network is getting initialized\n");
            
            threadDevice.devType = DEVICE_TYPE_SOLAR;
            threadDevice.devNameSize = sizeof(DEMO_DEVICE_NAME);
            memcpy(&threadDevice.devName, DEMO_DEVICE_NAME, sizeof(DEMO_DEVICE_NAME));

            TCC0_PWMStart();
            APP_TIMER_SetTimer(APP_TIMER_ID_0,APP_TIMER_5S,true);
            APP_TIMER_SetTimer(APP_TIMER_ID_2,APP_TIMER_5S,true);
            ADCHS_CallbackRegister(ADCHS_CH5,adchs_ch1_callback,0);
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
                        ADCHS_ChannelConversionStart(ADCHS_CH5);
                }  
                else if(p_appMsg->msgId == APP_I2C_LDR)
                {
                    SERCOM2_I2C_Read(ldr_add, (uint8_t *)ldr_data, 2);   // Convert little-endian bytes to a 16-bit unsigned integer
                    uint16_t result = (uint16_t)(((uint16_t)ldr_data[0] << 8) | ldr_data[1]);
                    SP_ldr = result / 1.2;     // Divide the result by 1.2               
                    app_printf("Light: %.2f lx\r\n",SP_ldr);                    
                }                
                else if(p_appMsg->msgId == APP_PWM_SERVO)
                {
                    switch(sp_position)
                    {
                        case 0:
                        {
                            app_printf("M1\r\n");
                            
                            if(dutyValue2 > 0x3CU){                                
                                for(dutyValue2; dutyValue2 >= 0x3CU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x3CU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue1 > 0x23U){                                
                                for(dutyValue1; dutyValue1 >= 0x23; --dutyValue1) {      //0x23 0x3E
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue1; dutyValue1 <= 0x23; ++dutyValue1) {     //0x23  0x3E
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue2 > 0x5AU){                                
                                for(dutyValue2; dutyValue2 >= 0x5AU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x5AU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                        }
                        break;
                        
                        case 1:
                        {
                            app_printf("M2\r\n");
                            
                            if(dutyValue2 > 0x3CU){                                
                                for(dutyValue2; dutyValue2 >= 0x3CU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x3CU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue1 > 0x60U){                                
                                for(dutyValue1; dutyValue1 >= 0x60U; --dutyValue1) {    //0x60U 0x5CU
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue1; dutyValue1 <= 0x60U; ++dutyValue1) {    //0x60U 0x5CU
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                        }
                        break;
                        
                        case 2:
                        {
                            app_printf("M3\r\n");
                            
                            if(dutyValue2 > 0x3CU){                                
                                for(dutyValue2; dutyValue2 >= 0x3CU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x3CU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue1 > 0x9DU){                                
                                for(dutyValue1; dutyValue1 >= 0x9DU; --dutyValue1) {    //0x9DU 0x7BU
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue1; dutyValue1 <= 0x9DU; ++dutyValue1) {    //0x9DU 0x7BU
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, dutyValue1);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            
                            if(dutyValue2 > 0x5AU){                                
                                for(dutyValue2; dutyValue2 >= 0x5AU; --dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
                            else{
                                for(dutyValue2; dutyValue2 <= 0x5AU; ++dutyValue2) {
                                
                                    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, dutyValue2);
                                    vTaskDelay(300 / portTICK_PERIOD_MS);
                                }  
                            }
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

void threadReceiveData(const otMessageInfo *aMessageInfo, uint16_t length, uint8_t *msgPayload)
{
    char string[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&(aMessageInfo->mPeerAddr), string, OT_IP6_ADDRESS_STRING_SIZE);
     
    
    USER_LED_Off();
    if( xTimerIsTimerActive( Data_receive_LED_Timer_Handle ) != pdFALSE )
    {
        /* xTimer is active, do something. */
        (void)xTimerStop( Data_receive_LED_Timer_Handle, pdMS_TO_TICKS(0) );
    }
    (void)xTimerStart(Data_receive_LED_Timer_Handle,pdMS_TO_TICKS(0));
    
    devMsgType_t *rxMsg;
    rxMsg = (devMsgType_t *)msgPayload;
    
//    app_printf("App_Log: UDP Received from [%s] len:[%d] type:[%d]\r\n", string, length, rxMsg->msgType);     
    
    
    if(MSG_TYPE_GATEWAY_DISCOVER_REQ == rxMsg->msgType)
    {
        memcpy(&gatewayAddr, rxMsg->msg, OT_IP6_ADDRESS_SIZE);
        demoMsg.msgType = MSG_TYPE_GATEWAY_DISCOVER_RESP;
        memcpy(&demoMsg.msg, &threadDevice, sizeof(devDetails_t));
        threadUdpSend(&gatewayAddr, sizeof(devMsgType_t), (uint8_t *)&demoMsg);
//        app_printf("App Log: DiscReq\r\n");
    }
    else if(MSG_TYPE_THERMO_SENSOR_GET == rxMsg->msgType)
    {
        devTypeThermostatSensorReport_t tempReport;
        //tempReport.temperature = temperature_value;
        if(temp_demo_value > 35.0)
        {
            temp_demo_value = 25.0;
        }
        tempReport.temperature = temp_demo_value;
        temp_demo_value += 0.1;
        demoMsg.msgType = MSG_TYPE_THERMO_SENSOR_REPORT;
        memcpy(&demoMsg.msg, &tempReport, sizeof(devTypeThermostatSensorReport_t));
        threadUdpSend(&gatewayAddr, 4 + sizeof(devTypeThermostatSensorReport_t), (uint8_t *)&demoMsg);
        app_printf("App Log:Temp Report\r\n");
    }
    else if(MSG_TYPE_SOLAR_GET == rxMsg->msgType)
    {
        devTypeSolarReport_t solarReport;
        solarReport.voltage = SP_Volt;
        solarReport.lightIntensity = SP_ldr;

        demoMsg.msgType = MSG_TYPE_SOLAR_REPORT;
        memcpy(&demoMsg.msg, &solarReport, sizeof(devTypeSolarReport_t));
        threadUdpSend(&gatewayAddr, 4 + sizeof(devTypeSolarReport_t), (uint8_t *)&demoMsg);
        app_printf("App Log:Solar Get\r\n");
    } 
    else if(MSG_TYPE_SOLAR_SET == rxMsg->msgType)
    {
        APP_Msg_T    appMsg;
        
        devTypeSolarSet_t *setSolar = (devTypeSolarSet_t *)rxMsg->msg;
        sp_position = setSolar->position;
        app_printf("App Log:Solar Set\r\n");
        
        appMsg.msgId = APP_PWM_SERVO;
        OSAL_QUEUE_Send(&appData.appQueue, &appMsg, 0);
    } 
    
//    else if(MSG_TYPE_ROBO_ARM_REPORT == rxMsg->msgType)
//    {
////        devTypeThermostatSensorReport_t *sensor;
////        sensor = (devTypeThermostatSensorReport_t *)rxMsg->msg;
//        app_printf("Recv Robo\r\n");
//    }
//    else if(MSG_TYPE_TEMP_SENSOR_REPORT == rxMsg->msgType)
//    {
////        uint16_t tempSensor;
////        tempSensor = (devTypeTemperatureSensorReport_t )rxMsg->msg;
//        app_printf("Recv Temperature: \r\n");//, tempSensor.temperature);
//    }
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

//void otUdpReceiveCb(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
//{
//    APP_Msg_T appMsg_otUdpReceiveCb;
//    
//    uint16_t len = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
//    uint8_t output_buffer[len+1];
//    
//    otMessageRead(aMessage,otMessageGetOffset(aMessage),output_buffer,len);
//    output_buffer[len] = '\0';
//    
//    
//    appMsg_otUdpReceiveCb.msgId = APP_MSG_OT_RECV_CB;
//    
//    otUdpReceiveData_t *otUdpReceiveData;
//    otUdpReceiveData = (otUdpReceiveData_t *)&appMsg_otUdpReceiveCb;
//        
//    memcpy(otUdpReceiveData->messageInfo, aMessageInfo, sizeof(otMessageInfo));
//    otUdpReceiveData->length = (uint8_t)len;
//    memcpy(otUdpReceiveData->msgPayload, &output_buffer, len);
//
//    OSAL_QUEUE_Send(&appData.appQueue, &appMsg_otUdpReceiveCb, 0);
//    
//}

void threadUdpOpen()
{
   otError err;
   app_printf("App_log: UDP Open\n");
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
            app_printf("App_Err: UDP Message Add fail\n");
            break;
        }
        
        err = otUdpSend(instance,&aSocket,udp_msg,&msgInfo);
        if(err != OT_ERROR_NONE)
        {
            app_printf("App_Err: UDP Send fail\n");
            break;
        }
        app_printf("App_Log: UDP Sent data: %d\r\n",err);
//        RGB_LED_GREEN_On();
        if( xTimerIsTimerActive( Data_sent_LED_Timer_Handle ) != pdFALSE )
        {
            /* xTimer is active, do something. */
            (void)xTimerStop( Data_sent_LED_Timer_Handle, pdMS_TO_TICKS(0) );
        }
        (void)xTimerStart(Data_sent_LED_Timer_Handle,pdMS_TO_TICKS(0));

    }while(false);
    
}

//void threadUdpSendAddress(otIp6Address mPeerAddr)
//{
//    otError err = OT_ERROR_NONE;
//    otMessageInfo msgInfo;
//    memset(&msgInfo,0,sizeof(msgInfo));
//    memcpy(&msgInfo.mPeerAddr, &mPeerAddr, OT_IP6_ADDRESS_SIZE);
//    msgInfo.mPeerPort = UDP_PORT_NO;
//    
//    do {
//        otMessage *udp_msg = otUdpNewMessage(instance,NULL);
//        err = otMessageAppend(udp_msg,msg,(uint16_t)strlen(msg));
//        if(err != OT_ERROR_NONE)
//        {
//            app_printf("App_Err: UDP Message Add fail\n");
//            break;
//        }
//        
//        err = otUdpSend(instance,&aSocket,udp_msg,&msgInfo);
//        if(err != OT_ERROR_NONE)
//        {
//            app_printf("App_Err: UDP Send fail\n");
//            break;
//        }
//        app_printf("App_Log: UDP Sent data: %s\n",msg);
//        RGB_LED_GREEN_On();
//        if( xTimerIsTimerActive( Data_sent_LED_Timer_Handle ) != pdFALSE )
//        {
//            /* xTimer is active, do something. */
//            (void)xTimerStop( Data_sent_LED_Timer_Handle, pdMS_TO_TICKS(0) );
//        }
//        (void)xTimerStart(Data_sent_LED_Timer_Handle,pdMS_TO_TICKS(0));
//        
//    }while(false);
//}

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
        app_printf("App_Log: UDP Listening on port %d\n",UDP_PORT_NO);
   }while(false);
}

/*******************************************************************************
 End of File
 */
