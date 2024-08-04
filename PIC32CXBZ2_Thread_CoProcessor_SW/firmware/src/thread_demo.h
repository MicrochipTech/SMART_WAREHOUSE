/*******************************************************************************
 System Tasks File

  File Name:
    thread_demo.c

  Summary:
    This file contains source code necessary to thread demo application.

  Description:
    This file contains source code necessary to thread demo application.
    
  Remarks:
    
 *******************************************************************************/

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
#ifndef _THREAD_DEMO_H
#define _THREAD_DEMO_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
#include "definitions.h"
#include "openthread/ip6.h"
#include "configuration.h"
#include "openthread/dataset.h"
#include "openthread/dataset_ftd.h"
#include "openthread/instance.h"
#include "openthread/thread.h"

#include "openthread/thread_ftd.h"

// *****************************************************************************
// *****************************************************************************
#define app_printf(fmt, ...)                 SYS_CONSOLE_Print(SYS_CONSOLE_DEFAULT_INSTANCE, fmt, ##__VA_ARGS__)

#define UDP_PORT_NO    2345

#define DEVICE_AS_LEADER         1

#define APP_BROADCAST_TIMER_INTERVAL_MS   10000

#define PANID           0xDADA
#define EXD_PANID       {0x4D,0x43,0x48,0x50,0x54,0x45,0x43,0x48}
#define NWK_NAME        "MCHP_THREAD" // max charin nwk name <= 16
#define CHANNEL         21LU
#define NW_KEY          {0x73,0x6d,0x61,0x72,0x74,0x77,0x61,0x72,0x65,0x68,0x6f,0x75,0x73,0x65,0x32,0x34}
#define ML_PREFIX       {0xfd,0x00,0x00,0x00,0xfb,0x01,0x00,0x01}

#define MAX_DEMO_NAME_SIZE      (20)
#define MAX_DEMO_MSG_SIZE       (30)

#define TOTAL_DEMO_DEVICES      (10)

typedef struct demoDevice_t{
	otIp6Address devAddr; ///< The IPv6 address.
    bool isAvailable;
    uint8_t devType;
    uint8_t devNameSize;
    uint8_t devName[MAX_DEMO_NAME_SIZE];
    uint8_t devMsgSize;
    uint32_t devMsg[(MAX_DEMO_MSG_SIZE / 4) + 1];
} demoDevice_t;

typedef struct devMsgType_t {
    uint8_t msgType;
    uint8_t reserved1;
    uint16_t reserved2;
    uint32_t msg[(MAX_DEMO_MSG_SIZE / 4) + 1];
} devMsgType_t;


#define DEVICE_TYPE_GATEWAY             (0x1U)
#define DEVICE_TYPE_LIGHT               (0x2U)
#define DEVICE_TYPE_THERMOSTAT_SENSOR   (0x3U)
#define DEVICE_TYPE_THERMOSTAT_HVAC     (0x4U)
#define DEVICE_TYPE_SOLAR               (0x5U)
#define DEVICE_TYPE_ACCESS_CONTROL      (0x6U)


#define MSG_TYPE_GATEWAY_DISCOVER_REQ  (0x01)
#define MSG_TYPE_GATEWAY_DISCOVER_RESP (0x02)

#define MSG_TYPE_LIGHT_SET             (0x10)
#define MSG_TYPE_LIGHT_GET             (0x11)
#define MSG_TYPE_LIGHT_REPORT          (0x12)

#define MSG_TYPE_THERMO_SENSOR_SET     (0x20)
#define MSG_TYPE_THERMO_SENSOR_GET     (0x21)
#define MSG_TYPE_THERMO_SENSOR_REPORT  (0x22)

#define MSG_TYPE_THERMO_HVAC_SET       (0x30)
#define MSG_TYPE_THERMO_HVAC_GET       (0x31)
#define MSG_TYPE_THERMO_HVAC_REPORT    (0x32)

#define MSG_TYPE_SOLAR_SET             (0x40)
#define MSG_TYPE_SOLAR_GET             (0x41)
#define MSG_TYPE_SOLAR_REPORT          (0x42)

#define MSG_TYPE_ACCESS_CONTROL_SET    (0x50)
#define MSG_TYPE_ACCESS_CONTROL_GET    (0x51)
#define MSG_TYPE_ACCESS_CONTROL_REPORT (0x52)

typedef struct devDetails_t {
    uint8_t devType;
    uint8_t devNameSize;
    uint8_t devName[MAX_DEMO_NAME_SIZE];
} devDetails_t;

typedef struct devTypeGateway_t {
    otIp6Address gatewayAddr;
}devTypeGateway_t;

typedef struct devTypeRGBLight_t {
    bool onOff;
    uint8_t hue;
    uint8_t saturation;
    uint8_t level;
}devTypeRGBLight_t;

typedef struct devTypeThermostatSensorSet_t {
    otIp6Address reportedThermostatHVAC;
    uint8_t reportInterval;
}devTypeThermostatSensorSet_t;

typedef struct devTypeThermostatSensorReport_t {
    float temperature;
}devTypeThermostatSensorReport_t;

typedef struct devTypeThermostatHVACSet_t {
    float setPoint;
}devTypeThermostatHVACSet_t;

typedef struct devTypeThermostatHVACReport_t {
    float setPoint;
    bool onOffStatus;
}devTypeThermostatHVACReport_t;

typedef struct devTypeSolarSet_t {
    uint8_t position;
}devTypeSolarSet_t;

typedef struct devTypeSolarReport_t {
    float voltage;
    float lightIntensity;
}devTypeSolarReport_t;

typedef struct devTypeAccessControlSet_t {
    bool garageDoor;
}devTypeAccessControlSet_t;

typedef struct devTypeAccessControlReport_t {
    bool garageDoor;
}devTypeAccessControlReport_t;


void threadAppinit();
void threadConfigNwParameters();
void threadNwStart();
void threadHandleStateChange(void);
void threadTmrCb();
void threadInitData();
void printIpv6Address(void);
void threadSendPeriodicMsg(void);

void threadReceiveData(const otMessageInfo *aMessageInfo, uint16_t length, uint8_t *msgPayload);
void addDemoDevice(const otIp6Address *addr, uint8_t nameSize, uint8_t *name, uint8_t type);
void updateDemoStatus(const otIp6Address *devAddr, void *msg);

void threadUdpOpen();
void threadUdpSend(otIp6Address *mPeerAddr, uint8_t msgLen, uint8_t* msg);
void threadUdpBind();
void threadUdpReceiveCb(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);

#endif //_THREAD_DEMO_H
/*******************************************************************************
 End of File
 */

