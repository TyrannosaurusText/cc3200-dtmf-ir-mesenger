//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Interrupt Demo Application
// Application Overview - The objective of this application is to showcase
//                        interrupt preemption and tail-chaining capabilities.
//                        Nested interrupts are synthesized when the interrupts
//                        have the same priority, increasing priorities and
//                        decreasing priorities. With increasing priorities,
//                        preemption will occur; in the other two cases tail-
//                        chaining will occur.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Interrupt_Demo_Application
// or
// docs\examples\CC32xx_Interrupt_Demo_Application.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup InterruptsReferenceApp
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>

// Simplelink includes
#include "simplelink.h"

// Driverlib includes
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_timer.h"
#include "uart.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "timer.h"
#include "timer_if.h"
#include "interrupt.h"
#include "gpio.h"
#include "spi.h"
// Common interface includes
//#include "systick_if.h"
#include "uart_if.h"
#include <stdlib.h>
#include <string.h>
#include "upng.h"
#include "commons.h"
#include "pinmux.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "templates.h"

#define SYSCLK               80000000
#define UART_PRINT           Report

// This is the time we delay inside the A0 timer interrupt handler.
#define SLOW_TIMER_DELAY_uS 2000

// Interrupt priorities used within the test.
#define LOW_PRIORITY  0xFF
#define HIGH_PRIORITY 0x00
#define MIDDLE_PRIORITY 0x80
//macros for OLED
#define TR_BUFF_SIZE     100
#define SPI_IF_BIT_RATE  500000
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//definitions for lab5
#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "cc3200"
#define APPLICATION_VERSION     "1.1.1.EEC.Winter2017"
#define SERVER_NAME             "api.imgur.com"
#define GOOGLE_DST_PORT          443

#define SL_SSL_CA_CERT "/cert/rootCA.der"
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                15    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2017  /* Current year */
#define HOUR                9    /* Time - hours */
#define MINUTE              12    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /3/upload/?image=https://memegen.link/" // POSTHEADER + template + / + TOP + / + BOTTOM + / + POSTHEADER2
#define POSTHEADER2 ".jpg HTTP/1.1\n\r"
#define HOSTHEADER "Host: api.imgur.com\r\n"
#define HOSTHEADER2 "Host: i.imgur.com\r\n"
#define CHEADER "Accept: */*\r\n"
#define CTHEADER "Authorization: Bearer aba671eae9e5f5901e94ca2b7a0687006a46eec3\r\n"
#define CLHEADER1 "Content-Length: 0\r\n"
#define CLHEADER2 "Content-Type: application/x-www-form-urlencoded\r\n\r\n"
#define GETHEADER "GET /7keEh92.png"
/*
 * If refresh token is expired a new refresh token must be obtained by going to https://api.imgur.com/oauth2/authorize?client_id=c601fb6d8149458&response_type=token
 *      You will be redirected to a new URL, copy the value in the redirected URL refresh_token=REFRESH_TOKEN
 *      There's no workaround for this since this code does not self maintain
 *
 *
 */

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
//globals for lab5
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
static char link[50];
// Globals used for OLED
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];

// Globals used to save and restore interrupt settings.
static unsigned long timeBuffer[80]; // time differences between signals
static unsigned long numKeyBuffer[80]; //buffer is larger than necessary to avoid overflow
static unsigned long enterOrDelete[80]; //buffer is larger than necessary to avoid overflow

 int i = 0;
 int j = 0;
 int skip = 0;
 int _keyCode = -1;
 int dataBits = 0;
 int successfulKeyRead = 0; //additional initializations necessary for functioning
 int _keyPressed= 0;
static int ulStatus; //interrupt status clearing
static int messagePosition; // current position in messageTX
static char messageTX[147]; // stores message to transmit
static int messageLoop = 0; // for multitap
static int pos = 0; // current position in signalTimeBuffer
const int buffersize = 40; //adjustable buffer size
static unsigned long signalTimeBuffer[40]; // buffer containing the time stamp for rising and falling edges
static int timerBlocking = 1; //prevents timers from doing anything
static int previousKeyCode = -1; //used for multitap texting
static unsigned int currSignalTime, prevSignalTime; //used to prevent button holds from registering

static int size;
static char* p;
static int indexer = 0;
static char* top;
static char* bottom;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
static int http_get(int);
long printErrConvenience(char * msg, long retVal);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %ui , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);



    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}
const int ENTER[] = {1,3,1,2,1,3,1,4,1,3,1,1,1,2,1}; //size 15
const int DELETE[] = {1,3,1,2,1,3,1,3,1,2,1,4,1,3,1};

//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP,uiCipher = SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA;
    long lRetVal = -1;
    int iSockID;
    int x = strlen((char*)g_Host);
    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }
    else {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }

    return iSockID;
}



long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    return retVal;
}



int connectToAccessPoint() {
    long lRetVal = -1;

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

/*
 * Clears signal buffer at certain intervals
 */
static void TimerA1IntHandler()
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    if(pos!=0 && timerBlocking == 0)
        pos = buffersize;
}

/**
 * Long pause timer (1s)
 */
static void TimerA2IntHandler()
{
    Timer_IF_InterruptClear(TIMERA2_BASE);
    if(timerBlocking == 0 && messageTX[messagePosition] != 0 && messagePosition != 146) //only if not reading a key, char at 0 isn't empty and not at the max buffer size
    {
        previousKeyCode = -1;
        drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, BLACK, 1);
        messagePosition++;
        messageLoop = 0;
        drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, WHITE, 1);
    }
}

/*
 * Triggers on rising and falling edges, stores current time
 */
static void
GPIOIntHandler()
{

    ulStatus = MAP_GPIOIntStatus(GPIOA3_BASE, false);
    MAP_GPIOIntClear(GPIOA3_BASE, ulStatus);
    MAP_IntPendClear(INT_GPIOA3);
    signalTimeBuffer[pos] = Timer_IF_GetCount(TIMERA0_BASE, TIMER_A);
    pos++;
    if(pos > buffersize) //full buffer
    {
        MAP_GPIOIntDisable(GPIOA3_BASE,0x10);
    }
}
//clears top screen
void clearRXScreen(int messageLength)
{
    int i;
    for(i = 0; i < messageLength; i++) //clears top screen
    {
        drawChar((i%21)*6,8*(i/21),0, BLUE, BLACK, 1);
    }
}
//clears bottom screen
void clearTXScreen(int messageLength)
{
    int i;
    for(i = 0; i < messageLength+1; i++) //clears top screen
    {
        messageTX[i] = 0;
        drawChar((i%21)*6,68+8*(i/21),0, BLUE, BLACK, 1);
    }
    messagePosition = -1;
}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function handling the interrupt example
//!
//! \param  none
//!
//! \return none
//! 010100101010101010101010101
//*****************************************************************************
/*
 * Turns on the board
 * Enable SPI communication and turns on the OLED
 */
void init()
{
    //UART initialization
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                            UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                    UART_CONFIG_PAR_NONE));
    MAP_UARTEnable(UARTA1_BASE);
    MAP_UARTFIFOEnable(UARTA1_BASE);

    //SPI initialization
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS |
                            SPI_4PIN_MODE |
                            SPI_TURBO_OFF |
                            SPI_CS_ACTIVEHIGH |
                            SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);
    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
                    SPI_CS_ENABLE|SPI_CS_DISABLE);

    //timers
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, (unsigned long)-1); // used to get current time

    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, TimerA1IntHandler); // timer for checking if buffer is empty or not

    Timer_IF_Init(PRCM_TIMERA2, TIMERA2_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA2_BASE, TIMER_A, TimerA2IntHandler); // message incrementing timer


    Adafruit_Init(); //turns on adafruit

    InitTerm();
    ClearTerm();
    UART_PRINT("Hello world!\n\r");
    fillScreen(BLACK);
    MAP_GPIOIntTypeSet(GPIOA3_BASE,0x10,GPIO_BOTH_EDGES);
    MAP_GPIOIntRegister(GPIOA3_BASE,GPIOIntHandler);


    /*clear the message buffer*/
    for(i = 0; i < 146; i++)
    {
        messageTX[i] = 0;
    }
    messagePosition = 0;

    drawLine(0,64,128,64,BLUE); //separates the screen


    //interrupts enabled before entering loop
    MAP_GPIOIntClear(GPIOA3_BASE,0x10);
    MAP_GPIOIntEnable(GPIOA3_BASE,0x10);
    drawChar(0,68, messageTX[messagePosition], BLUE, WHITE, 1);
    Timer_IF_Start(TIMERA1_BASE, TIMER_A, 8000000); // 100 millisecond timer
    Timer_IF_Start(TIMERA2_BASE, TIMER_A, 80000000); //1 seconds
}
/*
 * Determines which letter is printed after a key is pressed.
 * param - KeyCode 0-9 -
 * returns the corresponding multitap character
 */
char keyCodeParse(int keyCode)
{
    switch(keyCode)
    {
    case 0:
    {

        if(messageLoop == 2)
        {
            messageLoop = 0;
            return '-';
        }
        return '0';
    }
    //break;
    case 1:
    {
        return '1';
    }
    //break;
    case 2:
    {
        if(messageLoop == 3)
        {
            messageLoop = -1;
            return '2';
        }
        return 'A'+ messageLoop;
    }
    //break;
    case 3:
    {
        if(messageLoop == 3)
        {
            messageLoop = -1;
            return '3';
        }
        return 'D' + messageLoop;
    }
    //break;
    case 4:
    {
        if(messageLoop == 3)
        {
            messageLoop = -1;
            return '4';
        }
        return 'G' + messageLoop;
    }

    //break;
    case 5:
    {
        if(messageLoop == 3)
        {
            messageLoop = -1;
            return '5';
        }
        return 'J' + messageLoop;
    }
    //break;
    case 6:
    {
        if(messageLoop == 3)
        {
            messageLoop = -1;
            return '6';
        }
        return 'M' + messageLoop;
    }
    //break;
    case 7:
    {
        if(messageLoop == 4)
        {
            messageLoop = -1;
            return '7';
        }
        return 'P' + messageLoop;
    }
    //break;
    case 8:
    {
        if(messageLoop == 3)
        {
            messageLoop = -1;
            return '8';
        }
        return 'T' + messageLoop;
    }
    //break;
    case 9:
    {
        if(messageLoop == 4)
        {
            messageLoop = -1;
            return '9';
        }
        return 'W' + messageLoop;
    }
        //break;
    }
    return 0;
}


/*
 * After pressing a key from the number pad function prints out the corresponding character on the bottom half of the screen.
 * 0-9 print a character
 * param - keyCode 0-11
 *
 */
void proccessCommand(int keyCode)
{
    if(keyCode == previousKeyCode)
    {
        messageLoop ++;
    }
    else
    {
        messageLoop = 0;
        if(previousKeyCode != -1 && previousKeyCode != 10)
        {
            drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, BLACK, 1);
            if(messagePosition < 146 )
                messagePosition++;
            else
                messagePosition = 146;

            drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, WHITE, 1);
        }
    }
    char key = keyCodeParse(keyCode);
    if(key != '1' || messageTX[messagePosition] != '1')
    {
        messageTX[messagePosition] = key;
        previousKeyCode = keyCode;
        drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, WHITE, 1);
    }
    else
    {
        drawChar((messagePosition%21)*6,68+8*((messagePosition)/21),messageTX[messagePosition], BLUE, BLACK, 1);
        messagePosition++;
        messageTX[messagePosition] = key;
        previousKeyCode = keyCode;
        drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, WHITE, 1);
    }

}

void doDeltaCalc()
{
    for(i = buffersize-1; i > -1; i --) //calculates the delta time for all signals in the buffer
    {
        if(i != 0)
            timeBuffer[i] = signalTimeBuffer[i] - signalTimeBuffer[i-1];
        numKeyBuffer[i] = 0;
    }
    timeBuffer[0] = signalTimeBuffer[0];
}

int checkNumPadKeys()
{
    unsigned long bitToggle = 1; //for RC5 encoding
    int skip = 0;
    /*This section decodes the signal using RC5 Standard*/
    for(i = 0; i < buffersize; i++ ) //this loop assumes the key is a numpad key press 0-9 and attempts to proccess it as such.
    {
        if(timeBuffer[i] > 200000) //first bit is a 1
        {
            numKeyBuffer[j] = 10; //indicates the very first bit of each key press (should be at position 0,14,28,etc)
            j++;
            skip++;
            continue;
        }
        if(timeBuffer[i] > 80000)
        {
            skip++;
            bitToggle = bitToggle? 0:1;
        }
        if(skip%2)
        {
            skip++;
            continue;
        }
        skip++;
        numKeyBuffer[j] = bitToggle;
        j++;

    }
    /** Processes RC5 encoding to see if we have a number key press**/
    //2 high bits for start and 5 low bits for address (11x00000) x is toggle bit
    if(numKeyBuffer[0] == 10 && numKeyBuffer[1] == 1 && (numKeyBuffer[7]  + numKeyBuffer[6] * 2 + numKeyBuffer[5] *4 + numKeyBuffer[4]*8+numKeyBuffer[3]*16 == 0))
    {
        dataBits = numKeyBuffer[13]  + numKeyBuffer[12] * 2 + numKeyBuffer[11] *4 + numKeyBuffer[10]*8+numKeyBuffer[9]*16+numKeyBuffer[8]*32; //the last 4 bits of the 14 bit address determine the key code
        if(dataBits >= 0 && dataBits <= 9)
        {
            _keyCode = dataBits;
            return 1;
        }
    }
    return 0;
}

int checkEnterOrDelete()
{
    /*This section decodes the enter and delete signals by determining the width of each rising and falling edge*/
    j=0;
    for(i = 0; i < buffersize; i++ ) //determines the width of each crest/trough
    {
        if(timeBuffer[i] > 200000) //since the first edge is uninformative
        {
            enterOrDelete[j] = 10;
            j++;
            continue;
        }
        enterOrDelete[j] = timeBuffer[i]/14000; //divides each distance by 14000 clock cycles and truncates
        j++;

    }
    /** determines if the key was enter or delete **/
    int del = 0, ent = 0;
    for(i = 21; i < 36; i++) //compares the widths with the const arrays defined
    {
        if(del && ent) break;
        if(enterOrDelete[i] != ENTER[i-21])
            ent = 1;
        if(enterOrDelete[i] != DELETE[i-21])
            del = 1;
    }
    if(!ent)
    {
        _keyCode = 11; //ENTER
        return 1;
    }
    else if(!del)
    {
        _keyCode = 10; //DELETE
        return 1;
    }
    return 0;
}
/*
 *
 */
void png(const unsigned char* buffer, unsigned long size)
{
    upng_t* upng;
    upng = upng_new_from_bytes(buffer, size);

    if (upng != NULL) {
      upng_decode(upng);
      int i = upng_get_error(upng);
      if (upng_get_error(upng) == UPNG_EOK) {
        /* do stuff with image */
          int length = upng_get_size(upng);
          unsigned char b[50000];
          strcpy( (char*)b, (const char*)upng_get_buffer(upng));
          int i = 0;
          for(;i<length;i++)
          {
              UART_PRINT((const char*)b[i]);
          }
      }

      upng_free(upng);
    }
    free(p);

}
void simplePrint(int x, int y, char* message, int COLOR)
{
    int j=0;
    int k = strlen(message);
    for(; j < k; j++)
        drawChar((j)%21*6+x,(j)/21*8+y, message[j], COLOR, BLACK, 1);
}
void simplerPrint(int x, int y, char* message, int COLOR)
{
    int j=0;
    int k = strlen(message);
    for(; j < k; j++)
    {
        drawChar(j*6+x,y, message[j], COLOR, BLACK, 1);
        if(x > 21)
            return;
    }
}
void IRLoop(int mode)
{
    while(1)
    {
        if(pos == buffersize) //when the buffer is full button interrupts from the remote are disabled, this gives us time to process the last key pressed without worrying about stuff being overwritten while being read
        {
            successfulKeyRead=0;
            _keyCode = 0;
            j=0;

            MAP_GPIOIntDisable(GPIOA3_BASE,0x10); //prevents interrupt from writing anymore
            timerBlocking =1; // prevents timer from doing anything


            doDeltaCalc();
            int numkey = checkNumPadKeys();
            int enterdel = checkEnterOrDelete();
            successfulKeyRead = numkey + enterdel;
            /** This section is where keys are actuated */
            currSignalTime = Timer_IF_GetCount(TIMERA0_BASE, TIMER_A);
            if(previousKeyCode == _keyCode && currSignalTime - prevSignalTime < 10000000) //check if the button is being held by seeing if the key is still the same, within 100ms
            {
                prevSignalTime = currSignalTime;
                pos = 0;

                Timer_IF_Start(TIMERA2_BASE, TIMER_A, 80000000); //reload 1 second timer
                MAP_GPIOIntClear(GPIOA3_BASE,0x10);
                MAP_GPIOIntEnable(GPIOA3_BASE,0x10); //re-enable interrupts
                timerBlocking = 0;
                continue; //reenable interrupts before leaving the if statement
            }
            prevSignalTime = currSignalTime;
            _keyPressed = _keyCode;
            if(successfulKeyRead) //to prevent re-reading old keys, this variable is zero'd at the beginning of the function and is set 1 if an appropriate key is pressed
                if(mode == 0)
                {
                    if(_keyCode >= 0 && _keyCode <= 9)
                    {
                        proccessCommand(_keyCode);
                    }
                    else
                    {
                        if(_keyPressed == 10) //delete key
                        {
                            previousKeyCode = 10;
                            messageLoop = 0;
                            if(messageTX[messagePosition] != 0) //prioritize deleting whats currently highlighted
                            {
                                messageTX[messagePosition] = 0;
                                drawChar((messagePosition%21)*6,68+8*(messagePosition/21), messageTX[messagePosition], BLUE, WHITE, 1);
                            }
                            else if(messagePosition > 0) //if whats highlighted is blank, delete prev char
                            {
                                drawChar((messagePosition%21)*6,68+8*(messagePosition/21), messageTX[messagePosition], BLUE, BLACK, 1);
                                messagePosition--;
                                messageTX[messagePosition] = 0;
                                drawChar((messagePosition%21)*6,68+8*(messagePosition/21), messageTX[messagePosition], BLUE, WHITE, 1);

                            }
                        }
                        else //Send message and terminate loop (the program will only send one text per run to avoid oversending)
                        {
                            if(_keyPressed == 11 && (messagePosition > 0 || (messageTX[0] != 0 && messageTX[0] != ' '))) //enter key
                            {
                                MAP_GPIOIntDisable(GPIOA3_BASE,0x10);
                                Timer_IF_Stop(TIMERA1_BASE,TIMER_A);
                                Timer_IF_Stop(TIMERA2_BASE,TIMER_A);
                                return;
                            }
                        }
                    }
                }
                else
                {
                    int prev = indexer;
                    if(_keyCode == 2)
                        indexer++;
                    if(_keyCode == 5)
                        indexer--;
                    if(indexer > 97)
                        indexer = 0;
                    if(indexer < 0)
                        indexer = 97;
                    if(indexer != prev)
                    {
                        simplePrint(0,8,name[prev], BLACK);
                        simplePrint(0,8,name[indexer], BLUE);
                    }
                    if(_keyCode == 11) //select
                        return;
                }
            /** re enables interrupts **/
            Timer_IF_Start(TIMERA2_BASE, TIMER_A, 120000000); //reload 1 second timer
            MAP_GPIOIntClear(GPIOA3_BASE,0x10);
            MAP_GPIOIntEnable(GPIOA3_BASE,0x10); //re-enable interrupts

            pos = 0;
            timerBlocking = 0;
        }
    }
}
/**
 * Simple one line print
 */

void main()
{

    long lRetVal=-1;
    BoardInit();
    PinMuxConfig();
    init();


    simplePrint(0,0,"Pick Meme:",BLUE);
    simplePrint(0,8,name[0], BLUE);
    IRLoop(1);
    simplePrint(0,24,"TOP:(truncated)",BLUE);
    IRLoop(0);
    top = (char*) malloc(strlen(messageTX));
    strcpy(top,messageTX);
    simplerPrint(0,32,top, BLUE);
    //clearing
    simplePrint(0,40,"BOTTOM: (see top)",BLUE);

    clearTXScreen(messagePosition);
    for(i = 0; i < 146; i++)
    {
        messageTX[i] = 0;
    }
    drawChar(0,68, messageTX[messagePosition], BLUE, WHITE, 1);

    Timer_IF_Start(TIMERA1_BASE, TIMER_A, 8000000); // 100 millisecond timer
    Timer_IF_Start(TIMERA2_BASE, TIMER_A, 80000000); //1 seconds
    IRLoop(0);
    bottom = (char*) malloc(strlen(messageTX));
    strcpy(bottom,messageTX);
    simplerPrint(0,48,bottom, BLUE);

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
//    http_post(lRetVal);
    //signed char new_host[12] = "i.imgur.com";
    //g_Host = new_host;
    //lRetVal = tls_connect();
    http_post(lRetVal);
    //png((const unsigned char*) p, size);
    while(1);

}

//*****************************************************************************
//
//*****************************************************************************

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[4096];

    char* pcBufHeaders;
    int lRetVal = 0;
    pcBufHeaders = acSendBuff;
    // POSTHEADER + template + / + TOP + / + BOTTOM + / + POSTHEADER2
    strcpy(pcBufHeaders, POSTHEADER); //postheader1
    pcBufHeaders += strlen(POSTHEADER);

    strcpy(pcBufHeaders, path[indexer]);
    pcBufHeaders += strlen(path[indexer]); //template

    strcpy(pcBufHeaders, "/");
    pcBufHeaders += strlen("/"); // forward /

    strcpy(pcBufHeaders, top);
    pcBufHeaders += strlen(top); // top


    strcpy(pcBufHeaders, "/");
    pcBufHeaders += strlen("/"); // forward /

    strcpy(pcBufHeaders, bottom);
    pcBufHeaders += strlen(bottom); // bottom


    strcpy(pcBufHeaders, POSTHEADER2); //postheader2
    pcBufHeaders += strlen(POSTHEADER2);

    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);

    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);


    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
        char newText[50];
        char* p;
        strcpy(newText, strstr(acRecvbuff,".com\\/"));
        strcpy(newText, strstr(newText,"/"));
        p = strchr(newText,'\"');
        *p= 0;
        strcpy(link,newText);
        sl_Close(iTLSSockID);
    }

    return 0;
}

static int http_get(int iTLSSockID){
        char acSendBuff[512];
        char acRecvbuff[40000];
        char* pcBufHeaders;
        int lRetVal = 0;

        pcBufHeaders = acSendBuff;
        strcpy(pcBufHeaders, GETHEADER);
        pcBufHeaders += strlen(GETHEADER);

        //strcpy(pcBufHeaders, link);
        //pcBufHeaders += strlen(link);


        strcpy(pcBufHeaders, "\r\n");
        pcBufHeaders += strlen("\r\n");
        strcpy(pcBufHeaders, HOSTHEADER2);
        pcBufHeaders += strlen(HOSTHEADER2);
        strcpy(pcBufHeaders, CHEADER);
        pcBufHeaders += strlen(CHEADER);
        strcpy(pcBufHeaders, "\r\n\r\n");

        int testDataLength = strlen(pcBufHeaders);

        UART_PRINT(acSendBuff);


        //
        // Send the packet to the server */
        //
        lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
        if(lRetVal < 0) {
            UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
            sl_Close(iTLSSockID);
            return lRetVal;
        }
        int state = 0;
        int recvLen = 0;
        while(recvLen < 40000)
        {
            lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[recvLen], 16000, 0);
            if(!(lRetVal > 0)) {
                /* Error */
                sl_Close(iTLSSockID);
                break; /* Handling the error-codes is specific
                        * to the application */
            }
            if(lRetVal == 0)
                state++;
            else
                state = 0;
            if(state == 3)
                break;
            recvLen += lRetVal;
        }
        sl_Close(iTLSSockID);
        sl_Stop(SL_STOP_TIMEOUT);
        char* n = strstr(acRecvbuff,"Content-Length");
        n= n+16;
        size = atoi((const char*)n);
        char* loc = strstr(acRecvbuff,"PNG");
        p = (char*)malloc(size);
        int i;
        for(i = 0; i < size; i++)
            p[i] = *(loc-1+i);

        UART_PRINT("\n\r\n\r");

        return 0;
}
