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
#include "systick_if.h"
#include "uart_if.h"

#include "pin_mux_config.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

#define APPLICATION_VERSION  "1.1.1"
#define APP_NAME             "Interrupt Reference"
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

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
// Globals used for OLED
//static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
//static unsigned char g_ucRxBuff[TR_BUFF_SIZE];

// Globals used to save and restore interrupt settings.

static int messagePosition; // current position in messageTX
static int messageLength=0; // length of the message recieved
static char messageTX[147]; // stores message to transmit
static int messageLoop = 0; // for multitap
static int timerBlocking = 1; //prevents timers from doing anything
static int delay = 0; // time delay for uart transfers
static int previousSampleKeyCode = -1; //used for measuring confidence
static int previousKeyCode = -1; //used for multitap texting
static char c; // used for messageQuery, declared globally to avoid redeclaring
static unsigned long currSignalTime, prevSignalTime; //used to prevent button holds from registering


long goertzel(int samplep[], int coeff, int N); //prototyped functions
int post_test(void);

/**DTMF Detection variables **/
volatile unsigned char samplesMSB[410];
volatile unsigned char samplesLSB[410];
volatile int count=0;
volatile int flag;
volatile int new_dig;
static int _keyPressed;
static int SPIMutex = 0;


unsigned int power_all[8];
int f_tone[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
int coeff[8] = {31548, 31281, 30951, 30556, 29143, 28360, 27408, 26258};


#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

const int ENTER[] = {1,3,1,2,1,3,1,4,1,3,1,1,1,2,1}; //size 15
const int DELETE[] = {1,3,1,2,1,3,1,3,1,2,1,4,1,3,1};
int i;
static void TimerA1IntHandler()
{
    char t[2];
    Timer_IF_InterruptClear(TIMERA1_BASE);
    if(flag | SPIMutex | timerBlocking) return; //if the buffer is full, the SPI is being used, or timers are disable, do nothing
    SPIMutex = 1;
    MAP_GPIOPinWrite(GPIOA3_BASE, 0x10, 0x0); //set low to enable signal
    /*
    MAP_SPICSEnable(GSPI_BASE);
    SPIDataGet(GSPI_BASE,(unsigned long*)samples+count++);
    SPIDataGet(GSPI_BASE,(unsigned long*)samples+count++);
    MAP_SPICSDisable(GSPI_BASE);
    */
    MAP_SPITransfer(GSPI_BASE,0,t,2,SPI_CS_ENABLE|SPI_CS_DISABLE);
    MAP_GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10); //set high to disable signal
    samplesMSB[count] = t[0];
    samplesLSB[count] = t[1];

    //count++;
    count ++;
    if(count >= 410)
    {
        flag = 1;
        Timer_IF_Stop(TIMERA1_BASE,TIMER_A);
    }
    SPIMutex = 0;
}

/**
 * Long pause timer (1s)
 */
static void TimerA2IntHandler()
{
    Timer_IF_InterruptClear(TIMERA2_BASE);
    if(timerBlocking | SPIMutex || messageTX[messagePosition] == 0 && messagePosition != 146) //do nothing if mutexs are claimed, or no message/max buffer size
        return;
    SPIMutex = 1;
    previousKeyCode = -1;
    drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, BLACK, 1);
    messagePosition++;
    messageLoop = 0;
    drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, WHITE, 1);

    SPIMutex = 0;
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
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));
    //MAP_SPIFIFOEnable(GSPI_BASE, SPI_RX_FIFO);
    MAP_SPIFIFODisable(GSPI_BASE, SPI_RX_FIFO | SPI_TX_FIFO);
    MAP_SPIEnable(GSPI_BASE);
    Adafruit_Init(); //turns on adafruit

    //timers
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, (unsigned long)-1); // clock for ignoring button holds

    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, TimerA1IntHandler); // timer for sampling

    //Timer_IF_Init(PRCM_TIMERA2, TIMERA2_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    //Timer_IF_IntSetup(TIMERA2_BASE, TIMER_A, TimerA2IntHandler); // increment message position after timeout

    //GPIO P18 is set to CS for MCP3001
//
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
                return ' ';
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
/* Goertzel function */
long goertzel(int samples[], int coeff, int N)
{
    long Q, Q_prev, Q_prev2, i;
    long prod1, prod2, prod3, power;
    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero
    for(i=0; i<N; i++)
    {
        Q = (samples[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
        Q_prev2 = Q_prev;                                    // shuffle delay elements
        Q_prev = Q;
    }
    prod1=( Q_prev*Q_prev);
    prod2=( Q_prev2*Q_prev2);
    prod3=( Q_prev *coeff)>>14;
    prod3=( prod3 * Q_prev2);

    power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down

    return power;
}

/**post-test function**/
int post_test(void)
{
    //initialize variables to be used in the function
    int i,row,col,max_power;

     char row_col[4][4] =       // array with the order of the digits in the DTMF system
        {
        {1, 2, 3, 'A'},
        {4, 5, 6, 'B'},
        {7, 8, 9, 'C'},
        {10, 0, 11, 'D'} //delete, 0, enter
        };
    // find the maximum power in the row frequencies and the row number

        max_power=0;            //initialize max_power=0

        for(i=0;i<4;i++)        //loop 4 times from 0>3 (the indecies of the rows)
            {
            if (power_all[i] > max_power)   //if power of the current row frequency > max_power
                {
                max_power=power_all[i];     //set max_power as the current row frequency
                row=i;                      //update row number
                }
            }

    // find the maximum power in the column frequencies and the column number

        max_power=0;            //initialize max_power=0

        for(i=4;i<8;i++)        //loop 4 times from 4>7 (the indecies of the columns)
            {
                if (power_all[i] > max_power)   //if power of the current column frequency > max_power
                {
                    max_power=power_all[i];     //set max_power as the current column frequency
                    col=i;                      //update column number
                }
            }


    if((power_all[col]>1000 && power_all[row]>1000)) // check if maximum powers of row & column exceed certain threshold AND new_dig flag is set to 1
    {
        _keyPressed = row_col[row][col-4];
        if(_keyPressed < 12)
            return 1;
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
/**
 * Checks the RXBuffer to see if a new character is placed onto the buffer
 */
void messageQuery()
{
    for(delay=0; delay<1000; delay=delay+1);// delay minimum 1ms, delays are necessary for the uart message to properly be read
    if(UARTCharsAvail(UARTA1_BASE))
    {
        c = MAP_UARTCharGet(UARTA1_BASE);
        if(c == 0) //wipes screen on terminary character
        {
            clearRXScreen(messageLength);
            messageLength = 0;
            return;
        }
        drawChar((messageLength%21)*6,8*(messageLength/21),c , BLUE, BLACK, 1);
        messageLength++;

    }
}

void main()

{
    BoardInit();
    PinMuxConfig();
    init();

    fillScreen(BLACK);


    int succesfulKeyRead; //determines if a valid key was pressed
    int i = 0; //additional initializations necessary for functioning

    /*clear the message buffer*/
    for(i = 0; i < 146; i++)
    {
        messageTX[i] = 0;
    }
    messagePosition = 0;

    drawLine(0,64,128,64,BLUE); //separates the screen

    //interrupts enabled before entering loop
    int sum=0;
    int confidence;
    int timeout = 0;

    int tenbit[410];
    drawChar(0,68, messageTX[messagePosition], BLUE, WHITE, 1);
    //Timer_IF_Start(TIMERA2_BASE, TIMER_A, 80000000); //1 seconds
    Timer_IF_Start(TIMERA1_BASE, TIMER_A, 5000); // 5000 clock cycles * 12.5ns = 62.5 us = 16khz
    timerBlocking = 0;

    while(1)
    {

        if(UARTCharsAvail(UARTA1_BASE)) //checks if a message is avaliable. used during cross communication
        {
            timerBlocking = 1; // prevents timer from doing anything
            while(SPIMutex); // waits for all timer to finish
            while(UARTCharsAvail(UARTA1_BASE))
            {

                c = MAP_UARTCharGet(UARTA1_BASE);
                if(c == 0) //wipes screen on terminary character
                {
                    clearRXScreen(messageLength);
                    messageLength = 0;
                    break;
                }
                drawChar((messageLength%21)*6,8*(messageLength/21),c , BLUE, BLACK, 1);
                messageLength++;
                for(delay=0; delay<1000; delay=delay+1);// delay minimum 1ms, delays are necessary for the uart message to properly be read

            }
            timerBlocking = 0;
        }

        if(flag == 1) //when the buffer is full button interrupts from the remote are disabled, this gives us time to process the last key pressed without worrying about stuff being overwritten while being read
        {
            _keyPressed = -1;
            timerBlocking = 1; // prevents timer from doing anything
            while(SPIMutex); // waits for all timer to finish This line is actually not necessary but spaghetti code is nice.


            if(timeout == 90)
            {
                previousKeyCode = -1;
                drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, BLACK, 1);
                messagePosition++;
                messageLoop = 0;
                drawChar((messagePosition%21)*6,68+8*(messagePosition/21),messageTX[messagePosition], BLUE, WHITE, 1);
                timeout = 0;
            }
            if(messageTX[messagePosition] != 0 && messagePosition != 146)
                timeout++;
            succesfulKeyRead=0;

            /**
                 DTMF Signal processing
             */
            for(i = 0;i<410;i++)
                    tenbit[i] = (((samplesMSB[i]) & 0x1F) << 5) + ((samplesLSB[i]&0xF8) >>3);
            /**
             *  Removes DC bias from 10bit
             */
            for(i = 0; i < 410; i++)
                sum += tenbit[i];
            sum /= 410; //average
            for(i = 0; i < 410; i++)
                tenbit[i] -= sum;
            sum = 0;

                for(i = 0; i < 8; i++)
                    power_all[i] = goertzel( (int*)tenbit, coeff[i], 410 );
                succesfulKeyRead = post_test();
                if( succesfulKeyRead ) //3 level confidence to reduce noise
                {
                    if(_keyPressed == previousSampleKeyCode)
                    {
                        confidence++;
                    }
                    else confidence = 0;
                    previousSampleKeyCode = _keyPressed;
                }
                else confidence = 0;
            /**
             * Post processing
             */

                if(succesfulKeyRead && confidence == 4) //to prevent re-reading old keys, this variable is zero'd at the beginning of the function and is set 1 if an appropriate key is pressed
                {
                    timeout = 0; //we recognized a key dont time out
                    confidence = 0;
                    currSignalTime = Timer_IF_GetCount(TIMERA0_BASE, TIMER_A);
                    int x = currSignalTime - prevSignalTime;
                    if(previousKeyCode == _keyPressed && currSignalTime - prevSignalTime < 50000000 ) //check if the button is being held by seeing if the key is still the same
                    {
                        count = 0;
                        flag = 0;
                        //Timer_IF_Start(TIMERA2_BASE, TIMER_A, 80000000); //reload 1 second timer
                        prevSignalTime = currSignalTime;
                        MAP_GPIOIntClear(GPIOA3_BASE,0x10);
                        Timer_IF_Start(TIMERA1_BASE, TIMER_A, 5000);
                        MAP_GPIOIntEnable(GPIOA3_BASE,0x10); //re-enable interrupts
                        timerBlocking = 0;
                        continue; //reenable interrupts before leaving the if statement
                    }
                    prevSignalTime = currSignalTime;
                    if(_keyPressed >= 0 && _keyPressed <= 9)
                    {
                        proccessCommand(_keyPressed);
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
                            else
                            if(messagePosition > 0) //if whats highlighted is blank, delete prev char
                            {
                                drawChar((messagePosition%21)*6,68+8*(messagePosition/21), messageTX[messagePosition], BLUE, BLACK, 1);
                                messagePosition--;
                                messageTX[messagePosition] = 0;
                                drawChar((messagePosition%21)*6,68+8*(messagePosition/21), messageTX[messagePosition], BLUE, WHITE, 1);

                            }
                        }
                        else
                        {
                            if(_keyPressed == 11 && messagePosition > 0 || (messageTX[0] != 0 && messageTX[0] != ' ')) //enter key
                            {
                                previousKeyCode = 11;
                                messageLoop = 0;
                                UARTCharPutNonBlocking(UARTA1_BASE, 0);
                                messagePosition++; //increments message to print highlighted character it exists
                                for(i = 0; i < messagePosition; i++)
                                {
                                    if(messageTX[i] != 0)
                                    while(!UARTCharPutNonBlocking(UARTA1_BASE, messageTX[i]))
                                    {
                                        messageQuery(); // checks after sending
                                        messageQuery();
                                    }

                                }
                                messageQuery();
                                clearTXScreen(messagePosition);
                                drawChar(0,68, 0, BLUE, WHITE, 1);
                            }
                        }
                    }
                }
                /** re enables interrupts **/
                count = 0;
                flag = 0;
                Timer_IF_Start(TIMERA1_BASE, TIMER_A, 5000);
                //Timer_IF_Start(TIMERA2_BASE, TIMER_A, 80000000); //reload 1 second timer

                timerBlocking = 0;
        }
    }

}

//*****************************************************************************
//
//*****************************************************************************
