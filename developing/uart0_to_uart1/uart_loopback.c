//*****************************************************************************
//
// uart_echo.c - Example demonstrating UART module in internal loopback mode.
//
// Copyright (c) 2015-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

//*****************************************************************************
//
//! \addtogroup uart_examples_list
//! <h1>UART Loopback (uart_loopback)</h1>
//!
//! This example demonstrates the use of a UART port in loopback mode.  On
//! being enabled in loopback mode, the transmit line of the UART is internally
//! connected to its own receive line.  Hence, the UART port receives back the
//! entire data it transmitted.
//!
//! This example echoes data sent to the UART's transmit FIFO back to the same
//! UART's receive FIFO.  To achieve this, the UART is configured in loopback
//! mode.  In the loopback mode, the Tx line of the UART is directly connected
//! to its Rx line internally and all the data placed in the transmit buffer is
//! internally transmitted to the Receive buffer.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board.
//! - UART7 peripheral - For internal Loopback
//! - UART0 peripheral - As console to display debug messages.
//!     - UART0RX - PA0
//!     - UART0TX - PA1
//!
//! UART parameters for the UART0 and UART7 port:
//! - Baud rate - 115,200
//! - 8-N-1 operation
//
//*****************************************************************************

//*****************************************************************************
//
// Macros used in this application.
//
//*****************************************************************************
#define NUM_UART_DATA    4

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Send a string to the UART.  This function sends a string of characters to a
// particular UART module.
//
//*****************************************************************************
void
UARTSend(uint32_t ui32UARTBase, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        MAP_UARTCharPut(ui32UARTBase, *pui8Buffer++);
    }
}

int
uart_gets(uint32_t ui32Base, char *pcBuf, uint32_t ui32Len)
{
    uint32_t ui32Count = 0;
    int8_t cChar;
    static int8_t bLastWasCR = 0;

    if( (pcBuf == 0) || (ui32Len == 0))
        return -1;

    //
    // Adjust the length back by 1 to leave space for the trailing
    // null terminator.
    //
    ui32Len--;

    while(1)
    {
        cChar = MAP_UARTCharGet(ui32Base);

        // See if the backspace key was pressed.
        if(cChar == '\b')
        {
            if(ui32Count)
            {
                MAP_UARTCharPut(ui32Base, '\b');
                MAP_UARTCharPut(ui32Base, ' ');
                MAP_UARTCharPut(ui32Base, '\b');
                ui32Count--;
            }

            continue;
        }

        //
        // If this character is LF and last was CR, then just gobble up the
        // character because the EOL processing was taken care of with the CR.
        //
        if((cChar == '\n') && bLastWasCR)
        {
            bLastWasCR = 0;
            continue;
        }

        //
        // See if a newline or escape character was received.
        //
        if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
        {
            //
            // If the character is a CR, then it may be followed by a LF which
            // should be paired with the CR.  So remember that a CR was
            // received.
            //
            if(cChar == '\r')
            {
                bLastWasCR = 1;
            }

            break;
        }

        if(ui32Count < ui32Len)
        {
            pcBuf[ui32Count] = cChar;
            ui32Count++;

            // Reflect the character back to the user.
            MAP_UARTCharPut(ui32Base, cChar);
        }
    }

    // Add a null termination to the string.
    pcBuf[ui32Count] = 0;

    // Send a CRLF pair to the terminal to end the line.
    MAP_UARTCharPut(ui32Base, '\r');
    MAP_UARTCharPut(ui32Base, '\n');

    return(ui32Count);
}
//*****************************************************************************
//
// Configue UART in internal loopback mode and tranmsit and receive data
// internally.
//
//*****************************************************************************
int
main(void)
{

#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    uint32_t ui32SysClock;
#endif
    uint8_t ui8DataTx[NUM_UART_DATA];
    uint8_t ui8DataRx[NUM_UART_DATA];
    uint32_t ui32index;

    //
    // Set the clocking to run directly from the crystal.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_OSC), 25000000);
#else
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
#endif

    //
    // Enable the peripherals used by this example.
    // UART0 :  To dump information to the console about the example.
    // UART7 :  Enabled in loopback mode. Anything transmitted to Tx will be
    //          received at the Rx.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Internal loopback programming.  Configure the UART in loopback mode.
    //
    UARTLoopbackEnable(UART7_BASE);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, MAP_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    MAP_UARTConfigSetExpClk(UART7_BASE, MAP_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    while(1)
    {
        char c = 0;
        c = UARTCharGet(UART0_BASE); // get from terminal

        if(c == '\n')
        {
            UARTCharPut(UART7_BASE, '\r');
        }

        UARTCharPut(UART7_BASE, c);

        if(c == '\n')
        {
            break;
        }
    }

    char buf[100] = {0};
    uart_gets(UART0_BASE, buf, sizeof(buf));
    UARTSend(UART7_BASE, (uint8_t*)buf, sizeof(buf));
    UARTSend(UART0_BASE, (uint8_t*)buf, sizeof(buf));

    UARTSend(UART0_BASE, (uint8_t *)"\n\rDone", strlen("\n\rDone"));
    //
    // Return no errors
    //
    return(0);
}
