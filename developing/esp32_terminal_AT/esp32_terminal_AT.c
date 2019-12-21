//*****************************************************************************
//
// esp32_terminal_AT.c - Example demonstrating how to send AT commands to ESP32 via UART.
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

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
//! UART parameters for the UART0 and UART1 port:
//! - Baud rate - 115,200
//! - 8-N-1 operation
//!
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
UARTSend(uint32_t ui32UARTBase, const char *pui8Buffer, uint32_t ui32Count)
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
UARTRecv(uint32_t ui32Base, char *pcBuf, uint32_t ui32Len)
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

//    // Add a null termination to the string.
//    pcBuf[ui32Count] = 0;
    // Add a null termination to the string.
    pcBuf[ui32Count++] = '\r'; // add CR
    pcBuf[ui32Count++] = '\n'; // add LF
    pcBuf[ui32Count] = 0; // add Null

    // Send a CRLF pair to the terminal to end the line.
    MAP_UARTCharPut(ui32Base, '\r');
    MAP_UARTCharPut(ui32Base, '\n');

    return(ui32Count);
}
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = MAP_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    MAP_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(MAP_UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART7 and write it back to the UART0.
        //
        MAP_UARTCharPutNonBlocking(UART0_BASE,
                                   MAP_UARTCharGetNonBlocking(UART1_BASE));
    }
}

//*****************************************************************************
//
// Delay for millisecond.
//
//*****************************************************************************
void delay_ms(int32_t ms)
{
    int32_t i;
    for(i = 0; i < ms; i++)
    {
        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));
    }

}

//*****************************************************************************
//
// Configue UART in internal loopback mode and tranmsit and receive data
// internally.
//
//
// Pin connections:
// Tiva-C       ESP32-Wroom
//   PB0   -->    TX2
//   PB1   -->    RX2
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the peripherals used by this example.
    // UART0 :  To dump information to the console about the example.
    //          U0Rx/PA0 , U0Tx/PA1
    // UART1 :  To communicate with esp32
    //          U1Rx/PB0/PC4 , U1Tx/PB1/PC5
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Set GPIO B0 and B1 as UART1 pins.
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);



    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    MAP_UARTConfigSetExpClk(UART1_BASE, 16000000, 115200,
                            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_WLEN_8));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Initialize the UART0.
    //
    ConfigureUART();

    // Print instructions.
    UARTprintf("Type and press ENTER to send to ESP32\n");
    UARTprintf("Example AT commands:             \n");
    UARTprintf("  \"AT+RST\"  : Reset ESP32      \n");
    UARTprintf("  \"ATE0\"    : Switch echo off  \n");
    UARTprintf("  \"AT\"      : Test AT          \n");

    while(1)
    {
        char cmd[100];
        int len;

        // Get input from terminal
        len = UARTgets(cmd, sizeof(cmd));

        // Append CRLF and NULL to cmd
        cmd[len] = '\r'; // add CR
        cmd[len+1] = '\n'; // add LF
        cmd[len+2] = 0; // add null

        // Send to ESP32
        UARTSend(UART1_BASE, cmd, strlen(cmd));
    }

    return(0);
}
