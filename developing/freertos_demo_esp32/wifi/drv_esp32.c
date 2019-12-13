/*
 * drv_esp32.c
 *
 *  Created on: Dec 12, 2019
 *      Author: HoangSHC
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
//#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "drv_esp32.h"
#include "debug_serial.h"
#include "delay.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


// Hardware init UART1
void drv_esp32_init(void)
{
    //
//    // Enable the GPIO Peripheral used by the UART.
//    //
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//
//    //
//    // Set GPIO B0 and B1 as UART1 pins.
//    //
//    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
//    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
//    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
//    //
//    // Use the internal 16MHz oscillator as the UART clock source.
//    //
//    ROM_UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
//
//    //
//    // Enable the UART peripheral for use.
//    //
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//
//    //
//    // Configure the UART for 115,200, 8-N-1 operation.
//    //
//    MAP_UARTConfigSetExpClk(UART1_BASE, MAP_SysCtlClockGet(), 115200,
//                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                             UART_CONFIG_PAR_NONE));
}

//*****************************************************************************
//
// Send a string to the UART.  This function sends a string of characters to a
// particular UART module.
//
//*****************************************************************************
static void
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

int drv_esp32_send(const char* cmd)
{
    UARTSend(UART1_BASE, cmd, strlen(cmd));
    delay_ms(15);
    return 0;
}

unsigned char drv_esp32_uart_getc(void)
{
    return(MAP_UARTCharGet(UART1_BASE));
}

int drv_esp32_recv(char* recvbuf, int* len)
{
//    if(!recvbuf)
//    {
//        return -1;
//    }
//
//    int i = 0;
//    for(i = 0 ; i < *len ; i++)
//    {
//        recvbuf[i] = ROM_UARTCharGet(UART1_BASE); // Waits for a character
//    }
//    *len = i;
//    return i;
    return(MAP_UARTCharGet(UART1_BASE));
}

