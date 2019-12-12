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

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define delay_ms(ms)    vTaskDelay( ( TickType_t )ms / portTICK_PERIOD_MS ) //delay ms

// Hardware init UART1
void drv_esp32_init(void)
{
    //
    // Set the clocking to run directly from the crystal.
    //
//    MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
//                       SYSCTL_XTAL_16MHZ);
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Set GPIO B0 and B1 as UART1 pins.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Enable the UART peripheral for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    MAP_UARTConfigSetExpClk(UART1_BASE, MAP_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}

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

int drv_esp32_send(const char* cmd)
{
//    int send_len = strlen(cmd);
//    char at_flag_buf[4] = {0x02, 0x00, 0x00, 0x00};
//    char len_buf[4] = {0x00, 0x00, 0x00, 'A'};
//    debug_puts(cmd);
//
//    /* Send SEND request */
//    driver_spi1_transfer(4, at_flag_buf, 0);
//    delay_ms(5);
//
//    /* Slice the length if it over than 127 */
//    if (send_len <= ESP_AT_BUFFER_SIZE) {
//        len_buf[0] = send_len & 127;
//        len_buf[1] = send_len >> 7;
//        len_buf[3] = 'A'; // check info
//    } else {
//        return -1; // Too large data
//    }
//
//    /* Send data length */
//    driver_spi1_transfer(4, len_buf, 0);
//    delay_ms(5);
//
//    /* Send data */
//    driver_spi1_transfer(send_len, cmd, 0);
//    delay_ms(15);

    UARTSend(UART1_BASE, cmd, strlen(cmd));
    delay_ms(15);
    return 0;
}

int drv_esp32_recv(char* recvbuf, int* len)
{
//    char at_flag_buf[4] = {0x01, 0x00, 0x00, 0x00};
//    char len_buf[4] = {0x00, 0x00, 0x00, 0x00};
//    int recv_len = 0;
//
//    /* Send RECV request */
//    at_flag_buf[0] = 0x1;
//    if(driver_spi1_transfer(4, at_flag_buf, 0) != 0)
//    {
//        return -1;
//    }
//    delay_ms(5);
//
//    /* Receive data length */
//    if(driver_spi1_transfer(4, 0, len_buf) != 0)
//    {
//        return -1;
//    }
//    delay_ms(5);
//
//    /* Check info */
//    if(len_buf[3] == 'B')
//    {
//        recv_len = (len_buf[1] << 7) + len_buf[0];
//    }
//
//    /* Receive data */
//    if (recv_len < ESP_AT_BUFFER_SIZE) {
//        if(driver_spi1_transfer(recv_len, 0, recvbuf) != 0)
//        {
//            return -1;
//        }
//        recvbuf[recv_len] = 0;
////      debug_puts(recvbuf);
//    } else {
//        debug_puts("recv_len over size\n");
//    }
//
//    // return length
//    if(len) {
//        *len = recv_len;
//    }
//    delay_ms(15);

    int i = 0;
    for(i = 0 ; i < *len ; i++)
    {
        recvbuf[i] = UARTCharGet(UART1_BASE); // Waits for a character
    }
    *len = i;
    return i;
}

