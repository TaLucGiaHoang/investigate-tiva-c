/*
 * debug_serial.c
 *
 *  Created on: Nov 4, 2019
 *      Author: HoangSHC
 */

#include <stdint.h>
//#include <stdbool.h>
//#include <string.h>
#include "inc/hw_memmap.h"
//#include "inc/hw_uart.h"
//#include "inc/hw_types.h"
//#include "driverlib/fpu.h"
//#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/uart.h"
//#include "utils/uartstdio.h"


// Use UART0 to debug
void debug_init(void)
{
    ;// Empty
}

void debug_putc(unsigned char c)
{
    MAP_UARTCharPut( UART0_BASE, c );
}

int debug_puts(const char *s)
{
	int len = 0;

	while (*s) {
		debug_putc(*s);
		s++;
		len++;
	}
	return len;
}

//#define debug_puts(fmt, args...) UARTprintf("DEBUG: %s:%d:%s(): " fmt, \
//    __FILE__, __LINE__, __func__, ##args)
