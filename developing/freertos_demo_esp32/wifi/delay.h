/*
 * delay.h
 *
 *  Created on: Dec 13, 2019
 *      Author: HoangSHC
 */

#ifndef WIFI_DELAY_H_
#define WIFI_DELAY_H_

#include "FreeRTOS.h"
#include "timers.h"

#define delay_s(s)      vTaskDelay( ( TickType_t )( 1000 * s ) / portTICK_PERIOD_MS )
#define delay_ms(ms)    vTaskDelay( ( TickType_t )( ms ) / portTICK_PERIOD_MS )

#endif /* WIFI_DELAY_H_ */
