/*
 * drv_esp32.h
 *
 *  Created on: Dec 12, 2019
 *      Author: HoangSHC
 */

#ifndef DRV_ESP32_H_
#define DRV_ESP32_H_

#define ESP_AT_BUFFER_SIZE (1024)
void drv_esp32_init(void);
int drv_esp32_send(const char* cmd);
int drv_esp32_recv(char* recvbuf, int* len);

#endif /* DRV_ESP32_H_ */
