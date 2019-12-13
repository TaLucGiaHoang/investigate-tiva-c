#include <string.h>

#include "debug_serial.h"

#include "drv_esp32.h"
#include "esp32_AT.h"
#include "delay.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


enum {
    MSG_SEND_STRING,
	MSG_RECV_STRING,
	MSG_CLEAR_STRING,
};


static char messageReceive[ESP_AT_BUFFER_SIZE];
static QueueHandle_t xQueueRx, xQueueTx, xQueueRxLen;
static SemaphoreHandle_t xSemaphore = NULL; // spi1 semaphore

extern xSemaphoreHandle g_pUARTSemaphore;

struct ESP32_ST
{
    int id;
    void *data;
    int len;
} s_esp32;

static void prvESP32Task(void* pvParameters);
#ifdef USE_SPI
static void prvSpiRxTask(void* pvParameters);
#else // USE UART
static void prvUartRxTask(void* pvParameters);
#endif
static int queue_send_cmd(int id, void* data, int len);
static int queue_read_msg(char* s, int s_len);
static int queue_send_msg_len(size_t msg_len);

#ifdef USE_SPI
static void prvSpiRxTask( void *pvParameters )
{
	/* Prevent the compiler warning about the unused parameter. */
	( void ) pvParameters;
	char c;
//	int i = 0;
	char *s = messageReceive;
	int len = 0;
	int ret = 0;
	memset(s, 0, sizeof(messageReceive));
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	debug_puts("Start ");debug_puts(__func__);debug_puts("\n");
	xSemaphoreGive(xSemaphore);
	for( ;; )
	{

		if( xQueueRx != 0 )
		{debug_puts(__func__);debug_puts("\n");
			s = messageReceive;
			s[0] = 0;
			len = 0;
			xSemaphoreTake(xSemaphore, portMAX_DELAY);
			ret = drv_esp32_recv(s, &len);
			xSemaphoreGive(xSemaphore);

			if( ret != 0)
			{
				// recv error
			}
			else
			{
				if(len > 0) {
					s[len] = 0;
					debug_puts("esp32> ");  // DEBUG


					// send to queue
					while(*s) {
						c = *(s);
						debug_putc(c);  // DEBUG

						if( xQueueSend( xQueueRx, ( void * ) &c, ( TickType_t )2000 ) != pdPASS )
						{
							/* Failed to post the message. */
							debug_puts("xQueueRx is full --> clear\n");
							queue_read_msg(0,0); // clear queue after 2000 ticks
							xQueueSend( xQueueRx, ( void * ) &c, ( TickType_t )0 ); // re-send queue
						}
						s++;
					}
				}
			}

			memset(s, 0, sizeof(messageReceive));
			delay_ms(100); // sleep 100ms
		}
	}

    vTaskDelete(NULL);
    vAssertCalled();
}
#else
static void prvUartRxTask( void *pvParameters )
{
    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    char c;
    int i = 0;
    char *s = messageReceive;

    memset(s, 0, sizeof(messageReceive));

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    debug_puts("Start ");debug_puts(__func__);debug_puts("\n");
    xSemaphoreGive(g_pUARTSemaphore);
    for( ;; )
    {

        if( xQueueRx != 0 )
        {
            c = drv_esp32_uart_getc();
        ///////////////
            if(i == 0)
            {
                debug_puts("esp32> ");  // DEBUG
            }
            debug_putc(c);  // DEBUG

            /* Collect 1 line */
            s[i++] = c;
            if(c == '\n')
            {
                s[i] = 0;
                i = 0;
                s = messageReceive;
            }
            // send to queue
            if( xQueueSend( xQueueRx, ( void * ) &c, ( TickType_t )0  /*portMAX_DELAY*/ ) != pdPASS )
            {
                /* Failed to post the message, even after 10 ticks. */
                debug_puts("xQueueRx is full --> clear\n");
                queue_read_msg(0,0); // clear queue after 2000 ticks
                xQueueSend( xQueueRx, ( void * ) &c, ( TickType_t )0 ); // re-send queue
            }
        }
    }
    vTaskDelete(NULL);
//    vAssertCalled();
}
#endif
static void prvESP32Task(void* pvParameters)
{
	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;
	struct ESP32_ST esp32;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    debug_puts("Start ");debug_puts(__func__);debug_puts("\n");
    xSemaphoreGive(g_pUARTSemaphore);
	for(;;)
	{
		if( xQueueTx != 0 )
		{
			if( xQueueReceive( xQueueTx, ( void * )&esp32 , ( TickType_t ) 100 ) != pdPASS )
			{
			    /* Failed to get the message. */
			} else
			{debug_puts(__func__);debug_puts("\n");
				int id = s_esp32.id;
				switch (id)
				{
				case MSG_SEND_STRING:
				{
					char *data = (char*)esp32.data;
//					xSemaphoreTake(xSemaphore, portMAX_DELAY);
					drv_esp32_send(data);
//					xSemaphoreGive(xSemaphore);
					break;
				}
				case MSG_RECV_STRING:
				{
					char* msg = (char*)esp32.data;
					int len = esp32.len;
					uint32_t recv_len = 0;

					/* Read esp32 response messages */
					recv_len = queue_read_msg(msg, len);
					/////////////////
					/* len == 0 : clear xQueueRx
					 * len > 0  : get response messages
					 */
					if(len > 0)
					{
						if( queue_send_msg_len(recv_len) != 0 )
						{
							debug_puts("Error: queue_send_msg_len\n");
						}
					}
					break;
				}
				case MSG_CLEAR_STRING:
				{
//					char* msg = (char*)esp32.data;
//					int len = esp32.len;
//					int recv_len = 0;
					/* Read esp32 response messages */
					queue_read_msg(0,0); // recv_len = queue_read_msg(msg, len);
					break;
				}
				default:
				{
					break;
				};
				}
			}
		} else
		{
			debug_puts("xQueueTx was not created\n");
		}

	}

    vTaskDelete(NULL);
//    vAssertCalled();
}

static int queue_send_cmd(int id, void* data, int len)
{
    s_esp32.id = id;
    s_esp32.data = data;
    s_esp32.len = len;
    if( xQueueSend( xQueueTx, ( void * ) &s_esp32,( TickType_t ) portMAX_DELAY ) != pdPASS )
    {
      /* Failed to post the message */
    	return -1;
    }

    return 0;
}

static int queue_read_msg(char* s, int s_len)
{
	char c;
	int i = 0;
	do
	{
		if( xQueueReceive( xQueueRx, ( void * ) &c, ( TickType_t ) 0 ) != pdPASS )
		{
			break; // queue is empty
		}

		if(s)
			s[i] = c;

		if(s_len != 0)
			if(i > s_len)
				break;

		i++;
	}while(1);

	return i;
}

static int queue_send_msg_len(size_t msg_len)
{
	size_t u32;
	if( xQueueRxLen == 0 )
	{
		/* xQueueRxLen was not created */
		return -1;
	}

	/* Clear queue */
	if( xQueueReceive( xQueueRxLen, ( void * ) &u32 , ( TickType_t ) 0 ) != pdPASS )
	{
		/* Queue empty or timeout */
	}

	/* Send size to queue */
	if( xQueueSend( xQueueRxLen, ( void * ) &msg_len, ( TickType_t ) 0 ) != pdPASS )
	{
		/* Failed to post the message */
		return -1;
	}

	return 0;
}

uint32_t esp32_create_tasks(void)
{
	xSemaphore = xSemaphoreCreateMutex();
	if ( ( xSemaphore ) != NULL )
	  xSemaphoreGive( ( xSemaphore ) );

	debug_puts("esp32_create_tasks\r\n");
    xQueueRx = xQueueCreate( 1024, sizeof( char ) );
    if( xQueueRx == NULL )
    {
        /* Queue was not created and must not be used. */
        debug_puts("xQueueRx was not created\r\n");
        return 1;
    }

    xQueueTx = xQueueCreate( 2, sizeof( s_esp32 ) );
    if( xQueueTx == NULL )
    {
        /* Queue was not created and must not be used. */
    	debug_puts("xQueueTx was not created\r\n");
    	return 1;
    }

    xQueueRxLen = xQueueCreate( 1, sizeof( uint32_t ) );
    if( xQueueRxLen == NULL )
    {
        /* Queue was not created and must not be used. */
        debug_puts("xQueueRxLen was not created\r\n");
        return 1;
    }

    //
    // Create the ESP32 AT tasks.
    //
    if( xTaskCreate( prvESP32Task, "prvESP32Task", 1024, NULL, 1, NULL ) != pdTRUE )
    {
        return 1;
    }

    if( xTaskCreate( prvUartRxTask, "prvUartRxTask", 512, NULL, 1, NULL ) != pdTRUE )
    {
        return 1;
    }

    //
    // Success.
    //
    return(0);
}

int esp32_send_cmd(const char* cmd, int wait_ms)
{
	int ret;
	ret = queue_send_cmd( MSG_SEND_STRING, (void*)cmd, (int)strlen(cmd) );
	delay_ms(wait_ms + 50);
	return ret;
}

size_t esp32_recv(char* msg, int msg_len)
{
	uint32_t recv_len = 0;

	if(msg_len == 0)
	{
		queue_send_cmd( MSG_CLEAR_STRING, (void*)msg, msg_len);
		delay_ms(0 + 50);
	} else
	{
		queue_send_cmd( MSG_RECV_STRING, (void*)msg, msg_len);
		delay_ms(0 + 50);
		if(msg_len > 0)
		{
			if( xQueueRxLen != 0 )
			{
				/* Wait to receive size from queue */
				if( xQueueReceive( xQueueRxLen, ( void * ) &recv_len , ( TickType_t ) 10000 /*portMAX_DELAY*/ ) != pdPASS )
				{
					/* Failed to get from queue. */
					recv_len = 0;
				}
			}
		}
	}
	
	return recv_len;
}


//void esp32_wifi_conn(const char* ssid, const char* pssid, int wait_ms)
//{
////	esp32_send_cmd("AT+CWMODE=1\r\n", 100);
////	esp32_send_cmd("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASS "\"\r\n", 6000);
//	esp32_send_cmd("AT+CWJAP=\"", 0);
//	esp32_send_cmd(ssid, 0);
//	esp32_send_cmd("\",\"", 0);
//	esp32_send_cmd(pssid, 0);
//	esp32_send_cmd("\"\r\n", wait_ms);
//}
//
//void esp32_reset_module(void)
//{
//	esp32_send_cmd("AT+RST\r\n", 1000);  // reset ESP32-SOLO-1
//}
//
//void esp32_echo_off(void)
//{
//	esp32_send_cmd("ATE0\r\n", 1000);  // echo off
//}


