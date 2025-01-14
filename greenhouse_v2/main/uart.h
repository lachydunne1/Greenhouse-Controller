#ifndef uart
#define uart

#include "driver/uart.h"
#include "esp_log.h"

#define UART_BUF_SIZE (1024)
#define UART_BAUD_RATE 4800 //baud rate of sensor
#define UART_PORT_NUM UART_NUM_0 
#define PATTERN_CHR_NUM    (3)  

esp_err_t uart_init(void);
static void const uart_event_task(void *pvParameters);

#endif