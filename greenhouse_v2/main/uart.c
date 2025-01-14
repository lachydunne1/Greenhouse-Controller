#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "uart.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>
static const char * TAG = "UART RS485";
static QueueHandle_t uart_event_queue;

const uint8_t humi[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0xe84, 0x0a};
const uint8_t temp[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xd5, 0xca};
const uint8_t cond[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};
const uint8_t phph[] = {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0a};
const uint8_t nitro[] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xec5, 0xcb};
const uint8_t phos[] = {0x01, 0x03, 0x00, 0x05, 0x00, 0x01, 0xe94, 0x0b};
const uint8_t pota[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0xe64, 0x0b};
const uint8_t sali[] = {0x01, 0x03, 0x00, 0x07, 0x00, 0x01, 0xe35, 0xcb};
const uint8_t tds[] = {0x01, 0x03, 0x00, 0x08, 0x00, 0x01, 0xe05, 0xc8};

esp_err_t uart_init(){

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    /* install function args: S-O RX ring buffer, UART num, S-O TX ring buffer, EVENT Q handle and size, flag and interrupt */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_PORT_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_PORT_NUM, 20));

    ESP_ERROR_CHECK(uart_set_mode(UART_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX)); //configure for RS485 HD
    
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Init UART complete \n");

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    return ESP_OK;
}   


static void uart_event_task(void *pvParameters){

    //Waiting for UART event.
    uart_event_t uart_event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);

    for (;;) {
        if(xQueueReceive(uart_event_queue, (void * )&uart_event, (TickType_t)portMAX_DELAY)) {

            //is it necessary to zero the buffer?
            bzero(dtmp, UART_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_PORT_NUM);
            switch(uart_event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    /* malloc send param space for packing in prepare_data function*/
        
                    break;

                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;

                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;

                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;

                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_PORT_NUM,  UART_BUF_SIZE);
                    int pos = uart_pattern_pop_pos(UART_PORT_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, UART_BUF_SIZE);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_PORT_NUM);
                    } else {
                        uart_read_bytes(UART_PORT_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_PORT_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
             
                default:
                    ESP_LOGI(TAG, "uart event type: %d", uart_event.type);
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);

}