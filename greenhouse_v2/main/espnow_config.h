#ifndef ESPNOW_CONFIG_H
#define ESPNOW_CONFIG_H

#include "esp_now.h"

/* */
#define CONFIG_DEEP_SLEEP_TIME_MS 10000
#define CONFIG_ESPNOW_SEND_COUNT 100
#define CONFIG_ESPNOW_SEND_LEN 5 //questionable
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_DELAY_TIME 10
#define CONFIG_DATA_LEN 12 //test or now
#define ESPNOW_WIFI_MODE WIFI_MODE_STA

#define ESPNOW_PACKET_BUFFER_SIZE 1024*2
#define ESPNOW_QUEUE_SIZE 5

extern uint8_t destination_mac[ESP_NOW_ETH_ALEN];

void wifi_init(void);
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
void espnow_shutdown(void);

void espnow_data_prepare(uint8_t *data, float moisture_percentage, float current_1, float voltage_1, float humi, float temp, float cond, 
                        float ph, float nitro, float phos, float potas, float sali, float tds, int data_no);
void espnow_data_parse(const uint8_t *mac_addr, const uint8_t *data, int len);
esp_err_t init_esp_now(void);


typedef enum {
    ESPNOW_RECV_CB,
    ESPNOW_SEND_CB,
}espnow_event_id_t;

/* data is packed into send param, which contains transmit information */
typedef struct {
    float moisture_percentage;    
    float current_1; //greenhouse current draw
    float voltage_1; //greenhouse voltage level
    float humi;
    float temp;
    float cond;
    float ph;
    float nitro;
    float phos;
    float potas;
    float sali;
    float tds;
} __attribute__((packed)) espnow_data_t;

typedef struct {
    bool unicast; /* datalogger only requires unidirectional comms */
    uint8_t count; /* delay count */
    int len; /* data length */
    uint8_t *buffer; /* buffer pointing to the data */
    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; /* length of ESPNOW Mac address */
} espnow_send_param_t;

/* ESPNOW CALLBACK STRUCTS */

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
}espnow_event_recv_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
}espnow_event_send_cb_t;

/* for event handling in queue SM*/
typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
}espnow_event_info_t;

typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

/* send only structs */
enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
};

/* send param and espnow queue require access across callbacks and espnow tasks,
    so they have been initialised in src file and declared extern in header */
extern espnow_send_param_t *send_param;
extern QueueHandle_t espnow_queue;
#endif 
