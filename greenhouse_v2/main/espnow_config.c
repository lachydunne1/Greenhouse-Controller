#include <string.h> //memcpy
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "espnow_config.h"

// master address [1] -> should host receive and blynk app

uint8_t destination_mac[ESP_NOW_ETH_ALEN] =  {0x8c, 0x4f, 0x0, 0x10, 0xd2, 0x40};
static const char *TAG = "espnow_config";
espnow_send_param_t *send_param;


void wifi_init(){
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t init_esp_now(){

    send_param = malloc(sizeof(espnow_send_param_t));
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer_info = {};
    peer_info.channel = CONFIG_ESPNOW_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, destination_mac, ESP_NOW_ETH_ALEN);
    memcpy(send_param->dest_mac, destination_mac, ESP_NOW_ETH_ALEN); // this is allowed since there is only unidirectional communications
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    //ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb)); main purpose of datalogger is send
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    return ESP_OK;

}

/* not necessary in this, receiver is set up on blynk end */
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len){

    espnow_event_t event;
    espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0){
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    event.id = ESPNOW_RECV_CB;
    /* copy data into event */
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);

    if(recv_cb->data == NULL){
        ESP_LOGE(TAG, "Malloc receive data fail");
        free(recv_cb->data);
        return;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;

}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
    
    espnow_event_t event;
    espnow_event_send_cb_t *send_cb = &event.info.send_cb;

    if (mac_addr == NULL){
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    event.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    ESP_LOGI(TAG, "Sending data to "MACSTR, MAC2STR(send_cb->mac_addr));
    ESP_LOGI(TAG, "Send status: %d", send_cb->status);
}

/*
    14/01/25: Why the fuck was the assert passing when we were using esp32, but failing with esp32s3

*/
void espnow_data_prepare(uint8_t *data, float moisture_percentage, float current_1, float voltage_1, float humi, float temp, float cond, 
                        float ph, float nitro, float phos, float potas, float sali, float tds, int data_no){

    if (data_no!=CONFIG_DATA_LEN){
        ESP_LOGE(TAG, "Invalid command length");
    }
    
    send_param-> unicast = true;
    send_param-> count = CONFIG_ESPNOW_SEND_COUNT;
    send_param-> len = sizeof(float)*data_no;
    //TODO: ADD functionality
    send_param->buffer = malloc(sizeof(float)*data_no);

    if (send_param -> buffer == NULL){
        ESP_LOGE(TAG, "Malloc send buffer fail.");
        free(send_param);
    }

    espnow_data_t *buf = (espnow_data_t *)send_param-> buffer;
    /* check for buffer overruns*/
    ESP_LOGI(TAG, "SIZEOFDATA: %d" , sizeof(espnow_data_t));
    assert(send_param->len >= sizeof(espnow_data_t));

    /*Safely copy the sequential 4 bytes [float] into the send buffer */
    size_t offset = 0;
    memcpy(&(buf->moisture_percentage), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->humi), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->current_1), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->voltage_1), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->temp), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->cond), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->ph), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->nitro), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->phos), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->potas), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->sali), data + offset, sizeof(float)); offset += sizeof(float);
    memcpy(&(buf->tds), data + offset, sizeof(float));

}

void espnow_shutdown(){

    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}