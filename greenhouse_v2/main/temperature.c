/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/temperature_sensor.h"


// Doesn't work with esp32 chips (doesn't have a built-in temperature sensor)
static const char *TAG = "example";
temperature_sensor_handle_t temp_sensor = NULL;
void init_temp_sensor(void)
{
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 0~50 ℃");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(0, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

float read_temp_K(void){
    ESP_LOGI(TAG, "Reading temperature");
    float tsens_value;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
    return tsens_value+273.15;
}