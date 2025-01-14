#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
//#include "driver/temperature_sensor.h"
#include "temperature.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "adc.h"
#include "espnow_config.h"
#include "gpio.h"

/* Slave Plant IO includes:

    -average greenhouse plant moisture over three sensor locations
    -controller current draw
    -battery voltage monitoring

*/

/* Greenhouse_controller_v2:

    - TODO: Debug ESPNOW send failure

*/

#define SAMPLING_PERIOD 500 // ms
#define PUMP_DELAY 5000
#define MOISTURE_THESHOLD 0.3

static SemaphoreHandle_t Pump_trigger_semaphore;

volatile bool is_Pump_open = false; // Pump is closed initially
/* ^^^^this variable could be removed in favour of checking if gpio is high. to do so:
    configure input output and pullup to prevent noise on GPIO */

const static char *TAG = "Main";

void pump_task(void *pvParameters);
void adc_task(void *pvParameters);

/* SENSOR HELPER FUNCTIONS*/
float calculate_mositure(int moisture_reading); //maps ADC moisture reading to %
float calculate_current(float adc_voltage); //maps ACS712 output voltage to A
float calculate_voltage(float adc_voltage); //maps TL974 voltmeter voltage to actual voltage

void app_main(void)
{
    // create binary semaphore for triggering event tasks
    Pump_trigger_semaphore = xSemaphoreCreateBinary();
    if (Pump_trigger_semaphore == NULL){
        ESP_LOGE(TAG, "Failed to create binary Semaphore");
    }
    wifi_init();
    if(init_esp_now() != ESP_OK){
        ESP_LOGE(TAG, "Error: espnow failed to init");
    }
    //init periperals
    init_gpio();
    adc_esp32_init();
    adc1115_init();

    /* task, task name, stack depth, task parameter, priority, task handle */
    if(xTaskCreate(adc_task, "Sampling Task", 2048*4, NULL, 1, NULL) != pdPASS) {
        printf("Failed to start Sampling Task! \n");
    }
    if(xTaskCreate(pump_task, "Pump Task", 2048, NULL, 2, NULL) != pdPASS){
        printf("Failed to start Pump Task! \n");
    }

}

// Pump delay will be evaluated upon soil moisture content. 
void pump_task(void *param) {
    while (1) {
        // Wait for the semaphore to be given by adc_task
        if (xSemaphoreTake(Pump_trigger_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Low Moisture Content triggered: Opening Pump for X seconds");
            
            // TODO: turn on the pump
            gpio_set_level(PUMP_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(PUMP_DELAY));  // Simulate a delay for the event task (e.g., 2 seconds)
            gpio_set_level(PUMP_GPIO, 0);
            ESP_LOGI(TAG, "Event task completed");
            // Reset the flag to allow new triggers
            is_Pump_open = false;
        } else {
            ESP_LOGE(TAG, "Failed to take Semaphore");
        }
    }
}




void adc_task(void *pvParameters){
    /* in task: sample ADC's in series. */
    int16_t adc_raw[3];
    int esp32_adc_voltage_raw=0;
    int esp32_adc_voltage=0;

    float moisture_percentage[3];
    float avg_moisture = 0;

    float current_reading = 0;
    float voltage_reading = 0;
    float power = 0;

    uint8_t* dtmp = (uint8_t*)malloc(ESPNOW_PACKET_BUFFER_SIZE);
    
    while (1) {

        /* CONSECUTIVELY SAMPLE EACH ADC CHANNEL
            A0: SERIES CURRENT SENSOR
            A1: MOISTURE SENSOR 1
            A2: MOISTURE SENSOR 2
            A3: MOISTURE SENSOR 3
            ESP_ADC: VOLTMETER [OPTIONAL]
        */

        for (int i = 0; i<4; i++){
            /* ADS1115 must have the conversion started once the MUX is updated for oneshot conversion mode. */            
            ESP_ERROR_CHECK(ads111x_set_input_mux(&adc_device, mux_select[i]));
            ads111x_start_conversion(&adc_device);

            // wait for conversion end
            bool busy;
            do
            {
                ads111x_is_busy(&adc_device, &busy);
            }
            while (busy);

            if (ads111x_get_value(&adc_device, &adc_raw[i]) == ESP_OK){
                float voltage = adc1115_map_raw_to_voltage(&adc_raw[i], GAIN);
                //ESP_LOGI(TAG, "Raw Read: %i from channel: %i", raw[i], i);

                if (i == 0){
                    current_reading = calculate_current(voltage);
                    ESP_LOGI(TAG, "Current Read: %f A from channel: %d", current_reading, i);

                } else {
                    moisture_percentage[i] = calculate_mositure(voltage); //store moisture in arr of size [3]
                    ESP_LOGI(TAG, "Moisture Read: %f from channel: %d", moisture_percentage[i], i);
                }

                //ESP_LOGI(TAG, "Voltage Read: %f from channel: %d", voltage, i);
            } else {
                ESP_LOGE(TAG, "Failed to read value");
            }
            vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD));
        }

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ESP32_ADC1_C3, &esp32_adc_voltage_raw));
        if (do_calibration_chan3) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan3_handle, esp32_adc_voltage_raw, &esp32_adc_voltage));
            voltage_reading = calculate_current(esp32_adc_voltage/1000.0);
            ESP_LOGI(TAG, "ESP ADC%d Channel[%d] Voltage: %f V",3, ESP32_ADC1_C3, voltage_reading);
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD));
        
        /* calculate average moisture of soil */
        avg_moisture = (moisture_percentage[0]+moisture_percentage[1]+moisture_percentage[2])/3;
        power = current_reading*voltage_reading;

        ESP_LOGI(TAG, "Power Reading: %f W", power);

        /* if moisture is less than threshold, and pump is not open*/
        if(avg_moisture < MOISTURE_THESHOLD && !is_Pump_open){
            ESP_LOGI(TAG, "Opening the Pump");
            is_Pump_open = true;
            xSemaphoreGive(Pump_trigger_semaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD));
                
        /* prepare the readings to be sent to master via espnow */
        bzero(dtmp, ESPNOW_PACKET_BUFFER_SIZE);
        //TODO: The data fields are filled with zeroes, because the sensor modules support different sensors.
        // this module currently only supports moisture control and power drawn.
        espnow_data_prepare(dtmp, avg_moisture, current_reading, voltage_reading,0,0,0,0,0,0,0,0,12);

        esp_err_t send_error = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
        if(send_error == ESP_OK){
            ESP_LOGI(TAG, "Data sent to: "MACSTR, MAC2STR(destination_mac));
        } else{
            ESP_LOGE(TAG, "Send error: %s",esp_err_to_name(send_error));
        } 

        vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD));  // ms/ delay readings
    }
    free(send_param);
    //Tear Down IF TASK EXITS
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

    if (do_calibration_chan3) {
        adc_esp32_calibration_deinit(adc1_cali_chan3_handle);
    }
}

/* SENSOR HELPER FUNCTIONS */

float calculate_mositure(int moisture_reading){
    // Experimentally determined bounds
    float reading = (float)moisture_reading;
    float dry_reading = 2120.0f; 
    float saturated_reading = 910.0f;
    // Calculate normalized value
    float normalized_value = 1-(reading - saturated_reading) / (dry_reading - saturated_reading);

    // Clamp the result to [0, 1]
    if (normalized_value < 0.0f) {
        normalized_value = 0.0f;
    } else if (normalized_value > 1.0f) {
        normalized_value = 1.0f;
    }
    return normalized_value;
}

/* typical current draw 1.3A at 12V*/
/* ACS712 5A LINEAR RELATIONSHIP */
float calculate_current(float adc_voltage){
    return (5*adc_voltage-0.52);
}

/* tolerance pm 5%  [12V source returns 12V reading at ~24C]*/
/* OP-AMP CIRCUIT LINEAR RELATIONSHIP: SEE volt_meter.asc*/
float calculate_voltage(float adc_voltage){
    return(6.66*adc_voltage-8.88);
}