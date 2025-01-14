/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
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
#include "adc.h"
const static char *TAG = "ADC_ALTERNATE";


/* --------------------------------------------------------------
        1. ESP32 ON-board ADC
        2. ADS1115
---------------------------------------------------------------*/
/*---------------------------------------------------------------
       1.  ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels

//-------------ADC1 Init---------------//
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};

//-------------ADC1 Config---------------//
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};

adc_cali_handle_t adc1_cali_chan0_handle = NULL;
adc_cali_handle_t adc1_cali_chan1_handle = NULL;
adc_cali_handle_t adc1_cali_chan2_handle = NULL;
adc_cali_handle_t adc1_cali_chan3_handle = NULL;

bool do_calibration1_chan0 = true; 
bool do_calibration1_chan1 = true; 
bool do_calibration1_chan2 = true;
bool do_calibration_chan3 = true;

//-------------ADC1 Calibration Init---------------//
/* SINGLE USE FOR VOLTAGE DIVIDER VOLTMETER*/
void adc_esp32_init(void){

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ESP32_ADC1_C3, &config));
    do_calibration1_chan1 = adc_esp32_calibration_init(ADC_UNIT_1, ESP32_ADC1_C3, EXAMPLE_ADC_ATTEN, &adc1_cali_chan3_handle);
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_esp32_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_esp32_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

/*---------------------------------------------------------------
        1. ADS1115 General Macros
---------------------------------------------------------------*/

#include <ads111x.h>

ads111x_mux_t mux_select[4] = {
    ADS111X_MUX_0_GND,   //!< positive = AIN0, negative = GND
    ADS111X_MUX_1_GND,   //!< positive = AIN1, negative = GND
    ADS111X_MUX_2_GND,   //!< positive = AIN2, negative = GND
    ADS111X_MUX_3_GND,   //!< positive = AIN3, negative = GND
};

i2c_dev_t adc_device;
ads111x_mode_t mode;

void adc1115_init(){

    ESP_LOGI(TAG, "Initializing I2C driver");
    ESP_ERROR_CHECK(i2cdev_init()); //INIT LIBRARY
    ESP_LOGI(TAG, "Initializing ADS111x descriptor");
    ESP_ERROR_CHECK(ads111x_init_desc(&adc_device, ADDR_SELECT,I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    //adc_device->cfg.scl_pullup_en = true;   
    ESP_LOGI(TAG, "Setting ADS111x data rate");
    ESP_ERROR_CHECK(ads111x_set_data_rate(&adc_device, ADS111X_DATA_RATE_32));
    ESP_LOGI(TAG, "Setting ADS111x input mux");
    ESP_ERROR_CHECK(ads111x_set_input_mux(&adc_device, ADS111X_MUX_0_GND));  // positive = AIN0, negative = GND
    ESP_LOGI(TAG, "Setting ADS111x gain");  
    ESP_ERROR_CHECK(ads111x_set_gain(&adc_device, GAIN));

}

void adc1115_log_mode(ads111x_mode_t mode) {
    const char *mode_str;

    // Map enum to string
    switch (mode) {
        case ADS111X_MODE_CONTINUOUS:
            mode_str = "Continuous conversion mode";
            break;
        case ADS111X_MODE_SINGLE_SHOT:
            mode_str = "Single-shot mode (default)";
            break;
        default:
            mode_str = "Unknown mode";
            break;
    }

    // LOG the mode
    ESP_LOGI(TAG, "ADS111x Operational Mode: %s\n", mode_str);
}

// Map raw ADC value to actual voltage
float adc1115_map_raw_to_voltage(int16_t raw, ads111x_gain_t gain) {
    float full_scale_voltage = 0;

    // Determine the full-scale voltage based on the gain
    switch (gain) {
        case ADS111X_GAIN_6V144:
            full_scale_voltage = 6.144;
            break;
        case ADS111X_GAIN_4V096:
            full_scale_voltage = 4.096;
            break;
        case ADS111X_GAIN_2V048:
            full_scale_voltage = 2.048;
            break;
        case ADS111X_GAIN_1V024:
            full_scale_voltage = 1.024;
            break;
        case ADS111X_GAIN_0V512:
            full_scale_voltage = 0.512;
            break;
        case ADS111X_GAIN_0V256:
            full_scale_voltage = 0.256;
            break;
        default:
            full_scale_voltage = 2.048; // Default to ±2.048V
            break;
    }

    // Map the raw 16 bit ADC value to a voltage
    return ((float)raw / 32768.0) * full_scale_voltage;
}