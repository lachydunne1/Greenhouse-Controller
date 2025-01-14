#ifndef ADC_H
#define ADC_H
/* --------------------------------------------------------------

    Please note: this header is defined for two seperate ADCS.

        1. ESP32 ON-board ADC
        2. ADS1115

    [2] uses i2c and is an independent 16bit  Sigma-Delta ADC, offering much 
    higher accuracy then the on-board 12bit ESP32 SAR ADC. General macros are defined
    within this header file for [2], however the driver for the ADS111x family is used 
    for main functionality. This driver is a part of the library found at https://github.com/UncleRus/esp-idf-lib.

---------------------------------------------------------------*/

/* ADC's are currently defined for devkit1 esp32
    ADCS: CHAN0 = GPIO33
          CHAN1 = GPIO39
          CHAN2 = GPIO34
          CHAN3 = GPIO35
    note this is done so that each channel is consecutively
    laid out on the output pins.
*/
#define ESP32_ADC1_C3       ADC_CHANNEL_3
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

// Declare ADC global variables
extern adc_oneshot_unit_handle_t adc1_handle;
extern adc_oneshot_unit_init_cfg_t init_config1;
extern adc_oneshot_chan_cfg_t config;


extern adc_cali_handle_t adc1_cali_chan3_handle;
extern bool do_calibration_chan3;


void adc_esp32_init(void);
static bool adc_esp32_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_esp32_calibration_deinit(adc_cali_handle_t handle);
/*---------------------------------------------------------------
        1. ADS1115 General Macros
---------------------------------------------------------------*/

#include <ads111x.h>

#define ADDR_SELECT ADS111X_ADDR_GND
#define GAIN ADS111X_GAIN_4V096 // +-4.096V range?
#define ADS111X_MAX_VALUE 3.3
#define ESP32 0 // if using ESP32, set to 1 for i2c pins 22,21, else ESP32-S3, pins 8,9

#if ESP32
    #define I2C_MASTER_SCL_IO GPIO_NUM_22   // Set the GPIO number for SCL (adjust for your setup)
    #define I2C_MASTER_SDA_IO GPIO_NUM_21    // Set the GPIO number for SDA (adjust for your setup)
#else 
    #define I2C_MASTER_SCL_IO GPIO_NUM_9    
    #define I2C_MASTER_SDA_IO GPIO_NUM_8
#endif

#define I2C_MASTER_NUM 0// I2C port number for master

//static allocation avoids memory leakage via dynamic allocation
extern ads111x_mux_t mux_select[4];
extern i2c_dev_t adc_device; // Declare an i2c_dev_t structure
extern ads111x_mode_t mode;

void adc1115_init(void);
void adc1115_log_mode(ads111x_mode_t mode);
float adc1115_map_raw_to_voltage(int16_t raw, ads111x_gain_t gain);

#endif //ADC_H