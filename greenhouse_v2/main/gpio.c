#include "driver/gpio.h"
#include "gpio.h"

void init_gpio(){
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SW1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SW2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SW3_GPIO, GPIO_MODE_OUTPUT);
}
