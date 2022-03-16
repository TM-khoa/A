#include "output.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <stdio.h>
#include "esp_attr.h"

void output_io_create(uint64_t pin_select){
    gpio_pad_select_gpio(pin_select);
    gpio_set_direction(pin_select,GPIO_MODE_OUTPUT);
}
void output_io_toggle(gpio_num_t pin_select,uint32_t logic)
{
    gpio_set_level(pin_select,1 - logic);
}
