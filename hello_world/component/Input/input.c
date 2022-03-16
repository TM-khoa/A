#include "input.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <stdio.h>
#include "esp_attr.h"

esp_err_t input_io_conf(gpio_num_t gpio_num, gpio_int_type_t int_type)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = int_type;
    io_conf.pin_bit_mask = 1ULL << gpio_num;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    return ESP_OK;
}
int input_get_level(gpio_num_t gpio_num)
{
    return gpio_get_level(gpio_num);
}