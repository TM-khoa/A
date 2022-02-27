#ifndef INPUT_H
#define INPUT_H
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "esp_attr.h"
typedef void (*input_callback_t) (int);


esp_err_t input_io_conf(gpio_num_t gpio_num, gpio_int_type_t int_type);
int input_get_level(gpio_num_t gpio_num);
void input_set_callback(void *cb);
#endif