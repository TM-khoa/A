#ifndef OUTPUT_H
#define OUTPUT_H
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "esp_attr.h"
void output_io_create(uint64_t pin_select);
#endif