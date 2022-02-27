#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "input.h"

#define BlueLED 5
#define RedLED 3
#define GreenLED 4
#define BtnGPIO 9

static const char *TAG = "input_intr";
static xQueueHandle gpio_evt_queue = NULL;
static void IRAM_ATTR input_event_callback(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    static uint8_t count;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            gpio_set_level(BlueLED, (count%2) ? 1:0);
            gpio_set_level(GreenLED,(count%2) ? 0:1);
            count++;
        }
    }
}

void app_main(void)
{
    static int x = 0;
    gpio_pad_select_gpio(BlueLED);
    gpio_pad_select_gpio(RedLED);
    gpio_pad_select_gpio(GreenLED);
    gpio_set_direction(GreenLED,GPIO_MODE_OUTPUT);
    gpio_set_direction(RedLED,GPIO_MODE_OUTPUT);
    gpio_set_direction(BlueLED,GPIO_MODE_OUTPUT);

    input_io_conf(BtnGPIO,GPIO_INTR_POSEDGE);
    //change gpio intrrupt type for one pin
    gpio_set_intr_type(BtnGPIO, GPIO_INTR_NEGEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BtnGPIO, input_event_callback, (void*)BtnGPIO);
    while(1){
        x = 1 - x;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // gpio_set_level(BlueLED,x);
        gpio_set_level(RedLED,x);
        // gpio_set_level(GreenLED,x);
    }
}
