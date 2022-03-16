/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "output.h"
#include "input.h"
#include "freertos/event_groups.h"

xTimerHandle xTimers[2];
#define BlueLED 5
#define RedLED 3
#define GreenLED 4
#define BtnBuiltIn 9

#define SELECT_OUTPUT_PIN GreenLED
#define SELECT_OUTPUT_PIN2 RedLED
#define EVT_BTN_PRESS (1 << 0)
#define EVT_UART_RECV (1 << 1)

EventGroupHandle_t xCreatedEventGroup;
void task_EVTGRP_handle(void *vParameter)
{
    for(;;){
        EventBits_t uxBits = xEventGroupWaitBits(
                                                xCreatedEventGroup,
                                                EVT_BTN_PRESS | EVT_UART_RECV,
                                                pdTRUE,  // Clear evt bit before returning
                                                pdFALSE,
                                                portMAX_DELAY //wait
                                                                    );
        if(uxBits & EVT_BTN_PRESS){
            printf("BtnPress \n");
        }
        if(uxBits & EVT_UART_RECV){
            
        }
    }
    
}

static void IRAM_ATTR BtnCallback(int pin)
{
    if(pin == BtnBuiltIn){
        BaseType_t pxHigherPriorityTaskWoken;
        xEventGroupSetBitsFromISR(xCreatedEventGroup,EVT_BTN_PRESS, &pxHigherPriorityTaskWoken); 
    }
}

void vTimerCallback(TimerHandle_t xTimer)
{
    static uint32_t cnt;
    configASSERT(xTimer);
    uint32_t ulCount;
    ulCount = (uint32_t) pvTimerGetTimerID(xTimer);
    if(ulCount == 0){
        cnt++;
        gpio_set_level(SELECT_OUTPUT_PIN2,cnt%2);
        printf("TimerBlink Hello \n");
    }
    else if(ulCount == 1){
        cnt++;
        gpio_set_level(SELECT_OUTPUT_PIN,cnt%2);
        printf("TimerPrint Hello \n");
    }
}

void app_main(void)
{
    xTimers[0] = xTimerCreate("TimerBlink",pdMS_TO_TICKS(500),pdTRUE,(void *) 0,vTimerCallback);
    xTimers[1] = xTimerCreate("TimerPrint",pdMS_TO_TICKS(1000),pdTRUE,(void *) 1,vTimerCallback);
    output_io_create(SELECT_OUTPUT_PIN);
    output_io_create(SELECT_OUTPUT_PIN2);
    
    input_io_conf(BtnBuiltIn,GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BtnBuiltIn, BtnCallback, (void*)BtnBuiltIn);

    xCreatedEventGroup = xEventGroupCreate();
    if(xCreatedEventGroup != NULL) printf("EvtGrp was created \n"); 
    else {printf("EvtGrp was not created \n");}
    xTaskCreate(task_EVTGRP_handle,"event group task",1024,NULL,4,NULL);
    for(uint8_t i=0; i < 2;i++)
    {
        if(xTimers[i] != NULL)
            if(xTimerStart(xTimers[i],0) == pdPASS){
                printf("Timer %d pass \n",i);
            }
    }
    
}
