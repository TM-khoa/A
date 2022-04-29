#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "MAX30003.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 This code demonstrates how to use the SPI master full duplex mode to read/write a MAX30003 ECG sensor.
*/
#  define MAX30003_HOST    SPI2_HOST
#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10
#  define PIN_NUM_INTB 8
#  define PIN_NUM_INT2B 9
static const char TAG[] = "main";
static xQueueHandle qINTB = NULL;


// static void IRAM_ATTR INTB2B_ISR(void* arg)
// {
//     gpio_num_t gpio_num = (gpio_num_t) arg;
//     xQueueSendFromISR(qINTB, &gpio_num, NULL);
// }

// static void INTB2B_cb()
// {
//     gpio_num_t gpio_num ;
//     for(;;){
//         gpio_isr
//     }

// }

void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", MAX30003_HOST+1);
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(MAX30003_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    MAX30003_config_pin_t MAX30003_config_pin = {
        .cs_io = PIN_NUM_CS,
        .host = MAX30003_HOST,
        .miso_io = PIN_NUM_MISO,
        .intb = PIN_NUM_INTB,
        .int2b = PIN_NUM_INT2B,
    };
    CAL_t cal = {
        .REG = REG_CAL,
        .FCAL = CAL_FCAL_215,
        .FIFTY = CAL_FIFTY,
        .VCAL = CAL_EN_VCAL,
        .VMAG = CAL_VMAG_025mV,
        .THIGH = 0,
    };
    EMUX_t emux = {
        .REG = REG_EMUX,
        .OPENN = 0,
        .OPENP = 0,
        .CALN = 0,
        .CALP = 0,
        .POL = 0,
    };
    GEN_t gen = {
        .REG = REG_GEN,
        .ECG = 0,
        .ULP_LON = GEN_EN_ULP_LON,
        .FMSTR = GEN_FMSTR_32000_200HZ,
        .IPOL = GEN_IPOL_ECGP_PU_ECGN_PD,
        .IMAG = GEN_IMAG_20nA,
    };
    MAX30003_config_register_t cfgreg = {
        .CAL = &cal,
        .EMUX = &emux,
        .GEN = &gen,
    };
    gpio_install_isr_service(0);
    MAX30003_handle_t MAX30003_handle;

    ESP_LOGI(TAG, "Initializing device...");
    ret = MAX30003_init(&MAX30003_config_pin,&MAX30003_handle);
    if(ret == ESP_OK) ESP_LOGI(TAG,"Init done");
    ret = MAX30003_get_info(MAX30003_handle);
    // qINTB = xQueueCreate(2,sizeof(gpio_num_t));
    // xTaskCreate(INTB2B_cb,"INTB2B_cb",1024,NULL,3,NULL);

    MAX30003_conf_reg(MAX30003_handle,&cfgreg);
    while (1) {
        // Add your main loop handling code here.
        vTaskDelay(1);
    }
}
