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

#  define MAX30003_HOST    SPI2_HOST
#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10
#  define PIN_NUM_INTB 0
#  define PIN_BTN 9
#  define NOT_USE 0
static const char TAG[] = "main";
static TaskHandle_t NotiINTB = NULL;
MAX30003_handle_t MAX30003_handle;
// Funtion Prototype *************************************************************
esp_err_t SPI_init_bus();
esp_err_t MAX30003_init_device(MAX30003_handle_t *handle);
esp_err_t ECG_Config_register(MAX30003_handle_t handle);
void MAX30003_INTB_pin_init(bool USE_INT2B,gpio_num_t INTB_pin,gpio_num_t INT2B_pin);
// Funtion Prototype *************************************************************

esp_err_t ECG_Config_register(MAX30003_handle_t handle)
{
    esp_err_t ret;
    EN_INT_t enint = {
        .REG.REG_INTB = REG_ENINTB,
        .E_INT = EN_EINT,
        .E_OVF = NOT_USE,
        .E_FS =NOT_USE,
        .E_DCOFF = EN_DCLOFFINT,
        .E_LON = NOT_USE,
        .E_RR =EN_RRINT,
        .E_SAMP = NOT_USE,
        .E_PLL = NOT_USE,
        .E_INT_TYPE = EN_INTB_OD_PU,
    };
    GEN_t gen = {
        .REG = REG_GEN,
        .ULP_LON = NOT_USE,
        .FMSTR = GEN_FMSTR_32768_512HZ,
        .ECG = GEN_EN_ECG,
        .DCLOFF = GEN_DCLOFF_EN,
        .IPOL = GEN_IPOL_ECGP_PU_ECGN_PD,
        .IMAG = GEN_IMAG_10nA,
        .VTH = GEN_DCLOFF_VTH_300mV,
        .EN_RBIAS = GEN_EN_RBIAS,
        .RBIASV = GEN_RBIASV_100MOHM,
        .RBIASP = GEN_RBIASP_EN,
        .RBIASN = GEN_RBIASN_EN,
    };
    ECG_t ecg = {
        .REG = REG_ECG,
        .RATE = ECG_RATE_1, // 128sps, FMSTR = 00
        .GAIN = ECG_GAIN_160V,
        .DHPF = ECG_DHPF_05HZ,
        .DLPF = ECG_DLPF_40HZ,
    };
    MNGR_INT_t mngrint = {
        .REG = REG_MNGR_INT,
        .EFIT = 0b01111, // Assert EINT w/ 4 unread samples
        .CLR_RRINT = MNGR_INT_CLR_RRINT_RTOR,
    };
    EMUX_t emux ={
        .OPENN = NOT_USE,
        .OPENP = NOT_USE,
        .REG = REG_EMUX,
    };
    MNGR_DYN_t dyn = {
        .REG = REG_MNGR_DYN,
        .FAST = MNGR_DYN_FAST_NORMAL,
    };
    RTOR_t rtor = {
        .REG.RTOR1 = REG_RTOR1,
        .WNDW = RTOR1_WNDW_12,
        .GAIN = RTOR1_GAIN_AUTO,
        .EN = RTOR1_EN_RTOR,
        .PAVG = 0,
        .PTSF = (BIT10 | BIT9),
    };
    MAX30003_config_register_t cfgreg = {
        .EN_INT = &enint,
        .MNGR_INT = &mngrint,
        .DYN = &dyn,
        .GEN = &gen,
        .EMUX = &emux,
        .ECG = &ecg,
        .RTOR = &rtor,
    };
    ret = MAX30003_conf_reg(handle,&cfgreg);
    return ret;
}
static void INTB_ISR(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(NotiINTB,&xHigherPriorityTaskWoken);
}
static void INTB_ISR_Handle(void* arg)
{
    for(;;) {
        if(ulTaskNotifyTake(true,500/portTICK_RATE_MS) == pdTRUE)
            MAX30003_read_FIFO_normal(MAX30003_handle);
        else{
            ESP_LOGE("Err","Timeout");
            MAX30003_write(MAX30003_handle,REG_SYNCH_RST,0);
        }
    }
}
void app_main(void)
{
    gpio_install_isr_service(0);
    SPI_init_bus();
    MAX30003_init_device(&MAX30003_handle);
    xTaskCreate(INTB_ISR_Handle, "INTB_ISR_Handle", 2048, NULL, 10, &NotiINTB);
    MAX30003_INTB_pin_init(0,0,0);
    ECG_Config_register(MAX30003_handle);
    while (1) {
        vTaskDelay(10/portTICK_RATE_MS);
    }
}


esp_err_t MAX30003_init_device(MAX30003_handle_t *handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing device...");
    MAX30003_config_pin_t MAX30003_config_pin = {
        .cs_io = PIN_NUM_CS,
        .host = MAX30003_HOST,
        .miso_io = PIN_NUM_MISO,
    };
    ret = MAX30003_init(&MAX30003_config_pin,handle);
    if(ret == ESP_OK) ESP_LOGI(TAG,"Init done");
    ret = MAX30003_get_info(*handle);
    return ret;
}

esp_err_t SPI_init_bus()
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
    return ret;
}

void MAX30003_INTB_pin_init(bool USE_INT2B,gpio_num_t INTB_pin,gpio_num_t INT2B_pin)
{
    /** 2 chân INTB và INT2B đều là output low nên phải cấu hình ngắt cạnh xuống
     *  trở kéo lên
     */
    gpio_config_t intb_cfg = {
        .pin_bit_mask = BIT(INTB_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&intb_cfg);
    gpio_isr_handler_add(INTB_pin,INTB_ISR,(void*)INTB_pin);
    if(USE_INT2B)
    {
        gpio_config_t int2b_cfg = {
        .pin_bit_mask = BIT(INT2B_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
        };
        gpio_config(&int2b_cfg);
        gpio_isr_handler_add(INT2B_pin,INTB_ISR,(void*)INT2B_pin);
        gpio_intr_enable(INT2B_pin);
    }
}