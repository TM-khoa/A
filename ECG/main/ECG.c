#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "MAX30003.h"

#  define MAX30003_HOST    SPI2_HOST
#  define PIN_NUM_MISO 1
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10
#  define PIN_INTB 0
#  define PIN_BTN 2
#  define PIN_SIGNAL 5
#  define NOT_USE 0
#  define BAT_MIN_VOLT 3400
#  define BAT_MAX_VOLT 4100
#  define BAT_PERCENT_OFFSET 1
#  define ASSERT_BITS(__NUM__,__BIT__) (((__NUM__) & (__BIT__)) == __BIT__ ? 1 : 0)
#define EVTGR_TASK_INTB     (1 << 0)
#define EVTGR_TASK_BTN     (1 << 1)
#define EVTGR_TASK_BATTERY  (1 << 2)
#define EVTGR_LIGHT_SLEEP   (1 << 3)
#define EVTGR_FRESH_START   (1 << 4)

#define INTB_USE_TRANSISTOR_BUFFER

#ifdef INTB_USE_TRANSISTOR_BUFFER 
#define INTB_LIGHT_SLEEP_LOGIC_LEVEL GPIO_INTR_HIGH_LEVEL
#define INTB_ISR_EDGE GPIO_INTR_POSEDGE
#define INTB_PULL_SELECT GPIO_PULLDOWN_ENABLE
#elif
#define INTB_LIGHT_SLEEP_LOGIC_LEVEL GPIO_INTR_LOW_LEVEL
#define INTB_ISR_EDGE GPIO_INTR_NEGEDGE
#define INTB_PULL_SELECT GPIO_PULLUP_ENABLE
#endif
static const char TAG[] = "main";




EventGroupHandle_t xEvtGr1;
static esp_adc_cal_characteristics_t adc1_chars;
MAX30003_handle_t MAX30003_handle;
// Funtion Prototype *************************************************************
esp_err_t SPI_init_bus();
esp_err_t MAX30003_init_device(MAX30003_handle_t *handle);
esp_err_t ECG_Config_register(MAX30003_handle_t handle);


esp_err_t MAX30003Config_register_ECG_on(MAX30003_handle_t handle);
esp_err_t MAX30003Config_register_ULP(MAX30003_handle_t handle);

esp_err_t MAX30003Config_register_Spiritboi2();

static void INTB_handle(void* arg);
void PIN_BTN_ISR(void *arg);
static void BTN_handle(void* arg);
static void esp_check_light_sleep(void* arg);

void light_sleep_init();
void log_cause_wakeup();

void ADC1_init();
void read_battery(void* pvParameter);

// Funtion Prototype *************************************************************
void app_main(void)
{
    gpio_install_isr_service(0);
    ADC1_init();
    xEvtGr1 = xEventGroupCreate();
    light_sleep_init();
    // SPI_init_bus();
    // MAX30003_init_device(&MAX30003_handle);
    // MAX30003Config_register_ECG_on(MAX30003_handle);
    MAX30003Config_register_Spiritboi2();
    xEventGroupSetBits(xEvtGr1,EVTGR_FRESH_START);
    gpio_set_level(PIN_SIGNAL,1);
    vTaskDelay(500/portTICK_RATE_MS);
    gpio_set_level(PIN_SIGNAL,0);
    xTaskCreate(INTB_handle, "INTB_handle", 2048, NULL, 10,NULL);
    xTaskCreate(BTN_handle, "BTN_handle", 2048, NULL, 7,NULL);
    xTaskCreate(esp_check_light_sleep, "esp_check_light_sleep", 2048, NULL, 1, NULL);
    xTaskCreate(read_battery, "read_battery", 2048, NULL, 8, NULL);
    
    while (1) {
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

static void BTN_handle(void* arg)
{
    EventBits_t xEvtVal;
    for(;;) {
        xEvtVal = xEventGroupWaitBits(xEvtGr1,EVTGR_TASK_BTN,pdTRUE,pdFALSE,portMAX_DELAY);
        if(ASSERT_BITS(xEvtVal,EVTGR_TASK_BTN)){
            ESP_LOGI("EVT BTN","done");
        }
    }
}

static void INTB_handle(void* arg)
{
    EventBits_t xEvtVal;
    for(;;) {
        xEvtVal = xEventGroupWaitBits(xEvtGr1,EVTGR_TASK_INTB,pdTRUE,pdFALSE,portMAX_DELAY);
    }
}




static void esp_check_light_sleep(void* arg)
{
#ifndef INTB_USE_TRANSISTOR_BUFFER   
EventBits_t xEvtVal;
const EventBits_t xBitsToWaitSleep = (EVTGR_FRESH_START | EVTGR_LIGHT_SLEEP);
const EventBits_t xBitsToWaitTaskDone = (EVTGR_TASK_BATTERY | EVTGR_TASK_BTN | EVTGR_TASK_INTB);
    for(;;) {
        xEvtVal = xEventGroupWaitBits(xEvtGr1,xBitsToWaitSleep,pdTRUE,pdFALSE,500/portTICK_RATE_MS);
        if(ASSERT_BITS(xEvtVal,EVTGR_LIGHT_SLEEP) || ASSERT_BITS(xEvtVal,EVTGR_FRESH_START)){
            gpio_set_level(PIN_SIGNAL,0);
            esp_light_sleep_start();
            gpio_set_level(PIN_SIGNAL,1);
            xEventGroupSetBits(xEvtGr1,EVTGR_TASK_BATTERY);
            switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_TIMER:
                ESP_LOGW("LIGHTWAKE","Timeout");
                xEventGroupSetBits(xEvtGr1,EVTGR_LIGHT_SLEEP);
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                if(!gpio_get_level(PIN_BTN)){
                    ESP_LOGI("LIGHTWAKE","BTN");
                    xEventGroupSetBits(xEvtGr1,EVTGR_TASK_BTN);
                }
                if(!gpio_get_level(PIN_INTB)){
                    ESP_LOGI("LIGHTWAKE","INTB");
                }
                break;
            default:
                ESP_LOGW("LIGHTWAKE","Unknown source");
                break;
            }
        }
        xEvtVal = xEventGroupWaitBits(xEvtGr1,xBitsToWaitTaskDone,pdTRUE,pdFALSE,500/portTICK_RATE_MS); 
        if( !ASSERT_BITS(xEvtVal,EVTGR_TASK_BATTERY) && 
            !ASSERT_BITS(xEvtVal,EVTGR_TASK_BTN) && 
            !ASSERT_BITS(xEvtVal,EVTGR_TASK_INTB)){
            ESP_LOGI("SET BIT","Lightwake");
            xEventGroupSetBits(xEvtGr1,EVTGR_LIGHT_SLEEP);
        }
    }
#endif
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

esp_err_t MAX30003Config_register_ECG_on(MAX30003_handle_t handle)
{

    EN_INT_t enint = {
        .REG.REG_INTB = REG_ENINTB,
        .E_INT = NOT_USE,
        .E_OVF = NOT_USE,
        .E_FS =NOT_USE,
        .E_DCOFF = EN_DCLOFFINT,
        .E_LON = NOT_USE,
        .E_RR =EN_RRINT,
        .E_SAMP = NOT_USE,
        .E_PLL = NOT_USE,
        .E_INT_TYPE = EN_INTB_CMOS,
    };
    MNGR_INT_t mngrint = {
        .REG = REG_MNGR_INT,
        .EFIT = 0b01111, // Assert EINT w/ 4 unread samples
        .CLR_RRINT = MNGR_INT_CLR_RRINT_RTOR,
    };
    MNGR_DYN_t dyn = {
        .REG = REG_MNGR_DYN,
        .FAST = MNGR_DYN_FAST_NORMAL,
    };
    GEN_t gen = {   
        .REG = REG_GEN,
        .ULP_LON = NOT_USE,
        .FMSTR = GEN_FMSTR_32768_512HZ,
        .ECG = GEN_EN_ECG,
        .DCLOFF = GEN_DCLOFF_EN,
        .IPOL = GEN_IPOL_ECGP_PU_ECGN_PD,
        .IMAG = GEN_IMAG_100nA,
        .VTH = GEN_DCLOFF_VTH_300mV,
        .EN_RBIAS = GEN_EN_RBIAS,
        .RBIASV = GEN_RBIASV_200MOHM,
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
    RTOR_t rtor = {
        .REG.RTOR1 = REG_RTOR1,
        .WNDW = RTOR1_WNDW_12,
        .GAIN = RTOR1_GAIN_AUTO,
        .EN = NOT_USE,
        .PAVG = 0,
        .PTSF = (BIT10 | BIT9),
    };
    MAX30003_config_register_t cfgreg = {
        .EN_INT = &enint,
        .MNGR_INT = &mngrint,
        .DYN = &dyn,
        .GEN = &gen,
        // .CAL = &cal,
        // .EMUX = &emux,
        .ECG = &ecg,
        .RTOR = &rtor,
    };
    MAX30003_conf_reg(handle,&cfgreg);
    return ESP_OK;
}

esp_err_t MAX30003Config_register_ULP(MAX30003_handle_t handle)
{

    EN_INT_t enint = {
        .REG.REG_INTB = REG_ENINTB,
        .E_INT = NOT_USE,
        .E_OVF = NOT_USE,
        .E_FS =NOT_USE,
        .E_DCOFF = NOT_USE,
        .E_LON = EN_LONINT,
        .E_RR =NOT_USE,
        .E_SAMP = NOT_USE,
        .E_PLL = NOT_USE,
        .E_INT_TYPE = EN_INTB_CMOS,
    };
    GEN_t gen = {   
        .REG = REG_GEN,
        .ULP_LON = GEN_EN_ULP_LON,
        .FMSTR = GEN_FMSTR_32768_512HZ,
        .ECG = NOT_USE,
        .DCLOFF = NOT_USE,
        .IPOL = GEN_IPOL_ECGP_PU_ECGN_PD,
        .IMAG = GEN_IMAG_100nA,
        .VTH = GEN_DCLOFF_VTH_300mV,
        .EN_RBIAS = NOT_USE,
        .RBIASV = GEN_RBIASV_200MOHM,
        .RBIASP = GEN_RBIASP_EN,
        .RBIASN = GEN_RBIASN_EN,
    };
    CAL_t cal = {
        .REG = REG_CAL,
        .VCAL = NOT_USE,
        .VMODE = CAL_VMODE_BIPOLAR,
        .VMAG = CAL_VMAG_050mV,
        .FCAL = NOT_USE,
        .FIFTY = CAL_FIFTY,
        .THIGH = NOT_USE,
    };
    EMUX_t emux ={
        .OPENN = EMUX_OPENN,
        .OPENP = EMUX_OPENP,
        .CALP = EMUX_CALP_SEL_VCALN,
        .CALN = EMUX_CALN_SEL_VCALP,
        .REG = REG_EMUX,
    };
    MAX30003_config_register_t cfgreg = {
        .EN_INT = &enint,
        .GEN = &gen,
        .CAL = &cal,
        .EMUX = &emux,
    };
    MAX30003_conf_reg(handle,&cfgreg);
    return ESP_OK;
}

void PIN_BTN_ISR(void *arg)
{
    BaseType_t HigherPriorityTaskWoken;
    xEventGroupSetBitsFromISR(xEvtGr1,EVTGR_TASK_BTN,&HigherPriorityTaskWoken);
}
void light_sleep_init()
{
    gpio_config_t io_cfg = {};
    io_cfg.pin_bit_mask = BIT(PIN_SIGNAL);
    io_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_cfg);
    
    io_cfg.pin_bit_mask = BIT(PIN_BTN);
    io_cfg.mode = GPIO_MODE_INPUT;
    gpio_config(&io_cfg);

    io_cfg.pin_bit_mask = BIT(PIN_INTB);
    io_cfg.mode = GPIO_MODE_INPUT;
#ifdef INTB_USE_TRANSISTOR_BUFFER
    io_cfg.pull_down_en = INTB_PULL_SELECT;
#elif
    io_cfg.pull_up_en = INTB_PULL_SELECT;
#endif
    io_cfg.intr_type = INTB_ISR_EDGE;
    gpio_config(&io_cfg);
    gpio_isr_handler_add(PIN_BTN,PIN_BTN_ISR,NULL);

    gpio_wakeup_enable(PIN_BTN,GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable(PIN_INTB,INTB_LIGHT_SLEEP_LOGIC_LEVEL);
    esp_sleep_enable_timer_wakeup(5000000);
    esp_sleep_enable_gpio_wakeup();
}


void read_battery(void* pvParameter)
{
#define NUM_OF_SAMPLES 25
    
    EventBits_t xEvtVal;
     
    for(;;){
        int adc_raw=0;
        unsigned int voltage = 0;
        uint8_t bat_percent=0;
        xEvtVal = xEventGroupWaitBits(xEvtGr1,EVTGR_TASK_BATTERY,pdTRUE,pdFALSE,portMAX_DELAY);
        if(ASSERT_BITS(xEvtVal,EVTGR_TASK_BATTERY)){
            for(uint8_t i=0;i<NUM_OF_SAMPLES;i++){
                adc_raw += adc1_get_raw(ADC1_CHANNEL_4);
            }
            voltage = esp_adc_cal_raw_to_voltage(adc_raw/NUM_OF_SAMPLES,&adc1_chars);
            bat_percent = (((float)voltage*2 - BAT_MIN_VOLT)/(BAT_MAX_VOLT-BAT_MIN_VOLT))*100;
            ESP_LOGI("BatPercent", "%u%%", bat_percent);
        }
    }
}

void ADC1_init()
{
    esp_err_t ret;
    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11));
}















esp_err_t MAX30003Config_register_Spiritboi2()
{
    ESP_LOGI("Before malloc","%d",esp_get_free_heap_size()); 
    MAX30003_config_register_t_2 *cfgreg2 = (MAX30003_config_register_t_2*)malloc(sizeof(MAX30003_config_register_t_2));
    if(!cfgreg2) return ESP_ERR_NO_MEM;
    cfgreg2->EN_INT.REG.REG_INTB = REG_ENINTB;
    cfgreg2->EN_INT.E_OVF =NOT_USE;
    cfgreg2->EN_INT.E_INT = NOT_USE;
    cfgreg2->EN_INT.E_OVF = NOT_USE;
    cfgreg2->EN_INT.E_FS =NOT_USE;
    cfgreg2->EN_INT.E_DCOFF = EN_DCLOFFINT;
    cfgreg2->EN_INT.E_LON = EN_LONINT;
    cfgreg2->EN_INT.E_RR =EN_RRINT;
    cfgreg2->EN_INT.E_SAMP = NOT_USE;
    cfgreg2->EN_INT.E_PLL = NOT_USE;
    cfgreg2->EN_INT.E_INT_TYPE = EN_INTB_CMOS;

    cfgreg2->MNGR_INT.REG = REG_MNGR_INT;
    cfgreg2->MNGR_INT.EFIT = 0b01111; // Assert EINT w/ 4 unread samples
    cfgreg2->MNGR_INT.CLR_RRINT = MNGR_INT_CLR_RRINT_RTOR;


    cfgreg2->GEN.REG = REG_GEN;
    cfgreg2->GEN.ULP_LON = GEN_EN_ULP_LON;
    cfgreg2->GEN.FMSTR = GEN_FMSTR_32768_512HZ;
    cfgreg2->GEN.ECG = GEN_EN_ECG;
    cfgreg2->GEN.DCLOFF = GEN_DCLOFF_EN;
    cfgreg2->GEN.IPOL = GEN_IPOL_ECGP_PU_ECGN_PD;
    cfgreg2->GEN.IMAG = GEN_IMAG_100nA;
    cfgreg2->GEN.VTH = GEN_DCLOFF_VTH_300mV;
    cfgreg2->GEN.EN_RBIAS = GEN_EN_RBIAS;
    cfgreg2->GEN.RBIASV = GEN_RBIASV_200MOHM;
    cfgreg2->GEN.RBIASP = GEN_RBIASP_EN;
    cfgreg2->GEN.RBIASN = GEN_RBIASN_EN;
    
    cfgreg2->CAL.REG = REG_CAL;
    cfgreg2->CAL.VCAL = NOT_USE;
    cfgreg2->CAL.VMODE = CAL_VMODE_BIPOLAR;
    cfgreg2->CAL.VMAG = CAL_VMAG_050mV;
    cfgreg2->CAL.FCAL = NOT_USE;
    cfgreg2->CAL.FIFTY = CAL_FIFTY;
    cfgreg2->CAL.THIGH = NOT_USE;
    
    cfgreg2->EMUX.REG = REG_EMUX;
    cfgreg2->EMUX.OPENN = NOT_USE;
    cfgreg2->EMUX.OPENP = NOT_USE;
    cfgreg2->EMUX.CALP = EMUX_CALP_SEL_VCALN;
    cfgreg2->EMUX.CALN = EMUX_CALN_SEL_VCALP;
    
    cfgreg2->ECG.REG = REG_ECG;
    cfgreg2->ECG.RATE = ECG_RATE_1; // 128sps; FMSTR = 00
    cfgreg2->ECG.GAIN = ECG_GAIN_160V;
    cfgreg2->ECG.DHPF = ECG_DHPF_05HZ;
    cfgreg2->ECG.DLPF = ECG_DLPF_40HZ;
    
    cfgreg2->RTOR.REG.RTOR1 = REG_RTOR1;
    cfgreg2->RTOR.WNDW = RTOR1_WNDW_12;
    cfgreg2->RTOR.GAIN = RTOR1_GAIN_AUTO;
    cfgreg2->RTOR.EN = NOT_USE;
    cfgreg2->RTOR.PAVG = 0;
    cfgreg2->RTOR.PTSF = (BIT10 | BIT9);
    ESP_LOGI("After malloc","%d",esp_get_free_heap_size()); 
    free(cfgreg2);
    ESP_LOGI("After free","%d",esp_get_free_heap_size()); 
    return ESP_OK;
}

