#include "MAX30003.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"
static const char TAG[] = "MAX30003";
/// Context (config and data) of MAX30003
struct MAX30003_context_t{
    MAX30003_config_t cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    SemaphoreHandle_t INTB2Bsem; ///< Semaphore for INTB and INT2B ISR
};


typedef struct INTB2B_t INTB2B_t;
typedef struct MAX30003_context_t MAX30003_context_t;

// Workaround: The driver depends on some data in the flash and cannot be placed to DRAM easily for
// now. Using the version in LL instead.
#define gpio_set_level  gpio_set_level_patch
#include "hal/gpio_ll.h"
static inline esp_err_t gpio_set_level_patch(gpio_num_t gpio_num, uint32_t level)
{
    gpio_ll_set_level(&GPIO, gpio_num, level);
    return ESP_OK;
}

static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((MAX30003_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((MAX30003_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((MAX30003_context_t*)t->user)->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((MAX30003_context_t*)t->user)->cfg.cs_io);
}


esp_err_t MAX30003_init(const MAX30003_config_t *cfg, MAX30003_context_t** out_ctx)
{
    esp_err_t err = ESP_OK;
    // Nếu sử dụng SPI interrupt thì không thể sử dụng trên SPI1
    MAX30003_context_t* ctx = (MAX30003_context_t*)malloc(sizeof(MAX30003_context_t));
    if(!ctx) return ESP_ERR_NO_MEM;
    *ctx = (MAX30003_context_t){
        .cfg = *cfg, // đưa tham số config *cfg vào member .cfg của biến ctx vừa tạo
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 1*1000*1000, // speed 1Mhz
        .mode = 0,
        .command_bits = 8,
        .spics_io_num = -1,
        .pre_cb = cs_low, // trước khi bắt đầu truyền dữ liệu, CS ở mức thấp
        .post_cb = cs_high, // sau khi truyền dữ liệu, CS đưa lên mức cao
        .queue_size = 1,
        .input_delay_ns = 500,
        .flags = SPI_DEVICE_POSITIVE_CS,
    };
    //Attach the MAX30003 to the SPI bus
    err = spi_bus_add_device(ctx->cfg.host,&devcfg,&ctx->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }
    /** cấu hình chân CS là output
     *  trở kéo lên
     */
    gpio_set_level(ctx->cfg.cs_io,0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT(ctx->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    /** 2 chân INTB và INT2B đều là output low nên phải cấu hình ngắt cạnh xuống
     *  trở kéo lên
     */
    gpio_config_t intb_cfg = {
        .pin_bit_mask = BIT(ctx->cfg.intb),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&intb_cfg);
    gpio_config_t int2b_cfg = {
        .pin_bit_mask = BIT(ctx->cfg.int2b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&int2b_cfg);
    /** 
     *  gắn hàm xử lý ngắt vào 2 chân INTB và INT2B
     */
    // err = gpio_isr_handler_add(ctx->cfg.intb,INTB2B_ISR,ctx->cfg.intb);
    // if (err != ESP_OK) {
    //         goto cleanup;
    //     }
    // err = gpio_isr_handler_add(ctx->cfg.int2b,INTB2B_ISR,ctx->cfg.int2b);
    // if (err != ESP_OK) {
    //         goto cleanup;
    //     }
    // gpio_intr_enable(ctx->cfg.intb);
    // gpio_intr_enable(ctx->cfg.int2b);
    /** sau khi cấu hình xong, gửi ngược context *ctx về *out_ctx
     *  ra bên ngoài hàm để sử dụng cho mục đích khác
     */
    *out_ctx = ctx;
    return ESP_OK;
cleanup:
    /** gỡ slave khỏi bus, xóa handle dùng để xử lý slave đó 
     *  giải phóng vùng nhớ ctx
     */
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    free(ctx);
    return err;
}

esp_err_t MAX30003_read(MAX30003_context_t *ctx,uint8_t reg, uint32_t *out_data)
{
    spi_transaction_t trans = {
        .cmd = (reg << 1) | 0x01,
        .user = ctx,
        .flags = SPI_TRANS_USE_RXDATA,
        .rxlength = 24,
        .length = 24,
    };
    spi_transaction_t *pTrans = &trans;
    spi_device_queue_trans(ctx->spi,&trans,portMAX_DELAY);
    esp_err_t err = spi_device_get_trans_result(ctx->spi,&pTrans,portMAX_DELAY);
    if(err!= ESP_OK) {
        return err;
    }
    *out_data = trans.rx_data[0] << 16 | trans.rx_data[1] << 8 | trans.rx_data[2];
    return ESP_OK;
}

esp_err_t MAX30003_write(MAX30003_context_t *ctx,uint8_t reg, uint32_t in_data)
{
    esp_err_t err;
    uint8_t DataTemp[4]= {0};
    DataTemp[3] = (in_data >> 16) & 0xff;
    DataTemp[2] = (in_data >> 8) & 0xff;
    DataTemp[1] = in_data & 0xff;
    spi_transaction_t trans = {
        .cmd = (reg << 1) | 0x00,
        .user = ctx,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {DataTemp[3],DataTemp[2],DataTemp[1]},
        .length = 24,
    };
    spi_transaction_t *pTrans = &trans;
    spi_device_queue_trans(ctx->spi,&trans,portMAX_DELAY);
    err = spi_device_get_trans_result(ctx->spi,&pTrans,portMAX_DELAY);
    if(err!= ESP_OK) {
        return err;
    }
    return ESP_OK;
}

esp_err_t MAX30003_INTB2B_callback(MAX30003_context_t *cxt)
{
    return ESP_OK;
}

esp_err_t MAX30003_get_info(MAX30003_context_t *ctx)
{
    esp_err_t err;
    uint32_t info=0;
    err = MAX30003_read(ctx,REG_INFO,&info);
    if(err != ESP_OK){
        ESP_LOGE(TAG,"Not found MAX30003",NULL);
    }
    else{
        if(info & (5 << 24)){
            ESP_LOGI(TAG,"Found MAX30003, info value: %lu",info);
        }
        else{
            ESP_LOGW(TAG,"Uncorrect ID pattern: %lu",info);
        }
    }
}

