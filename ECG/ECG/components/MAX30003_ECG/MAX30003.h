#ifndef _MAX30003_H
#define _MAX30003_H

#pragma once
#include "driver/spi_master.h"
#include "esp_bit_defs.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
/**********************************************************************************/
/**
 * @name STATUS REGISTER
 * @brief Register Status and its bits
 */
#define REG_STATUS          0x01
#define STATUS_EINT         BIT23 //ECG FIFO Interrupt
#define STATUS_EOVF         BIT22 //ECG FIFO Overflow
#define STATUS_FSINT        BIT21 //ECG Fast Recovery Mode
#define STATUS_DCLOFF_INT   BIT20 //DC Lead-Off Dectect Interrupt
#define STATUS_LONINT       BIT11 //Ultra-Low Power Leads On Dectection Interrupt
#define STATUS_RRINT        BIT10 //R to R Detector R Event Interrupt
#define STATUS_SAMP         BIT9  //R to R Detector R Event Interrupt
#define STATUS_PLLINT       BIT8  //PLL Unlocked Interrupt
#define STATUS_LDOFF_PH     BIT3  //ECGP is above the high threshold
#define STATUS_LDOFF_PL     BIT2  //ECGP is below the low threshold
#define STATUS_LDOFF_NH     BIT1  //ECGN is above the high threshold
#define STATUS_LDOFF_NL     BIT0  //ECGN is below the low threshold
/**********************************************************************************/
/**
 * @name ENABLE INTERRUPT REGISTER
 * @brief Register that govern the operation of the INTB output and INT2B output
 */
#define REG_ENINTB       0x02
#define REG_ENINT2B      0x03
#define EN_EINT         BIT23
#define EN_EOVF         BIT22
#define EN_FSTINT       BIT21 
#define EN_DCLOFFINT    BIT20
#define EN_LONINT       BIT11
#define EN_RRINT        BIT10
#define EN_SAMP         BIT9
#define EN_PPLINT       BIT8
#define EN_INTB_DISABLED ~(BIT1 | BIT0) //00 Disabled
#define EN_INTB_CMOS    BIT0            //01 CMOS Driver
#define EN_INTB_OD      BIT1            //10 Open-Drain NMOS Driver
#define EN_INTB_OD_PU   BIT1 | BIT0     //11 Open-Drain NMOS Driver Internal 125kOhm pullup resistance
/**********************************************************************************/
/**
 * @name INFOMATION, REVISION ID REGISTER
 * @brief Revision ID readback (read-only)
 */
#define REG_INFO        0x0F
#define RevisionID      (BIT16 | BIT17 | BIT18 | BIT19)

/**********************************************************************************/
/**
 * @name MANAGE INTERRUPT REGISTER 
 * @brief Manage operation register of the configurable interrupt 
 * bits in response to ECG FIFO conditions
 */
#define REG_MNGR_INT    0x04
/**
 * @name EFIT[4:0]
 * @brief ECG FIFO Interrupt Threshold (issues EINT based on number of unread
 * FIFO records)
 * 00000 to 11111 = 1 to 32, respectively (i.e. EFIT[4:0]+1 unread records)
 * EFIT[4:0] is not defined but set inside the code
 */
#define MNGR_INT_CLR_FAST   BIT6
#define MNGR_INT_CLR_SAMP   BIT2
/** @brief RTOR R Detect Interrupt (RRINT) Clear Behavior:
 *  CLR_RRINT[1:0]:
 *  00 = Clear RRINT on STATUS Register Read Back
 *  01 = Clear RRINT on RTOR Register Read Back
 *  10 = Self-Clear RRINT after one ECG data rate cycle, approximately 2ms to 8ms
 *  11 = Reserved. Do not use
 */
#define MNGR_INT_CLR_RRINT_1    BIT5
#define MNGR_INT_CRL_RRINT_0    BIT4
/** 
 * @brief Sample Synchronization Pulse (SAMP) Frequency
 * 00 = issued every sample instant
 * 01 = issued every 2nd sample instant
 * 10 = issued every 4th sample instant
 * 11 = issued every 16th sample instant
 */
#define MNGR_INT_SAMP_IT_1      BIT1
#define MNGR_INT_SAMP_IT_0      BIT0
/**********************************************************************************/
/** 
 * @name MANAGE GENERAL/DYNAMIC MODE
 * @brief Manages the settings of any general/dynamic modes within the device
 * 
 * FAST[1:0]
 * ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass):
 * 00 = Normal Mode (Fast Recovery Mode Disabled)
 * 01 = Manual Fast Recovery Mode Enable (remains active until disabled)
 * 10 = Automatic Fast Recovery Mode Enable (Fast Recovery automatically
 * activated when/while ECG outputs are saturated, using FAST_TH).
 * 11 = Reserved. Do not use.
 * 
 * FAST_TH is not defined but set inside the code
 */
#define REG_MNGR_DYN 0x05
#define MNGR_DYN_FAST_1 BIT23
#define MNGR_DYN_FAST_0 BIT22
/**********************************************************************************/
/** 
 * @name SOFTWARE RESET REGISTER
 * @name SYNCHRONIZE RESET REGISTER
 * @name FIFO RESET REGISTER
 */
#define REG_SW_RST 0x08
#define REG_SYNCH_RST 0x09
#define REG_FIFO_RST 0x0A
/**********************************************************************************/
typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `MAX30003_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `MAX30003_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `MAX30003_init()`
    gpio_num_t intb;        ///< INTB pin of MAX30003
    gpio_num_t int2b;       ///< INT2B pin of MAX30003
} MAX30003_config_t;

typedef struct MAX30003_context_t* MAX30003_handle_t;
/**********************************************************************************/
/**
 * @brief Initialize the hardware
 *
 * @param config Configuration of the MAX30003
 * @param out_handle Output context of MAX30003 communication
 * @return
 *  - ESP_OK: on success
 *  - ESP_ERR_INVALID_ARG: If the configuration in the context is incorrect.
 *  - ESP_ERR_NO_MEM: if semaphore create failed.
 *  - or other return value from `spi_bus_add_device()` or `gpio_isr_handler_add()`.
 */
esp_err_t MAX30003_init(const MAX30003_config_t *cfg, MAX30003_handle_t *out_handle);
/**********************************************************************************/
/**
 * @brief Read a register 24 bits from the MAX30003.
 *
 * @param handle Context of MAX30003 communication.
 * @param reg      register to read.
 * @param out_data  Buffer to output 24 bits the read data.
 * @return return value from `spi_device_get_trans_result()`.
 */
esp_err_t MAX30003_read(MAX30003_handle_t handle,uint8_t reg, uint32_t *out_data);
/**********************************************************************************/
/**
 * @brief Write 3 bytes(3,2,1) into the MAX30003
 *
 * @param handle Context of MAX30003 communication.
 * @param reg  register to write.
 * @param in_data  buffer to store 24 bits command of register to write data, 
 * using OR operation to set command
 * 
 *  @return . 
 *  - ESP_OK: on success
 *  - ESP_ERR_TIMEOUT: if the EEPROM is not able to be ready before the time in the spec. This may mean that the connection is not correct.
 *  - or return value from `spi_device_acquire_bus()` `spi_device_polling_transmit()`.
 */
esp_err_t MAX30003_write(MAX30003_handle_t handle,uint8_t reg, uint32_t in_data);
/**********************************************************************************/

#endif