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
 * EFIT[4:0] value will be set inside the code
 */
#define MNGR_INT_EFIT_POS   BIT19 // EFIT position

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
 * FAST_TH value will be set inside the code
 */
#define REG_MNGR_DYN 0x05

/**
 * @brief ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass):
 * 00 = Normal Mode (Fast Recovery Mode Disabled)
 * 01 = Manual Fast Recovery Mode Enable (remains active until disabled)
 * 10 = Automatic Fast Recovery Mode Enable (Fast Recovery automatically activated when/while 
 * ECG outputs are saturated, using FAST_TH)
 * 11 = Reserved. Do not use
 */
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
/** 
 * @name CONFIG CALIBRATION
 * @brief configures the operation, settings, and function of the Internal Calibration Voltage Sources
 * (VCALP and VCALN) The output of the voltage sources can be routed to the ECG inputs through the channel
 * input MUXes to facilitate end-to-end testing operations
 * @note
 */
#define REG_CNFG_CAL 0x12

#define CNFG_CAL_EN_VCAL BIT22 //Calibration Source (VCALP and VCALN) Enable

#define CNFG_CAL_VMODE BIT21 //Calibration Source Mode Selection

#define CNFG_CAL_VMAG BIT20 // Calibration Source Magnitude Selection (VMAG)
/**
 * Calibration Source Frequency Selection (FCAL)
 * 000 = FMSTR/128 (Approximately 256Hz)
 * 001 = FMSTR /512 (Approximately 64Hz)
 * 010 = FMSTR /2048 (Approximately 16Hz)
 * 011 = FMSTR /8192 (Approximately 4Hz)
 * 100 = FMSTR /215 (Approximately 1Hz)
 * 101 = FMSTR /217 (Approximately 1/4Hz)
 * 110 = FMSTR /219 (Approximately 1/16Hz)
 * 111 = FMSTR /221 (Approximately 1/64Hz)
 * 
 */
#define CNFG_CAL_FCAL_2     BIT14
#define CNFG_CAL_FCAL_1     BIT13
#define CNFG_CAL_FCAL_0     BIT12

#define CNFG_CAL_FIFTY      BIT11 // Calibration Source Duty Cycle Mode Selection (1 = THIGH 50%)
// THIGH value will be set inside the code
/**********************************************************************************/
/** 
 * @name CONFIG EMUX
 * @brief CNFG_EMUX is a read/write register which configures the operation, settings, and 
 * functionality of the Input Multiplexer associated with the ECG channel
 */
#define REG_CNFG_EMUX 0x14

#define CNFG_EMUX_POL   BIT23 //ECG Input Polarity Selection (1 = Inverted) 

#define CNFG_EMUX_OPENP BIT21 //Open the ECGP Input Switch (1 = internally isolated from AFE Channel)
#define CNFG_EMUX_OPENP BIT20 //Open the ECGN Input Switch (1 = internally isolated from AFE Channel)

#define CNFG_EMUX_CALP_SEL_NO_CAL   0x00 //No calibration signal applied
#define CNFG_EMUX_CALP_SEL_VMID     0x01 // Input is connected to VMID
#define CNFG_EMUX_CALP_SEL_VCALP    0x02 // Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
#define CNFG_EMUX_CALP_SEL_VCALN    0x03 // Input is connected to VCALN (only available if CAL_EN_VCAL = 1)
/** 
 * ECG calib select position, shift left CNFG_EMUX_CALP_SEL_xx to valid ECGP position or ECGN position
 * (CNFG_EMUX_CALP_SEL_xx << ECGP_CAL_SELECT_POS)
 */
#define ECGP_CAL_SELECT_POS         BIT18 
#define ECGN_CAL_SELECT_POS         BIT16  
/**********************************************************************************/
/** 
 * @name CONFIG ECG
 * @brief configures the operation, settings, and functionality of the ECG channel.
 * Anytime a change to CNFG_ECG is made, there may be discontinuities in the ECG record and possibly 
 * changes to thesize of the time steps recorded in the ECG FIFO
 * 
 */
#define REG_CNFG_ECG        0x15

/**
 * @brief ECG Data Rate (also dependent on FMSTR selection)
 * 
 */
#define CNFG_ECG_RATE_1     BIT23
#define CNFG_ECG_RATE_0     BIT22

/**
 * @brief ECG Channel Gain Setting
 * 
 */
#define GAIN_POS            BIT16
#define CNFG_ECG_GAIN_20V   0
#define CNFG_ECG_GAIN_40V   BIT16
#define CNFG_ECG_GAIN_80V   BIT17
#define CNFG_ECG_GAIN_160V  (BIT17 | BIT16)

#define CNFG_ECG_DHPF       BIT14 // enable 0.5Hz High-Pass Filter through ECG Channel

/**
 * ECG Channel Digital Low-Pass Filter Cutoff Frequency
 * 00 = Bypass (Decimation only, no FIR filter applied)
 * 01 = approximately 40Hz (Except for 125 and 128sps settings) Note: See Table 33.
 * 10 = approximately 100Hz (Available for 512, 256, 500, and 250sps ECG Rate selections only)
 * 11 = approximately 150Hz (Available for 512 and 500sps ECG Rate selections only)
 */
#define CNFG_ECG_DLPF_1     BIT13
#define CNFG_ECG_DLPF_0     BIT12
/**********************************************************************************/
/**
 * @name CONFIG RTOR 
 * @brief: CNFG_RTOR is a two-part read/write register that configures the operation, 
 * settings, and function of the RTOR heart rate detection block. The first register contains 
 * algorithmic voltage gain and threshold parameters, the second contains algorithmic timing parameters.
 * 
 */
#define REG_CNFG_RTOR1  0x1D
#define REG_CNFG_RTOR2  0x1E

/**
 * adjusts the algorithm sensitivity to the width of the QRS complex
 * valid from 0000 to 1011
 * R to R Window Averaging (Window Width = RTOR_WNDW[3:0]*8ms)
 * default 0011 = 12*8 = 96ms
 */
#define CNFG_RTOR_WNDW_3    BIT23
#define CNFG_RTOR_WNDW_2    BIT22
#define CNFG_RTOR_WNDW_1    BIT21
#define CNFG_RTOR_WNDW_0    BIT20

/**
 * @brief R to R Gain (where Gain = 2^GAIN[3:0], plus an auto-scale option). This is used to maximize 
 * the dynamic range of the algorithm.
 * In Auto-Scale mode, the initial gain is set to 64.
 * 
 */
#define CNFG_RTOR_GAIN_3    BIT19
#define CNFG_RTOR_GAIN_2    BIT18
#define CNFG_RTOR_GAIN_1    BIT17
#define CNFG_RTOR_GAIN_0    BIT16

#define CNFG_RTOR_EN        BIT15 // Enable RTOR Dectection

/**
 * @brief R to R Peak Averaging Weight Factor
 * Peak_Average(n) = [Peak(n) + (RTOR_PAVG-1) x Peak_Average(n-1)] / RTOR_PAVG
 */
#define CNFG_RTOR_PAVG_1    BIT13
#define CNFG_RTOR_PAVG_0    BIT12

/**
 * @brief R to R Peak Threshold Scaling Factor
 * This is the fraction of the Peak Average value used in the Threshold computation.
 * Values of 1/16 to 16/16 are selected directly by (RTOR_PTSF[3:0]+1)/16, default is 4/16
 * 
 */
#define CNFG_RTOR_PTFS_3    BIT11      
#define CNFG_RTOR_PTFS_2    BIT10
#define CNFG_RTOR_PTFS_1    BIT9
#define CNFG_RTOR_PTFS_0    BIT8
/**********************************************************************************/
#define ETAG_VALID      0x000
#define ETAG_FAST       0x001
#define ETAG_VALID_EOF  0X010
#define ETAG_FAST_EOF   0X011
#define ETAG_EMPTY      0X110
#define ETAG_OVERFLOW   0X111
/**********************************************************************************/
typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `MAX30003_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `MAX30003_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `MAX30003_init()`
    gpio_num_t intb;        ///< INTB pin of MAX30003
    gpio_num_t int2b;       ///< INT2B pin of MAX30003
} MAX30003_config_t;

typedef struct {

}MAX30003_param_setting_t;

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