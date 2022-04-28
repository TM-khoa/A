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

#define EN_INTB_DISABLED    (BIT1 & BIT0) //00 Disabled
#define EN_INTB_CMOS        BIT0            //01 CMOS Driver
#define EN_INTB_OD          BIT1            //10 Open-Drain NMOS Driver
#define EN_INTB_OD_PU       BIT1 | BIT0     //11 Open-Drain NMOS Driver Internal 125kOhm pullup resistance
/**********************************************************************************/
/**
 * @name MANAGE INTERRUPT REGISTER 
 * @brief Manage operation register of the configurable interrupt 
 * bits in response to ECG FIFO conditions
 */
#define REG_MNGR_INT    0x04

/**
 * @brief ECG FIFO Interrupt Threshold (issues EINT based on number of unread
 * FIFO records)
 * 00000 to 11111 = 1 to 32, respectively (i.e. EFIT[4:0]+1 unread records)
 * EFIT[4:0] value will be set inside the code
 */
#define MNGR_INT_EFIT_POS   BIT19 // EFIT position
#define CHECK_EFIT_VALUE(__VALUE__)    (if(__VALUE__ <= 32))

/**
 * @brief FAST MODE Interrupt Clear
 * 0 = FSTINT remains active until the FAST mode is disengaged (manually or
 * automatically), then held until cleared by STATUS read back (32nd SCLK).
 * 1 = FSTINT remains active until cleared by STATUS read back (32nd SCLK),
 * even if the MAX30003 remains in FAST recovery mode. Once cleared,
 * FSTINT will not be re-asserted until FAST mode is exited and re-entered,
 * either manually or automatically
 */
#define MNGR_INT_CLR_FAST   BIT6


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
 * @brief Sample Synchronization Pulse (SAMP) Clear
 * 0 = Clear SAMP on STATUS Register Read Back (recommended for debug/evaluation only).
 * 1 = Self-clear SAMP after approximately one-fourth of one data rate cycle.
 */
#define MNGR_INT_CLR_SAMP   BIT2

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
 * @name INFOMATION, REVISION ID REGISTER
 * @brief Revision ID readback (read-only)
 */
#define REG_INFO        0x0F
#define RevisionID      (BIT16 | BIT17 | BIT18 | BIT19)
/**********************************************************************************/
/**
 * @name CONFIG GENERAL
 * @brief  which governs general settings, most significantly the master 
 * clock rate for all internal timing operations
 */
#define REG_CNFG_GEN 0x10

/**
 * @brief Ultra-Low Power Lead-On Detection Enable
 * ULP mode is only active when the ECG channel is powered down/disabled.
 */
#define CNFG_GEN_EN_ULP_LON BIT22 

/**
 * @brief Master Clock Frequency
 * Selects the Master Clock Frequency (FMSTR), and Timing Resolution (TRES), 
 * which also determines the ECG and CAL timing characteristics.These are generated 
 * from FCLK, which is always 32.768Khz
 * 
 */
#define CNFG_GEN_FMSTR_32768_512HZ (BIT21 & BIT20)
#define CNFG_GEN_FMSTR_32000_500HZ  BIT20
#define CNFG_GEN_FMSTR_32000_200HZ  BIT21
#define CNFG_GEN_FMSTR_31968_199HZ  BIT21 | BIT20


#define CNFG_GEN_EN_ECG BIT19 // ECG Channel Enable

/**
 * @brief DC Lead-Off Detection Enable
 * DC Method, requires active selected channel, enables DCLOFF interrupt 
 * and status bit behavior.
 * Uses current sources and comparator thresholds set below.
 * 
 */
#define CNFG_GEN_EN_DCLOFF BIT12 

/**
 * @brief DC Lead-Off Current Polarity (if current sources are enabled/connected)
 * 0 = ECGP - Pullup ECGN – Pulldown 
 * 1 = ECGP - Pulldown ECGN – Pullup
 * 
 */
#define CNFG_GEN_DCLOFF_IPOL BIT11

/**
 * @brief DC Lead-Off Current Magnitude Selection
 * 000 = 0nA (Disable and Disconnect Current Sources)
 * 001 = 5nA
 * 010 = 10nA
 * 011 = 20nA
 * 100 = 50nA
 * 101 = 100nA
 */
#define CNFG_GEN_IMAG_2 BIT10
#define CNFG_GEN_IMAG_1 BIT9
#define CNFG_GEN_IMAG_0 BIT8


/**
 * @brief DC Lead-Off Voltage Threshold Selection
 * 00 = VMID ± 300mV
 * 01 = VMID ± 400mV
 * 10 = VMID ± 450mV
 * 11 = VMID ± 500mV
 */
#define CNFG_GEN_DCLOFF_VTH_1 BIT7
#define CNFG_GEN_DCLOFF_VTH_0 BIT6

/**
 * @brief Enable and Select Resistive Lead Bias Mode
 * 00 = Resistive Bias disabled
 * 01 = ECG Resistive Bias enabled if EN_ECG is also enabled
 * If EN_ECG is not asserted at the same time as prior to EN_RBIAS[1:0] being set to
 * 01, then EN_RBIAS[1:0] will remain set to 00
 */
#define CNFG_GEN_EN_RBIAS BIT4
/**********************************************************************************/
/** 
 * @name CONFIG CALIBRATION
 * @brief configures the operation, settings, and function of the Internal Calibration Voltage Sources
 * (VCALP and VCALN) The output of the voltage sources can be routed to the ECG inputs through the channel
 * input MUXes to facilitate end-to-end testing operations
 */
#define REG_CNFG_CAL 0x12

/**
 * @brief Calibration Source (VCALP and VCALN) Enable
 * 0 = Calibration sources and modes disabled
 * 1 = Calibration sources and modes enabled
 */
#define CNFG_CAL_EN_VCAL BIT22 //Calibration Source (VCALP and VCALN) Enable

/**
 * @brief Calibration Source Mode Selection
 * 0 = Unipolar, sources swing between VMID ± VMAG and VMID
 * 1 = Bipolar, sources swing between VMID + VMAG and VMID - VMAG
 */
#define CNFG_CAL_VMODE BIT21 //Calibration Source Mode Selection

/**
 * @brief Calibration Source Magnitude Selection (VMAG)
 * 0 = 0.25mV
 * 1 = 0.50mV
 */
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
 * Actual frequencies are determined by FMSTR selection (see CNFG_GEN for
 * details), approximate frequencies are based on a 32768Hz clock (FMSTR[2:0] = 000).
 * TCAL = 1/FCAL.
 */
#define CNFG_CAL_FCAL_2     BIT14
#define CNFG_CAL_FCAL_1     BIT13
#define CNFG_CAL_FCAL_0     BIT12


/**
 * @brief Calibration Source Duty Cycle Mode Selection
 * 0 = Use CAL_THIGH to select time high for VCALP and VCALN
 * 1 = THIGH = 50% (CAL_THIGH[10:0] are ignored)
 */
#define CNFG_CAL_FIFTY      BIT11 

/**
 * @brief Calibration Source Time High Selection
 * THIGH = THIGH[10:0] x CAL_RES
 * CAL_RES is determined by FMSTR selection (see CNFG_GEN for details)
 * if FMSTR[2:0] = 000,CAL_RES = 30.52µs.
 * THIGH value will be set inside the code
 */
#define CNFG_CAL_THIGH_POS ~BIT0

/**********************************************************************************/
/** 
 * @name CONFIG EMUX
 * @brief CNFG_EMUX is a read/write register which configures the operation, settings, and 
 * functionality of the Input Multiplexer associated with the ECG channel
 */
#define REG_CNFG_EMUX 0x14

/**
 * @brief ECG Input Polarity Selection
 * 0 = Non-inverted
 * 1 = Inverted
 */
#define CNFG_EMUX_POL   BIT23 //ECG Input Polarity Selection (1 = Inverted) 

/**
 * @brief Open the ECGP Input Switch (most often used for testing and calibration studies)
 * 0 = ECGP is internally connected to the ECG AFE Channel
 * 1 = ECGP is internally isolated from the ECG AFE Channel
 */
#define CNFG_EMUX_OPENP BIT21 
#define CNFG_EMUX_OPENN BIT20 

/**
 * @brief ECGN Calibration Selection
 * 00 = No calibration signal applied
 * 01 = Input is connected to VMID
 * 10 = Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
 * 11 = Input is connected to VCALN (only available if CAL_EN_VCAL = 1)
 * ECG calib select position, shift left CNFG_EMUX_CALP_SEL_xx to valid 
 * ECGP position or ECGN position (CNFG_EMUX_CAL_SEL_xx << CNFG_EMUX_CAL_SEL_POS_ECGN)
 */
#define CNFG_EMUX_CAL_SEL_NO_CAL   (BIT1 & BIT2) 
#define CNFG_EMUX_CAL_SEL_VMID     BIT1 
#define CNFG_EMUX_CAL_SEL_VCALP    BIT2 
#define CNFG_EMUX_CAL_SEL_VCALN    (BIT1 | BIT2) 
#define CNFG_EMUX_CAL_SEL_POS_ECGP         BIT18 
#define CNFG_EMUX_CAL_SEL_POS_ECGN         BIT16  
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
 * FMSTR = 00: fMSTR = 32768Hz, tRES = 15.26µs (512Hz ECG progressions)
 * 00 = 512sps
 * 01 = 256sps
 * 10 = 128sps
 * FMSTR = 01: fMSTR = 32000Hz, tRES = 15.63µs (500Hz ECG progressions)
 * 00 = 500sps
 * 01 = 250sps
 * 10 = 125sps
 * FMSTR = 10: fMSTR = 32000Hz, tRES = 15.63µs (200Hz ECG progressions)
 * 10 = 200sps
 * FMSTR = 11: fMSTR = 31968Hz, tRES = 15.64µs (199.8Hz ECG progressions)
 * 10 = 199.8sps
 */
#define CNFG_ECG_RATE_1     BIT23
#define CNFG_ECG_RATE_0     BIT22

/**
 * @brief ECG Channel Gain Setting
 * 00 = 20V/V
 * 01 = 40V/V
 * 10 = 80V/V
 * 11 = 160V/V
 */
#define CNFG_ECG_GAIN_20V   (BIT17 & BIT16)
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
 * 
 * @brief: CNFG_RTOR is a two-part read/write register that configures the operation, 
 * settings, and function of the RTOR heart rate detection block. The first register contains 
 * algorithmic voltage gain and threshold parameters, the second contains algorithmic timing parameters.
 * 
 */
#define REG_CNFG_RTOR1  0x1D
#define REG_CNFG_RTOR2  0x1E


/**
 * @brief This is the width of the averaging window, which adjusts the algorithm sensitivity to
 * the width of the QRS complex.
 * R to R Window Averaging (Window Width = RTOR_WNDW[3:0]*8ms)
 * 0000 = 6
 * 0001 = 8
 * 0010 = 10
 * 0011 = 12 (default = 96ms)
 * 0100 = 14
 * 0101 = 16
 * 0110 = 18
 * 0111 = 20
 * 1000 = 22
 * 1001 = 24
 * 1010 = 26
 * 1011 = 28
 */
#define CNFG_RTOR1_WNDW_3    BIT23
#define CNFG_RTOR1_WNDW_2    BIT22
#define CNFG_RTOR1_WNDW_1    BIT21
#define CNFG_RTOR1_WNDW_0    BIT20

/**
 * @brief R to R Gain (where Gain = 2^GAIN[3:0], plus an auto-scale option). This is used to maximize 
 * the dynamic range of the algorithm.
 * In Auto-Scale mode, the initial gain is set to 64.
 * 0000 = 1, 1000 = 256
 * 0001 = 2, 1001 = 512
 * 0010 = 4, 1010 = 1024
 * 0011 = 8, 1011 = 2048
 * 0100 = 16, 1100 = 4096
 * 0101 = 32, 1101 = 8192
 * 0110 = 64, 1110 = 16384
 * 0111 = 128, 1111 = Auto-Scale (default)
 */
#define CNFG_RTOR1_GAIN_3    BIT19
#define CNFG_RTOR1_GAIN_2    BIT18
#define CNFG_RTOR1_GAIN_1    BIT17
#define CNFG_RTOR1_GAIN_0    BIT16

/**
 * @brief ECG RTOR Detection Enable
 * 0 = RTOR Detection disabled
 * 1 = RTOR Detection enabled if EN_ECG is also enabled
 */
#define CNFG_RTOR1_EN_RTOR        BIT15 

/**
 * @brief R to R Peak Averaging Weight Factor
 * Peak_Average(n) = [Peak(n) + (RTOR_PAVG-1) x Peak_Average(n-1)] / RTOR_PAVG
 * 00 = 2
 * 01 = 4
 * 10 = 8 (default)
 * 11 = 16
 * Peak_Average(n) = [Peak(n) + (RTOR_PAVG-1) x Peak_Average(n-1)] / RTOR_PAVG
 */
#define CNFG_RTOR1_PAVG_1    BIT13
#define CNFG_RTOR1_PAVG_0    BIT12

/**
 * @brief R to R Peak Threshold Scaling Factor
 * This is the fraction of the Peak Average value used in the Threshold computation.
 * Values of 1/16 to 16/16 are selected directly by (RTOR_PTSF[3:0]+1)/16, default is 4/16
 * PTFS will be set inside the code
 * Shift left value with CNFG_RTOR_PTFS_POS position
 */
#define CNFG_RTOR1_PTSF_POS    BIT8

/**
 * @brief R to R Minimum Hold Off
 * This sets the absolute minimum interval used for the 
 * static portion of the Hold Off criteria.
 * Values of 0 to 63 are supported, default is 32
 * tHOLD_OFF_MIN = HOFF[5:0] * tRTOR, where tRTOR is ~8ms, as determined by
 * FMSTR[1:0] in the CNFG_GEN register (representing approximately ¼ second).
 * The R to R Hold Off qualification interval is tHold_Off = MAX(tHold_Off_Min, tHold_Off_Dyn)
 * Shift left value with CNFG_RTOR2_HOFF_POS position
 */
#define CNFG_RTOR2_HOFF_POS BIT16

/**
 * @brief R to R Interval Averaging Weight Factor
 * This is the weighting factor for the current RtoR interval observation vs. the past interval
 * observations when determining dynamic holdoff criteria. Lower numbers weight current
 * intervals more heavily.
 * 00 = 2
 * 01 = 4
 * 10 = 8 (default)
 * 11 = 16
 * Interval_Average(n) = [Interval(n) + (RAVG-1) x Interval_Average(n-1)] / RAVG
 */
#define CNFG_RTOR2_RAVG_1 BIT13
#define CNFG_RTOR2_RAVG_0 BIT12


/**
 * @brief R to R Interval Hold Off Scaling Factor
 * This is the fraction of the RtoR average interval used for the dynamic portion of the holdoff
 * criteria (tHOLD_OFFDYN).
 * Values of 0/8 to 7/8 are selected directly by RTOR_RHSF[3:0]/8, default is 4/8.
 * If 000 (0/8) is selected, then no dynamic factor is used and the holdoff criteria is
 * determined by HOFF[5:0] only 
 * Shift left value with CNFG_RTOR2_RHSF_POS position
 */
#define CNFG_RTOR2_RHSF_POS BIT8

/**********************************************************************************/
#define REG_ECG_BURST   0x20
#define REG_ECG_NORMAL  0x21
#define REG_RTOR        0x25

#define ETAG_VALID      0
#define ETAG_FAST       1
#define ETAG_VALID_EOF  2
#define ETAG_FAST_EOF   3
#define ETAG_EMPTY      6
#define ETAG_OVERFLOW   7
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
/**
 * @brief Check if MAX30003 present on SPI bus
 *
 * @param handle Context of MAX30003 communication.
 *  @return . 
 *  - ESP_OK: on success
 *  - ESP_ERR_INVALID_RESPONSE: Received data from MAX30003 but nibble bits[20:23] is not 0101
 *  - ESP_ERR_TIMEOUT: no respond from MAX30003 
 */
esp_err_t MAX30003_get_revID(MAX30003_handle_t handle);
/**********************************************************************************/
/**
 * @brief Read ECG data on normal mode
 *
 * @param handle Context of MAX30003 communication.
 */
esp_err_t MAX30003_read_FIFO_normal(MAX30003_handle_t handle);
/**********************************************************************************/
/**
 * @brief Read RTOR 
 *
 * @param handle Context of MAX30003 communication.
 */
esp_err_t MAX30003_read_RTOR(MAX30003_handle_t handle);
/**********************************************************************************/
/**
 * @brief check ETAG status
 *
 * @param ECG_Data: get value by reading register 0x21 or 0x20.
 */
void MAX30003_check_ETAG(uint32_t ECG_Data);
#endif
