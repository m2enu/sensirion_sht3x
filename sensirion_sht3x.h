/**
 * @file sensirion_sht3x.h
 * @brief Sensirion SHT3x Temperature & Humidity Sensor control via I2C
 * @author m2enu
 * @date 31/07/2017
 */
#ifndef SENSIRION_SHT3X_INCLUDED
#define SENSIRION_SHT3X_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

// name C standard macros {{{1
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

// I2C device address definitions {{{1
#define SHT3X_I2C_PRIMARY       0x44 //!< ADDR(pin2) connected to VSS: default
#define SHT3X_I2C_SECONDARY     0x45 //!< ADDR(pin2) connected to VDD

// Measurement commands for single shot data acquisition mode {{{1
#define SHT3X_SINGLE_HI_CSEN    0x2c06 //!< Repeatability High  , Clock stretching enable
#define SHT3X_SINGLE_MD_CSEN    0x2c0d //!< Repeatability Medium, Clock stretching enable
#define SHT3X_SINGLE_LO_CSEN    0x2c10 //!< Repeatability Low   , Clock stretching enable
#define SHT3X_SINGLE_HI_CSDE    0x2400 //!< Repeatability High  , Clock stretching disable
#define SHT3X_SINGLE_MD_CSDE    0x240b //!< Repeatability Medium, Clock stretching disable
#define SHT3X_SINGLE_LO_CSDE    0x2416 //!< Repeatability Low   , Clock stretching disable

// wait time after measurement {{{1
#define SHT3X_MSEC_WAIT_LO      9  //!< wait time for Repeatablility Low
#define SHT3X_MSEC_WAIT_MD      11 //!< wait time for Repeatablility Midium
#define SHT3X_MSEC_WAIT_HI      20 //!< wait time for Repeatablility High

// CRC-8 calculation {{{1
#define SHT3X_CRC8_POLY         0x31 //!< CRC-8 poly x8 + x5 + x4 + 1
#define SHT3X_CRC8_INIT         0xff //!< CRC-8 initial value

// error code definition {{{1
enum SHT3X_ERRCODE {
    SHT3X_SUCCESS               = 0, //!< NO ERROR

    EROR_SHT3X_INIT             = -1, //!< SHT3X initialization failed
    EROR_SHT3X_NULL_PTR         = -2, //!< SHT3X validation failed
    EROR_SHT3X_I2C_NACK_WRITE   = -11, //!< Nack occured when Writing I2C
    EROR_SHT3X_I2C_NACK_READ    = -12, //!< Nack occured when Reading I2C
    EROR_SHT3X_I2C_CRC_TEMP     = -21, //!< invalid Temperature CRC
    EROR_SHT3X_I2C_CRC_HUMI     = -22, //!< invalid Humidity CRC
    EROR_SHT3X_OTHERS           = -127, //!< Other error

    WARN_SHT3X_OTHRES           = 127, //!< Other warning
};
#define I2C_OK                  0 //!< I2C master return code: SUCCESS

// device function definitions {{{1
typedef int8_t (*sht3x_com_fptr_t)(uint8_t dev_id, uint8_t* data, uint8_t len);
typedef void (*sht3x_wait_fptr_t)(uint32_t msec_wait);

// uncompensated SHT3x data structure {{{1
typedef struct sht3x_uncomp_data_t_tag {
    uint16_t temperature; //!< uncompensated temperature
    uint16_t humidity; //!< uncompensated humidity
} sht3x_uncomp_data_t;

// conmensated SHT3x data structure {{{1
typedef struct sht3x_data_t_tag {
    float temperature; //!< compensated temperature
    float humidity; //!< compensated humidity
} sht3x_data_t;

// SHT3x device structure {{{1
typedef struct sht3x_t_tag {
    bool fvalid; //!< true: connection valid, false: connection invalid
    uint8_t addr_wr; //!< I2C device address for write
    uint8_t addr_rd; //!< I2C device address for read
    uint16_t cmd_single; //!< measurement command for single shot acquisition
    uint32_t msec_wait; //!< wait time after measurement in milliseconds

    sht3x_uncomp_data_t crc; //<! CRC-8 data (divert data structure)
    sht3x_uncomp_data_t uncomp; //!< uncompensated data
    sht3x_data_t comp; //!< compensated data

    sht3x_com_fptr_t i2c_rd; //!< I2C read function pointer
    sht3x_com_fptr_t i2c_wr; //!< I2C write function pointer
    sht3x_wait_fptr_t wait_ms; //!< wait function pointer
} sht3x_t;

// function declarations {{{1
void sht3x_clear(sht3x_t*);
void sht3x_clear_uncomp(sht3x_uncomp_data_t*);
void sht3x_clear_comp(sht3x_data_t*);
int8_t sht3x_error_check(sht3x_t*);
int8_t sht3x_null_ptr_check(sht3x_t*);
int8_t sht3x_init(sht3x_t*, uint8_t, uint16_t, uint32_t);
int8_t sht3x_meas(sht3x_t*);
int8_t sht3x_read(sht3x_t*);
int8_t sht3x_calc(sht3x_data_t*, sht3x_uncomp_data_t*);
int8_t sht3x_run(sht3x_t*);
uint8_t sht3x_crc(uint8_t*, uint32_t);

#if defined(__cplusplus)
} // extern "C"
#endif

#endif // SENSIRION_SHT3X_INCLUDED {{{1

// end of file {{{1
// vim:ft=cpp:et:nowrap:fdm=marker
