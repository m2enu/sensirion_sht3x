/**
 * @file sensirion_sht3x.h
 * @brief Sensirion SHT3x Temperature & Humidity Sensor control via I2C
 * @author m2enu
 * @date 31/07/2017
 */
#if !defined(__SENSIRION_SHT3X__)
#define __SENSIRION_SHT3X__

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// name C standard macros
#ifndef NULL
#ifdef __cplusplus
#define NULL   0  //!< NULL definition
#else
#define NULL   ((void *) 0)  //!< NULL definition
#endif
#endif

/** <!-- {{{1 --> @brief I2C device address definitions
 */
typedef enum sht3x_i2c_address_t_tag {
    SHT3X_I2C_PRIMARY           = 0x44, //!< ADDR(pin2) connected to VSS: default
    SHT3X_I2C_SECONDARY         = 0x45, //!< ADDR(pin2) connected to VDD
} sht3x_i2c_address_t;

/** <!-- {{{1 --> @brief Measurement commands for single shot data acquisition mode
 */
typedef enum sht3x_command_t_tag {
    SHT3X_SINGLE_HI_CSEN        = 0x2c06, //!< Repeatability High  , Clock stretching enable
    SHT3X_SINGLE_MD_CSEN        = 0x2c0d, //!< Repeatability Medium, Clock stretching enable
    SHT3X_SINGLE_LO_CSEN        = 0x2c10, //!< Repeatability Low   , Clock stretching enable
    SHT3X_SINGLE_HI_CSDE        = 0x2400, //!< Repeatability High  , Clock stretching disable
    SHT3X_SINGLE_MD_CSDE        = 0x240b, //!< Repeatability Medium, Clock stretching disable
    SHT3X_SINGLE_LO_CSDE        = 0x2416, //!< Repeatability Low   , Clock stretching disable
} sht3x_command_t;

/** <!-- {{{1 --> @brief wait time after measurement
 */
typedef enum sht3x_wait_time_t_tag {
    SHT3X_MSEC_WAIT_LO          = 9,  //!< wait time for Repeatablility Low
    SHT3X_MSEC_WAIT_MD          = 11, //!< wait time for Repeatablility Midium
    SHT3X_MSEC_WAIT_HI          = 20, //!< wait time for Repeatablility High
} sht3x_wait_time_t;

/** <!-- {{{1 --> @brief error code definition
 */
typedef enum sht3x_errcode_t_tag {
    SHT3X_SUCCESS               = 0, //!< NO ERROR

    EROR_SHT3X_INIT             = -1, //!< SHT3X initialization failed
    EROR_SHT3X_NULL_PTR         = -2, //!< SHT3X validation failed
    EROR_SHT3X_I2C_NACK_WRITE   = -11, //!< Nack occured when Writing I2C
    EROR_SHT3X_I2C_NACK_READ    = -12, //!< Nack occured when Reading I2C
    EROR_SHT3X_I2C_CRC_TEMP     = -21, //!< invalid Temperature CRC
    EROR_SHT3X_I2C_CRC_HUMI     = -22, //!< invalid Humidity CRC
    EROR_SHT3X_OTHERS           = -127, //!< Other error

    WARN_SHT3X_OTHRES           = 127, //!< Other warning
} sht3x_errorcode_t;

/** <!-- {{{1 --> @brief definition of I2C write function
 */
typedef int8_t (*sht3x_com_w_fptr_t)(uint8_t dev_id, const uint8_t* data, uint8_t len);

/** <!-- {{{1 --> @brief definition of I2C read function
 */
typedef int8_t (*sht3x_com_r_fptr_t)(uint8_t dev_id, uint8_t* data, uint8_t len);

/** <!-- {{{1 --> @brief definition of wait function
 */
typedef void (*sht3x_wait_fptr_t)(uint32_t msec_wait);

/** <!-- {{{1 --> @brief uncompensated SHT3x data structure
 */
typedef struct sht3x_uncomp_data_t_tag {
    uint16_t temperature; //!< uncompensated temperature
    uint16_t humidity; //!< uncompensated humidity
} sht3x_uncomp_data_t;

/** <!-- {{{1 --> @brief conmensated SHT3x data structure
 */
typedef struct sht3x_data_t_tag {
    float temperature; //!< compensated temperature
    float humidity; //!< compensated humidity
} sht3x_data_t;

/** <!-- {{{1 --> @brief SHT3x device structure
 */
typedef struct sht3x_t_tag {
    bool fvalid; //!< true: connection valid, false: connection invalid
    uint8_t addr_wr; //!< I2C device address for write
    uint8_t addr_rd; //!< I2C device address for read
    uint16_t cmd_single; //!< measurement command for single shot acquisition
    uint32_t msec_wait; //!< wait time after measurement in milliseconds

    sht3x_uncomp_data_t crc; //!< CRC-8 data (divert data structure)
    sht3x_uncomp_data_t uncomp; //!< uncompensated data
    sht3x_data_t comp; //!< compensated data

    sht3x_com_r_fptr_t i2c_rd; //!< I2C read function pointer
    sht3x_com_w_fptr_t i2c_wr; //!< I2C write function pointer
    sht3x_wait_fptr_t wait_ms; //!< wait function pointer
} sht3x_t;

/** <!-- {{{1 --> @brief initialization of SHT3x device
 * @param[out] _this SHT3x device structure
 * @param[in] devaddr I2C device address of SHT3x
 * @param[in] cmd_single measurement command for single shot acquisition
 * @param[in] msec_wait wait time after measurement in milliseconds
 * @return result of initialization
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
int8_t sht3x_init(sht3x_t*, sht3x_i2c_address_t, sht3x_command_t, sht3x_wait_time_t);

/** <!-- {{{1 --> @brief run measurement and readout sequence
 * @param[out] _this device structure
 * @return result of run
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
int8_t sht3x_run(sht3x_t*);

#if defined(__cplusplus)
} // extern "C"
#endif

#endif // endif for __SENSIRION_SHT3X__ {{{1

// end of file {{{1
// vim:ft=cpp:et:nowrap:fdm=marker
