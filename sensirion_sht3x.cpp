/**
 * @file sensirion_sht3x.cpp
 * @brief Sensirion SHT3x Temperature & Humidity Sensor control via I2C
 * @author m2enu
 * @date 31/07/2017
 */
#include "sensirion_sht3x.h"

extern "C" {

// CRC-8 calculation {{{1
#define SHT3X_CRC8_POLY         (uint8_t)0x31 //!< CRC-8 poly x8 + x5 + x4 + 1
#define SHT3X_CRC8_INIT         (uint8_t)0xff //!< CRC-8 initial value

// I2C communication return code
#define I2C_OK                  0 //!< I2C master return code: SUCCESS

// private function declarations {{{1
static int8_t sht3x_clear(sht3x_t*);
static int8_t sht3x_clear_uncomp(sht3x_uncomp_data_t*);
static int8_t sht3x_clear_comp(sht3x_data_t*);
static int8_t sht3x_error_check(const sht3x_t*);
static int8_t sht3x_null_ptr_check(const sht3x_t*);
static int8_t sht3x_meas(const sht3x_t*);
static int8_t sht3x_read(sht3x_t*);
static int8_t sht3x_calc(sht3x_data_t*, const sht3x_uncomp_data_t*);
static uint8_t sht3x_crc(const uint8_t*, uint32_t);

/** <!-- {{{1 --> @brief clear SHT3X device structure
 * @param[out] _this SHT3x device structure
 * @return result of clear
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_clear(sht3x_t* _this)
{
    _this->fvalid = false;
    _this->addr_wr = 0x00;
    _this->addr_rd = 0x01;
    _this->cmd_single = SHT3X_SINGLE_LO_CSDE;
    _this->msec_wait = SHT3X_MSEC_WAIT_HI;

    sht3x_clear_uncomp(&_this->crc);
    sht3x_clear_uncomp(&_this->uncomp);
    sht3x_clear_comp(&_this->comp);

    return SHT3X_SUCCESS;
}

/** <!-- {{{1 --> @brief clear uncompensated SHT3X data
 * @param[out] uncomp uncompensated SHT3x data
 * @return result of clear
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_clear_uncomp(sht3x_uncomp_data_t* uncomp)
{
    uncomp->temperature = 0;
    uncomp->humidity = 0;
    return SHT3X_SUCCESS;
}

/** <!-- {{{1 --> @brief clear compensated SHT3X data
 * @param[out] comp compensated SHT3x data
 * @return result of clear
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_clear_comp(sht3x_data_t* comp)
{
    comp->temperature = 0;
    comp->humidity = 0;
    return SHT3X_SUCCESS;
}

/** <!-- {{{1 --> @brief SHT3X device error check
 * @param[out] _this SHT3x device structure
 * @return result of error check
 * @retval 0: no error
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_error_check(const sht3x_t* _this)
{
    if (!_this->fvalid) {
        return EROR_SHT3X_INIT;
    }
    return sht3x_null_ptr_check(_this);
}

/** <!-- {{1 --> @brief internal API which used to validate the device pointer for null
 * conditions.
 * @param[in] _this device structure
 * @return result of validation
 * @retval zero: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_null_ptr_check(const sht3x_t* _this)
{
    if ((_this == NULL) || (_this->i2c_rd == NULL) ||
        (_this->i2c_wr == NULL) || (_this->wait_ms == NULL)) {
        return EROR_SHT3X_NULL_PTR;
    }
    return SHT3X_SUCCESS;
}

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
int8_t sht3x_init(sht3x_t* _this, sht3x_i2c_address_t devaddr, sht3x_command_t cmd_single, sht3x_wait_time_t msec_wait)
{
    // clear device and store arguments
    sht3x_clear(_this);
    _this->addr_wr = ((uint8_t)devaddr << 1) | 0x00;
    _this->addr_rd = ((uint8_t)devaddr << 1) | 0x01;
    _this->cmd_single = (uint16_t)cmd_single;
    _this->msec_wait = (uint32_t)msec_wait;
    _this->fvalid = true; // temporary for measurement and readout

    // try I2C communication to check established connection
    int8_t err = sht3x_run(_this);
    if (err != SHT3X_SUCCESS) {
        _this->fvalid = false;
        return EROR_SHT3X_INIT;
    }
    return SHT3X_SUCCESS;
}

/** <!-- {{{1 --> @brief single shot data acquisition
 * @param[out] _this SHT3x device structure
 * @return result of data acquisition
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_meas(const sht3x_t* _this)
{
    int8_t err;
    // device pointer error check
    err = sht3x_error_check(_this);
    if (err != SHT3X_SUCCESS) {
        return err;
    }
    // activate one-shot measurement
    uint8_t cmd[2];
    cmd[0] = (_this->cmd_single & 0xff00) >> 8;
    cmd[1] = (_this->cmd_single & 0x00ff) >> 0;
    err = _this->i2c_wr(_this->addr_wr, cmd, 2);
    if (err != I2C_OK) {
        return EROR_SHT3X_I2C_NACK_WRITE;
    }
    return SHT3X_SUCCESS;
}

/** <!-- {{{1 --> @brief Readout of measurement results for single shot mode
 * @param[out] _this SHT3x device structure
 * @return result of readout
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_read(sht3x_t* _this)
{
    int8_t err;
    // device pointer error check
    err = sht3x_error_check(_this);
    if (err != SHT3X_SUCCESS) {
        return err;
    }
    // read out measurement results
    uint8_t dat[6];
    err = _this->i2c_rd(_this->addr_rd, dat, 6);
    if (err != I2C_OK) {
        return EROR_SHT3X_I2C_NACK_READ;
    }
    // checksum
    _this->crc.temperature = (uint16_t)sht3x_crc(dat + 0, 2);
    _this->crc.humidity    = (uint16_t)sht3x_crc(dat + 3, 2);
    uint16_t crc_temp_ans = (uint16_t)dat[2];
    uint16_t crc_humi_ans = (uint16_t)dat[5];
    if (_this->crc.temperature != crc_temp_ans) {
        return EROR_SHT3X_I2C_CRC_TEMP;
    } else if (_this->crc.humidity != crc_humi_ans) {
        return EROR_SHT3X_I2C_CRC_HUMI;
    }
    // store read data
    // TODO: datasheet says data is big endian,
    // but the device returns little endian
    _this->uncomp.temperature = ((uint16_t)dat[0] << 8) | (uint16_t)dat[1];
    _this->uncomp.humidity    = ((uint16_t)dat[3] << 8) | (uint16_t)dat[4];
    // compensation
    return sht3x_calc(&_this->comp, &_this->uncomp);
}

/** <!-- {{{1 --> @brief temperature and humidity compensation
 * @param[out] comp compensated SHT3x data
 * @param[in] uncomp uncompensated SHT3x data
 * @return result of compensation
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
static int8_t sht3x_calc(sht3x_data_t* comp, const sht3x_uncomp_data_t* uncomp)
{
    comp->temperature = -45.0 + (175.0 * (float)uncomp->temperature) / 65535.0;
    comp->humidity = 100.0 * (float)uncomp->humidity / 65535.0;
    return SHT3X_SUCCESS;
}

/** <!-- {{{1 --> @brief run measurement and readout sequence
 * @param[out] _this device structure
 * @return result of run
 * @retval 0: success
 * @retval +ve: warning
 * @retval -ve: error
 */
int8_t sht3x_run(sht3x_t* _this)
{
    int8_t err;
    // single shot measurement
    err = sht3x_meas(_this);
    if (err != SHT3X_SUCCESS) {
        return err;
    }
    // wait after single shot measurement
    _this->wait_ms(_this->msec_wait);
    // readout temperature and humidity
    return sht3x_read(_this);
}

/** <!-- {{{1 --> @brief 8bit CRC Checksum calculation
 * @param[in] data read and/or write data
 * @param[in] len data byte length
 * @return result of CRC-8 calculation
 */
static uint8_t sht3x_crc(const uint8_t* data, uint32_t len)
{
    uint8_t crc = SHT3X_CRC8_INIT;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SHT3X_CRC8_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

} // extern "C"

// end of file {{{1
// vim:ft=cpp:et:nowrap:fdm=marker
