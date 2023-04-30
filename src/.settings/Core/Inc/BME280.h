/*
 * BME280 I2C Driver header file.
 * Author  : Dinkar Jain
 * Created : August 31, 2021
 */

#ifndef BME280_I2C_DRIVER_H_
#define BME280_I2C_DRIVER_H_

/* Header includes */
#include "BME280_defs.h"

/* Functions */

// Initialization
uint8_t bme280_Init(bme280 *dev, I2C_HandleTypeDef *i2cHandle);

// Data Acquisition -> Single I2C expression inside, returns HAL_Status.
HAL_StatusTypeDef bme280_get_sensor_data(bme280 *dev, struct bme280_data *comp_data);
// Data Parsing.
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);
// Data compensation.
void bme280_compensate_data(const struct bme280_uncomp_data *uncomp_data, struct bme280_data *comp_data, struct bme280_calib_data *calib_data);


/* Low Level Functions */

// Reading BME280 registers
HAL_StatusTypeDef bme280_ReadRegister(bme280 *dev, uint8_t reg, uint8_t *data_buffer);
HAL_StatusTypeDef bme280_ReadRegisters(bme280 *dev, uint8_t reg, uint8_t *data_buffer, uint8_t length);

// Writing BME280 registers
HAL_StatusTypeDef bme280_WriteRegister(bme280 *dev, uint8_t reg, uint8_t *data_buffer);


#endif /* BME280_I2C_DRIVER_H_ */
