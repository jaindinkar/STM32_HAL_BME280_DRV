/*
 * BME280 I2C Driver C file.
 * Author  : Dinkar Jain
 * Created : August 31, 2021
 *
 */

#include "BME280.h"


/* Start Internal function declarations*/


/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 */
static HAL_StatusTypeDef get_calib_data(bme280 *dev);

/*!
 *  @brief This internal API is used to parse the temperature,
 *  pressure and humidity calibration data and store it in the device structure.
 *
 *  @param[out] dev     : Structure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains the calibration data to be parsed.
 *
 */
static void parse_g1_calib_data(bme280 *dev, const uint8_t *reg_data);

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 *
 *  @param[out] dev     : Structure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains calibration data to be parsed.
 *
 */
static void parse_g2_calib_data(bme280 *dev, const uint8_t *reg_data);

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data  : Pointer to calibration data structure.
 *
 * @return Compensated temperature data in integer.
 *
 */
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
                                      struct bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data  : Pointer to the calibration data structure.
 *
 * @return Compensated pressure data in integer.
 *
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated humidity data.
 * @param[in] calib_data  : Pointer to the calibration data structure.
 *
 * @return Compensated humidity data in integer.
 *
 */
static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data);


/* End internal function declarations */



/* Start Internal function definitions*/

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
static HAL_StatusTypeDef get_calib_data(bme280 *dev)
{
	HAL_StatusTypeDef rslt;
    uint8_t reg_addr = BME280_G1_CALIB_DATA_START;

    /* Array to store calibration data */
    uint8_t calib_data[BME280_G1_CALIB_DATA_LEN] = { 0 };

    /* Read the calibration data from the sensor */
    // I2C call.
    rslt = bme280_ReadRegisters(dev, reg_addr, calib_data, BME280_G1_CALIB_DATA_LEN);

    if (rslt == HAL_OK)
    {
        /* Parse temperature and pressure calibration data and store
         * it in device structure
         */
        parse_g1_calib_data(dev, calib_data);
        reg_addr = BME280_G2_CALIB_DATA_START;

        /* Read the humidity calibration data from the sensor */
        // I2C call.
        rslt = bme280_ReadRegisters(dev, reg_addr, calib_data, BME280_G2_CALIB_DATA_LEN);

        if (rslt == HAL_OK)
        {
            /* Parse humidity calibration data and store it in
             * device structure
             */
            parse_g2_calib_data(dev, calib_data);
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to parse the temperature,
 *  pressure and humidity calibration data and store it in device structure.
 */
static void parse_g1_calib_data(bme280 *dev, const uint8_t *reg_data)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;

    calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_h1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
static void parse_g2_calib_data(bme280 *dev, const uint8_t *reg_data)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data->dig_h6 = (int8_t)reg_data[6];
}


/* 32 bit compensation for pressure data */

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 * Pressure output is in Pascal(Pa).
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_p6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_p4) * 65536);
    var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_p1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)calib_data->dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 * Temperature output is in (degC * 100)
 */
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
                                      struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_t1 * 2));
    var1 = (var1 * ((int32_t)calib_data->dig_t2)) / 2048;
    var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_t3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 * Humidity output is in (%RH * 1024)
 */
static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
    var4 = ((int32_t)calib_data->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}

/* End internal function definitions */


/* Exposed APIs */

// Initialization function
uint8_t bme280_Init(bme280 *dev, I2C_HandleTypeDef *i2cHandle){

	//Setting structure parameters
	dev->i2cHandle = i2cHandle;

	/* Store number of transaction errors. */
	uint8_t errCount = 0;
	HAL_StatusTypeDef status;

	// A reusable 8 bit tx/rx buffer.
	uint8_t regData;
	// Check for the chip ID
	status = bme280_ReadRegister(dev, BME280_REGISTER_CHIPID, &regData);
	errCount += (status != HAL_OK);
	if(regData != BME280_CHIP_ID) return 255;

	/* Reading the calibration data and storing it in structure */
	get_calib_data(dev);

	/* Setting the configuration data. Please MAINTAIN the order.*/

	// 1). Setting configuration data t_stdby for normal mode, tc for IIR filter, SPI mode(if enabled)
	// t_stdby = 0.5ms(000), tc for IIR filter = 4(010), reserved_bit(0), spi_mode = 0  0b00001000
	// regData = 0b00001000;
	// status = bme280_WriteRegister(dev, BME280_REGISTER_CONFIG, &regData);
	// errCount += (status != HAL_OK);

	// 2). Setting Humidity measurement parameters -> oversampling = x8(100)  0b100
	regData = 0b100;
	status = bme280_WriteRegister(dev, BME280_REGISTER_CONTROL_HUMID, &regData);
	errCount += (status != HAL_OK);

	// 3). Setting Pressure, Temperature measurement parameters + measurement modes.
	// p_oversampling = x8 (100), t_oversampling = x8(100), mode = normal(11) 0b10010011
	regData = 0b10010011;
	status = bme280_WriteRegister(dev, BME280_REGISTER_CONTROL_PTMOD, &regData);
	errCount += (status != HAL_OK);

	/* errCount = 0 for successful initialization. */
	return errCount;

}

// Data acquisition functions
/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
HAL_StatusTypeDef bme280_get_sensor_data(bme280 *dev, struct bme280_data *comp_data) {
	uint8_t reg_data[BME280_SENSOR_DATA_LEN] = { 0 };
	struct bme280_uncomp_data uncomp_data = { 0 };

	HAL_StatusTypeDef rslt = bme280_ReadRegisters(dev, BME280_SENSOR_DATA_START, reg_data, BME280_SENSOR_DATA_LEN);

	if (rslt == HAL_OK)
	{
		/* Parse the read data from the sensor */
		bme280_parse_sensor_data(reg_data, &uncomp_data);

		/* Compensate the pressure and/or temperature and/or
		 * humidity data from the sensor
		 */
		bme280_compensate_data(&uncomp_data, comp_data, &dev->calib_data);
	}

	return rslt;

}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (uint32_t)reg_data[0] << 12;
    data_lsb = (uint32_t)reg_data[1] << 4;
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)reg_data[3] << 12;
    data_lsb = (uint32_t)reg_data[4] << 4;
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (uint32_t)reg_data[6] << 8;
    data_lsb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}


/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
void bme280_compensate_data(const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data)
{
	/* Initialize to zero */
	comp_data->temperature = 0;
	comp_data->pressure = 0;
	comp_data->humidity = 0;

	/* Compensate the temperature data */
	comp_data->temperature = compensate_temperature(uncomp_data, calib_data);

	/* Compensate the pressure data */
	comp_data->pressure = compensate_pressure(uncomp_data, calib_data);

	/* Compensate the humidity data */
	comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
}



/* Low Level Functions */

// Read and write functions for I2C Bus.
HAL_StatusTypeDef bme280_ReadRegister(bme280 *dev, uint8_t reg, uint8_t *data_buffer){
	return HAL_I2C_Mem_Read(dev->i2cHandle, BME280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data_buffer, 1, 100);
}

HAL_StatusTypeDef bme280_ReadRegisters(bme280 *dev, uint8_t reg, uint8_t *data_buffer, uint8_t length){
	return HAL_I2C_Mem_Read(dev->i2cHandle, BME280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data_buffer, length, 100);
}

HAL_StatusTypeDef bme280_WriteRegister(bme280 *dev, uint8_t reg, uint8_t *data_buffer){
	return HAL_I2C_Mem_Write(dev->i2cHandle, BME280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data_buffer, 1, 100);
}
