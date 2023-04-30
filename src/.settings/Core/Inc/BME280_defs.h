/*
 *  BME280 I2C Driver data-structure definitions header file. BME280_defs.h
 *  Author  : Dinkar Jain
 *  Created : September 04, 2021
 */

#ifndef INC_BME280_DEFS_H_
#define INC_BME280_DEFS_H_

#include "stm32f1xx_hal.h"  // Needed for basic definitions.


#define BME280_I2C_ADDR   (0x76 << 1)  // define the default I2C address (7bit)

#define BME280_CHIP_ID  0x60

// Calibration data registers.
#define    BME280_CAL_DIG_T1_REG   0x88
#define    BME280_CAL_DIG_T2_REG   0x8A
#define    BME280_CAL_DIG_T3_REG   0x8C

#define    BME280_CAL_DIG_P1_REG   0x8E
#define    BME280_CAL_DIG_P2_REG   0x90
#define    BME280_CAL_DIG_P3_REG   0x92
#define    BME280_CAL_DIG_P4_REG   0x94
#define    BME280_CAL_DIG_P5_REG   0x96
#define    BME280_CAL_DIG_P6_REG   0x98
#define    BME280_CAL_DIG_P7_REG   0x9A
#define    BME280_CAL_DIG_P8_REG   0x9C
#define    BME280_CAL_DIG_P9_REG   0x9E

#define    BME280_CAL_DIG_H1_REG   0xA1
#define    BME280_CAL_DIG_H2_REG   0xE1
#define    BME280_CAL_DIG_H3_REG   0xE3
#define    BME280_CAL_DIG_H4_REG   0xE4
#define    BME280_CAL_DIG_H5_REG   0xE5
#define    BME280_CAL_DIG_H6_REG   0xE7

// Bulk query for calibration data. (Data stored in 2 sets of successive registers)
#define    BME280_G1_CALIB_DATA_START       0x88
#define    BME280_G1_CALIB_DATA_LEN         26
#define    BME280_G2_CALIB_DATA_START       0xE1
#define    BME280_G2_CALIB_DATA_LEN         7


// Configuration registers.
#define    BME280_REGISTER_CHIPID           0xD0
#define    BME280_REGISTER_VERSION          0xD1
#define    BME280_REGISTER_SOFTRESET        0xE0
#define    BME280_REGISTER_CAL26            0xE1
#define    BME280_REGISTER_CONTROL_HUMID    0xF2
#define    BME280_REGISTER_CONTROL_PTMOD    0xF4
#define    BME280_REGISTER_CONFIG           0xF5

// Sensor data registers.
#define    BME280_REGISTER_PRES_DATA        0xF7
#define    BME280_REGISTER_TEMP_DATA        0xFA
#define    BME280_REGISTER_HUMI_DATA        0xFD

// Bulk query for sensor data. (Data stored in 1 set of successive registers)
#define    BME280_SENSOR_DATA_START         0xF7
#define    BME280_SENSOR_DATA_LEN           8

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/*!
 * @brief Calibration data
 */
struct bme280_calib_data
{
    /*< Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t2;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t3;

    /*< Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p2;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p3;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p4;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p5;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p6;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p7;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p8;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p9;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h2;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h4;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h5;

    /*< Calibration coefficient for the humidity sensor */
    int8_t dig_h6;

    /*< Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
};


/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
struct bme280_data
{
    /*< Compensated pressure */
    uint32_t pressure;

    /*< Compensated temperature */
    int32_t temperature;

    /*< Compensated humidity */
    uint32_t humidity;
};


/*!
 * @brief bme280 sensor structure which comprises of un-compensated temperature,
 * pressure and humidity data
 */
struct bme280_uncomp_data
{
    /*< un-compensated pressure */
    uint32_t pressure;

    /*< un-compensated temperature */
    uint32_t temperature;

    /*< un-compensated humidity */
    uint32_t humidity;
};


typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Calibration data */
	struct bme280_calib_data calib_data;

	/* Sensor settings data */
	// BME280_Settings settings;
	// Settings are hard coded for now.
	// Ref: bme280_init()

} bme280;


#endif /* INC_BME280_DEFS_H_ */


