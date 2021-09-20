# STM32_HAL_BME280_DRV

BME280 I2C Driver for STM32 Devices based on STM32-HAL. A rewrite of original Bosch driver written in C programming Language.


## Desrciption:

Easy to understand crude driver for BME280(Pressure, Temperature and Humidity Sensor) over the STM32 HAL, based on original Bosch Library. Code is simplified a lot for easy understanding, a lot of complex error checking is also removed. It's basically a rewrite of original library. This library utilize full integration with STM32 HAL(Hardware Abstraction Layer) and optimized bosch library.

Main purpose for development of this library is to understand the HAL structure for STM32 and to provide basic low level functionality to the end user. I will not say it's easy to understand but it's a joy to rewrite a driver on own after understanding all the intricacies. 

In this implementation a lot of error checking is stripped down which would be rather necessary in production environment. This functionality can be rewritten using the Original Bosch driver. I would highly suggest for getting the driver from the original source. This is just a less complicated version for the purpose of understanding.

This driver can be further extended to include EUCAL layer as well, which will abstract the difficult and messy parts and expose only configuration parameters to the end user.

Functions are choosen as to include only 32 bit operations as STM32 devices only support 32bit atomic operations.


## References:

[Bosch BME280 Driver API](https://github.com/BoschSensortec/BME280_driver)