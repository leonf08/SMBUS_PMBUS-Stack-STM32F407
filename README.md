# SMBUS_PMBUS-Stack-STM32F407
There are files of ST HAL library and SMBUS/PMBUS stack which is necessary for the implementation of SMBUS/PMBUS interface on the popular MCU line STM32F407. 
The ST company doesn't provide SMBUS/PMBUS stack for this MCUs. It's only avaliable for STM32F0, STM32F3, STM32L0, STM32L4. Though this MCUs are better for using SMBUS/PMBUS interface because of more suitable architecture of I2C hardware, however there is a demand to use SMBUS/PMBUS interface on the STM32F4 using standard commands written in SMBUS/PMBUS stack code.

To use this SMBUS/PMBUS stack do as follows:
1. Simply add files into your project.
2. Edit stm32f4xx_hal_conf.h file adding #define HAL_SMBUS_MODULE_ENABLED.
3. Then lower, where any header files of peripheral are included, add to include header files of SMBUS driver:

#ifdef HAL_SMBUS_MODULE_ENABLED
#include "stm32f4xx_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

4.Try.
