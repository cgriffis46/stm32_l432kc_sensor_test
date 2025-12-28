################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../bme280/BME280.cpp 

OBJS += \
./bme280/BME280.o 

CPP_DEPS += \
./bme280/BME280.d 


# Each subdirectory must supply rules for building sources it contributes
bme280/%.o bme280/%.su bme280/%.cyclo: ../bme280/%.cpp bme280/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/sht31" -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/htu21df" -I"D:/stm32_projects/L432KC_SHT31/bmp280" -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/bme280" -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/Si7021" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bme280

clean-bme280:
	-$(RM) ./bme280/BME280.cyclo ./bme280/BME280.d ./bme280/BME280.o ./bme280/BME280.su

.PHONY: clean-bme280

