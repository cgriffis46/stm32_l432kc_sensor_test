################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../bmp280/bmp280.cpp 

OBJS += \
./bmp280/bmp280.o 

CPP_DEPS += \
./bmp280/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
bmp280/%.o bmp280/%.su bmp280/%.cyclo: ../bmp280/%.cpp bmp280/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/stm32_projects/L432KC_Sensor_Test/sht31" -I"D:/stm32_projects/L432KC_Sensor_Test/htu21df" -I"D:/stm32_projects/L432KC_SHT31/bmp280" -I"D:/stm32_projects/L432KC_Sensor_Test/bme280" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmp280

clean-bmp280:
	-$(RM) ./bmp280/bmp280.cyclo ./bmp280/bmp280.d ./bmp280/bmp280.o ./bmp280/bmp280.su

.PHONY: clean-bmp280

