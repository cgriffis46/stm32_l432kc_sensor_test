################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../sht31/SHT31.cpp 

OBJS += \
./sht31/SHT31.o 

CPP_DEPS += \
./sht31/SHT31.d 


# Each subdirectory must supply rules for building sources it contributes
sht31/%.o sht31/%.su sht31/%.cyclo: ../sht31/%.cpp sht31/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/stm32_projects/L432KC_Sensor_Test/sht31" -I"D:/stm32_projects/L432KC_Sensor_Test/htu21df" -I"D:/stm32_projects/L432KC_SHT31/bmp280" -I"D:/stm32_projects/L432KC_Sensor_Test/bme280" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-sht31

clean-sht31:
	-$(RM) ./sht31/SHT31.cyclo ./sht31/SHT31.d ./sht31/SHT31.o ./sht31/SHT31.su

.PHONY: clean-sht31

