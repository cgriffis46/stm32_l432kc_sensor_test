################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../bmp390/bmp390.cpp 

OBJS += \
./bmp390/bmp390.o 

CPP_DEPS += \
./bmp390/bmp390.d 


# Each subdirectory must supply rules for building sources it contributes
bmp390/%.o bmp390/%.su bmp390/%.cyclo: ../bmp390/%.cpp bmp390/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/stm32_projects/L432KC_SHT31/sht31" -I"D:/stm32_projects/L432KC_SHT31/htu21df" -I"D:/stm32_projects/L432KC_SHT31/bmp280" -I"D:/stm32_projects/L432KC_SHT31/bme280" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmp390

clean-bmp390:
	-$(RM) ./bmp390/bmp390.cyclo ./bmp390/bmp390.d ./bmp390/bmp390.o ./bmp390/bmp390.su

.PHONY: clean-bmp390

