################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../mpl3115a2/mpl3115a2.cpp 

OBJS += \
./mpl3115a2/mpl3115a2.o 

CPP_DEPS += \
./mpl3115a2/mpl3115a2.d 


# Each subdirectory must supply rules for building sources it contributes
mpl3115a2/%.o mpl3115a2/%.su mpl3115a2/%.cyclo: ../mpl3115a2/%.cpp mpl3115a2/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/sht31" -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/htu21df" -I"D:/stm32_projects/L432KC_SHT31/bmp280" -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/bme280" -I"D:/stm32_projects/stm32_l432kc_sensor_test/L432KC_Sensor_Test/Si7021" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-mpl3115a2

clean-mpl3115a2:
	-$(RM) ./mpl3115a2/mpl3115a2.cyclo ./mpl3115a2/mpl3115a2.d ./mpl3115a2/mpl3115a2.o ./mpl3115a2/mpl3115a2.su

.PHONY: clean-mpl3115a2

