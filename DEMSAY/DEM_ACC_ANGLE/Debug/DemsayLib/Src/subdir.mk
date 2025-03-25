################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DemsayLib/Src/DigitalInputOutputs.c 

OBJS += \
./DemsayLib/Src/DigitalInputOutputs.o 

C_DEPS += \
./DemsayLib/Src/DigitalInputOutputs.d 


# Each subdirectory must supply rules for building sources it contributes
DemsayLib/Src/%.o DemsayLib/Src/%.su DemsayLib/Src/%.cyclo: ../DemsayLib/Src/%.c DemsayLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I"../Core/Inc" -I"../Drivers/STM32G0xx_HAL_Driver/Inc" -I"../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy" -I"../Drivers/CMSIS/Device/ST/STM32G0xx/Include" -I"../Drivers/CMSIS/Include" -I"../DemsayLib/Inc" -I../lis2dw12 -I"C:/Users/harun/STM32CubeIDE/DEMSAY/DEM_ACC_ANGLE/ssd1306" -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-DemsayLib-2f-Src

clean-DemsayLib-2f-Src:
	-$(RM) ./DemsayLib/Src/DigitalInputOutputs.cyclo ./DemsayLib/Src/DigitalInputOutputs.d ./DemsayLib/Src/DigitalInputOutputs.o ./DemsayLib/Src/DigitalInputOutputs.su

.PHONY: clean-DemsayLib-2f-Src

