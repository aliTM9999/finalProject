################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/wm8994/wm8994.c 

OBJS += \
./Drivers/Components/wm8994/wm8994.o 

C_DEPS += \
./Drivers/Components/wm8994/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/wm8994/%.o Drivers/Components/wm8994/%.su Drivers/Components/wm8994/%.cyclo: ../Drivers/Components/wm8994/%.c Drivers/Components/wm8994/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I"C:/Users/Ali/STM32CubeIDE/workspace_1.16.0/finalProject/Drivers/Components" -I"C:/Users/Ali/STM32CubeIDE/workspace_1.16.0/finalProject/Drivers/Components/Common" -I"C:/Users/Ali/STM32CubeIDE/workspace_1.16.0/finalProject/Drivers/Components/lsm6dsl" -I"C:/Users/Ali/STM32CubeIDE/workspace_1.16.0/finalProject/Drivers/Components/lis3mdl" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-wm8994

clean-Drivers-2f-Components-2f-wm8994:
	-$(RM) ./Drivers/Components/wm8994/wm8994.cyclo ./Drivers/Components/wm8994/wm8994.d ./Drivers/Components/wm8994/wm8994.o ./Drivers/Components/wm8994/wm8994.su

.PHONY: clean-Drivers-2f-Components-2f-wm8994

