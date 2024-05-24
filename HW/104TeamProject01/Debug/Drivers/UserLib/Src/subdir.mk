################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UserLib/Src/esp_control.c \
../Drivers/UserLib/Src/module_control.c \
../Drivers/UserLib/Src/toilet.c 

OBJS += \
./Drivers/UserLib/Src/esp_control.o \
./Drivers/UserLib/Src/module_control.o \
./Drivers/UserLib/Src/toilet.o 

C_DEPS += \
./Drivers/UserLib/Src/esp_control.d \
./Drivers/UserLib/Src/module_control.d \
./Drivers/UserLib/Src/toilet.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UserLib/Src/%.o Drivers/UserLib/Src/%.su Drivers/UserLib/Src/%.cyclo: ../Drivers/UserLib/Src/%.c Drivers/UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/SSAFY/STM32CubeIDE/workspace_1.14.0/104TeamProject01/Drivers/UserLib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-UserLib-2f-Src

clean-Drivers-2f-UserLib-2f-Src:
	-$(RM) ./Drivers/UserLib/Src/esp_control.cyclo ./Drivers/UserLib/Src/esp_control.d ./Drivers/UserLib/Src/esp_control.o ./Drivers/UserLib/Src/esp_control.su ./Drivers/UserLib/Src/module_control.cyclo ./Drivers/UserLib/Src/module_control.d ./Drivers/UserLib/Src/module_control.o ./Drivers/UserLib/Src/module_control.su ./Drivers/UserLib/Src/toilet.cyclo ./Drivers/UserLib/Src/toilet.d ./Drivers/UserLib/Src/toilet.o ./Drivers/UserLib/Src/toilet.su

.PHONY: clean-Drivers-2f-UserLib-2f-Src

