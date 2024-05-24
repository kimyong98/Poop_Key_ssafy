################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UserLib/Src/mlx90614.c 

OBJS += \
./Drivers/UserLib/Src/mlx90614.o 

C_DEPS += \
./Drivers/UserLib/Src/mlx90614.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UserLib/Src/%.o Drivers/UserLib/Src/%.su Drivers/UserLib/Src/%.cyclo: ../Drivers/UserLib/Src/%.c Drivers/UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/SSAFY/STM32CubeIDE/workspace_1.14.0/mlx/Drivers/UserLib/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-UserLib-2f-Src

clean-Drivers-2f-UserLib-2f-Src:
	-$(RM) ./Drivers/UserLib/Src/mlx90614.cyclo ./Drivers/UserLib/Src/mlx90614.d ./Drivers/UserLib/Src/mlx90614.o ./Drivers/UserLib/Src/mlx90614.su

.PHONY: clean-Drivers-2f-UserLib-2f-Src

