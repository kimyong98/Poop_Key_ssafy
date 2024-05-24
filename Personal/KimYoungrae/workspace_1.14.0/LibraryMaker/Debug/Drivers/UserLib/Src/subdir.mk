################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UserLib/Src/IO_Init.c \
../Drivers/UserLib/Src/mlx.c \
../Drivers/UserLib/Src/servo.c \
../Drivers/UserLib/Src/sonar.c \
../Drivers/UserLib/Src/tilt.c 

OBJS += \
./Drivers/UserLib/Src/IO_Init.o \
./Drivers/UserLib/Src/mlx.o \
./Drivers/UserLib/Src/servo.o \
./Drivers/UserLib/Src/sonar.o \
./Drivers/UserLib/Src/tilt.o 

C_DEPS += \
./Drivers/UserLib/Src/IO_Init.d \
./Drivers/UserLib/Src/mlx.d \
./Drivers/UserLib/Src/servo.d \
./Drivers/UserLib/Src/sonar.d \
./Drivers/UserLib/Src/tilt.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UserLib/Src/%.o Drivers/UserLib/Src/%.su Drivers/UserLib/Src/%.cyclo: ../Drivers/UserLib/Src/%.c Drivers/UserLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/SSAFY/STM32CubeIDE/workspace_1.14.0/LibraryMaker/Drivers/UserLib/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-UserLib-2f-Src

clean-Drivers-2f-UserLib-2f-Src:
	-$(RM) ./Drivers/UserLib/Src/IO_Init.cyclo ./Drivers/UserLib/Src/IO_Init.d ./Drivers/UserLib/Src/IO_Init.o ./Drivers/UserLib/Src/IO_Init.su ./Drivers/UserLib/Src/mlx.cyclo ./Drivers/UserLib/Src/mlx.d ./Drivers/UserLib/Src/mlx.o ./Drivers/UserLib/Src/mlx.su ./Drivers/UserLib/Src/servo.cyclo ./Drivers/UserLib/Src/servo.d ./Drivers/UserLib/Src/servo.o ./Drivers/UserLib/Src/servo.su ./Drivers/UserLib/Src/sonar.cyclo ./Drivers/UserLib/Src/sonar.d ./Drivers/UserLib/Src/sonar.o ./Drivers/UserLib/Src/sonar.su ./Drivers/UserLib/Src/tilt.cyclo ./Drivers/UserLib/Src/tilt.d ./Drivers/UserLib/Src/tilt.o ./Drivers/UserLib/Src/tilt.su

.PHONY: clean-Drivers-2f-UserLib-2f-Src

