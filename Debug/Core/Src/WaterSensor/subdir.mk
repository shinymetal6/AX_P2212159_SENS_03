################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/WaterSensor/Ws.c \
../Core/Src/WaterSensor/irqs.c \
../Core/Src/WaterSensor/result.c \
../Core/Src/WaterSensor/timers.c \
../Core/Src/WaterSensor/uart.c 

OBJS += \
./Core/Src/WaterSensor/Ws.o \
./Core/Src/WaterSensor/irqs.o \
./Core/Src/WaterSensor/result.o \
./Core/Src/WaterSensor/timers.o \
./Core/Src/WaterSensor/uart.o 

C_DEPS += \
./Core/Src/WaterSensor/Ws.d \
./Core/Src/WaterSensor/irqs.d \
./Core/Src/WaterSensor/result.d \
./Core/Src/WaterSensor/timers.d \
./Core/Src/WaterSensor/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/WaterSensor/%.o Core/Src/WaterSensor/%.su Core/Src/WaterSensor/%.cyclo: ../Core/Src/WaterSensor/%.c Core/Src/WaterSensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/Src/WaterSensor -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-WaterSensor

clean-Core-2f-Src-2f-WaterSensor:
	-$(RM) ./Core/Src/WaterSensor/Ws.cyclo ./Core/Src/WaterSensor/Ws.d ./Core/Src/WaterSensor/Ws.o ./Core/Src/WaterSensor/Ws.su ./Core/Src/WaterSensor/irqs.cyclo ./Core/Src/WaterSensor/irqs.d ./Core/Src/WaterSensor/irqs.o ./Core/Src/WaterSensor/irqs.su ./Core/Src/WaterSensor/result.cyclo ./Core/Src/WaterSensor/result.d ./Core/Src/WaterSensor/result.o ./Core/Src/WaterSensor/result.su ./Core/Src/WaterSensor/timers.cyclo ./Core/Src/WaterSensor/timers.d ./Core/Src/WaterSensor/timers.o ./Core/Src/WaterSensor/timers.su ./Core/Src/WaterSensor/uart.cyclo ./Core/Src/WaterSensor/uart.d ./Core/Src/WaterSensor/uart.o ./Core/Src/WaterSensor/uart.su

.PHONY: clean-Core-2f-Src-2f-WaterSensor

