################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/control_receive.c \
../Core/Src/debug_transmit.c \
../Core/Src/dynamics.c \
../Core/Src/esc.c \
../Core/Src/gain_receive.c \
../Core/Src/main.c \
../Core/Src/printf_define.c \
../Core/Src/sensor.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tick.c \
../Core/Src/timer.c \
../Core/Src/uart_utils.c 

OBJS += \
./Core/Src/control_receive.o \
./Core/Src/debug_transmit.o \
./Core/Src/dynamics.o \
./Core/Src/esc.o \
./Core/Src/gain_receive.o \
./Core/Src/main.o \
./Core/Src/printf_define.o \
./Core/Src/sensor.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tick.o \
./Core/Src/timer.o \
./Core/Src/uart_utils.o 

C_DEPS += \
./Core/Src/control_receive.d \
./Core/Src/debug_transmit.d \
./Core/Src/dynamics.d \
./Core/Src/esc.d \
./Core/Src/gain_receive.d \
./Core/Src/main.d \
./Core/Src/printf_define.d \
./Core/Src/sensor.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tick.d \
./Core/Src/timer.d \
./Core/Src/uart_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/control_receive.d ./Core/Src/control_receive.o ./Core/Src/control_receive.su ./Core/Src/debug_transmit.d ./Core/Src/debug_transmit.o ./Core/Src/debug_transmit.su ./Core/Src/dynamics.d ./Core/Src/dynamics.o ./Core/Src/dynamics.su ./Core/Src/esc.d ./Core/Src/esc.o ./Core/Src/esc.su ./Core/Src/gain_receive.d ./Core/Src/gain_receive.o ./Core/Src/gain_receive.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/printf_define.d ./Core/Src/printf_define.o ./Core/Src/printf_define.su ./Core/Src/sensor.d ./Core/Src/sensor.o ./Core/Src/sensor.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tick.d ./Core/Src/tick.o ./Core/Src/tick.su ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/timer.su ./Core/Src/uart_utils.d ./Core/Src/uart_utils.o ./Core/Src/uart_utils.su

.PHONY: clean-Core-2f-Src

