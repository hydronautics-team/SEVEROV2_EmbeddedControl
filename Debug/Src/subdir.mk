################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FreeRTOSTick.c \
../Src/MS5837.c \
../Src/checksum.c \
../Src/communication.c \
../Src/dma.c \
../Src/flash.c \
../Src/freertos.c \
../Src/global.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/stabilization.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_hal_timebase_tim.c \
../Src/stm32f3xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f3xx.c \
../Src/thrusters.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/FreeRTOSTick.o \
./Src/MS5837.o \
./Src/checksum.o \
./Src/communication.o \
./Src/dma.o \
./Src/flash.o \
./Src/freertos.o \
./Src/global.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/stabilization.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_hal_timebase_tim.o \
./Src/stm32f3xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f3xx.o \
./Src/thrusters.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/FreeRTOSTick.d \
./Src/MS5837.d \
./Src/checksum.d \
./Src/communication.d \
./Src/dma.d \
./Src/flash.d \
./Src/freertos.d \
./Src/global.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/stabilization.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_hal_timebase_tim.d \
./Src/stm32f3xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f3xx.d \
./Src/thrusters.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/FreeRTOSTick.d ./Src/FreeRTOSTick.o ./Src/FreeRTOSTick.su ./Src/MS5837.d ./Src/MS5837.o ./Src/MS5837.su ./Src/checksum.d ./Src/checksum.o ./Src/checksum.su ./Src/communication.d ./Src/communication.o ./Src/communication.su ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/flash.d ./Src/flash.o ./Src/flash.su ./Src/freertos.d ./Src/freertos.o ./Src/freertos.su ./Src/global.d ./Src/global.o ./Src/global.su ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/stabilization.d ./Src/stabilization.o ./Src/stabilization.su ./Src/stm32f3xx_hal_msp.d ./Src/stm32f3xx_hal_msp.o ./Src/stm32f3xx_hal_msp.su ./Src/stm32f3xx_hal_timebase_tim.d ./Src/stm32f3xx_hal_timebase_tim.o ./Src/stm32f3xx_hal_timebase_tim.su ./Src/stm32f3xx_it.d ./Src/stm32f3xx_it.o ./Src/stm32f3xx_it.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f3xx.d ./Src/system_stm32f3xx.o ./Src/system_stm32f3xx.su ./Src/thrusters.d ./Src/thrusters.o ./Src/thrusters.su ./Src/tim.d ./Src/tim.o ./Src/tim.su ./Src/usart.d ./Src/usart.o ./Src/usart.su

.PHONY: clean-Src

