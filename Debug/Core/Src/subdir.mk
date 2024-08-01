################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmi270.c \
../Core/Src/bmi270Config.c \
../Core/Src/dma.c \
../Core/Src/elrsToRcData.c \
../Core/Src/esc.c \
../Core/Src/expresslrs.c \
../Core/Src/flashMemoryConfig.c \
../Core/Src/gpio.c \
../Core/Src/imu.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/motorMixer.c \
../Core/Src/pid.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/sx1280.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c 

OBJS += \
./Core/Src/bmi270.o \
./Core/Src/bmi270Config.o \
./Core/Src/dma.o \
./Core/Src/elrsToRcData.o \
./Core/Src/esc.o \
./Core/Src/expresslrs.o \
./Core/Src/flashMemoryConfig.o \
./Core/Src/gpio.o \
./Core/Src/imu.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/motorMixer.o \
./Core/Src/pid.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/sx1280.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o 

C_DEPS += \
./Core/Src/bmi270.d \
./Core/Src/bmi270Config.d \
./Core/Src/dma.d \
./Core/Src/elrsToRcData.d \
./Core/Src/esc.d \
./Core/Src/expresslrs.d \
./Core/Src/flashMemoryConfig.d \
./Core/Src/gpio.d \
./Core/Src/imu.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/motorMixer.d \
./Core/Src/pid.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/sx1280.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bmi270.cyclo ./Core/Src/bmi270.d ./Core/Src/bmi270.o ./Core/Src/bmi270.su ./Core/Src/bmi270Config.cyclo ./Core/Src/bmi270Config.d ./Core/Src/bmi270Config.o ./Core/Src/bmi270Config.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/elrsToRcData.cyclo ./Core/Src/elrsToRcData.d ./Core/Src/elrsToRcData.o ./Core/Src/elrsToRcData.su ./Core/Src/esc.cyclo ./Core/Src/esc.d ./Core/Src/esc.o ./Core/Src/esc.su ./Core/Src/expresslrs.cyclo ./Core/Src/expresslrs.d ./Core/Src/expresslrs.o ./Core/Src/expresslrs.su ./Core/Src/flashMemoryConfig.cyclo ./Core/Src/flashMemoryConfig.d ./Core/Src/flashMemoryConfig.o ./Core/Src/flashMemoryConfig.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motorMixer.cyclo ./Core/Src/motorMixer.d ./Core/Src/motorMixer.o ./Core/Src/motorMixer.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/sx1280.cyclo ./Core/Src/sx1280.d ./Core/Src/sx1280.o ./Core/Src/sx1280.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su

.PHONY: clean-Core-2f-Src

