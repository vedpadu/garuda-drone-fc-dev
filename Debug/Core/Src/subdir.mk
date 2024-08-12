################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/biquadlpf.c \
../Core/Src/bmi270.c \
../Core/Src/bmi270_config_file.c \
../Core/Src/button_handler.c \
../Core/Src/com_debugging.c \
../Core/Src/dma.c \
../Core/Src/elrs_rcdata_handler.c \
../Core/Src/esc.c \
../Core/Src/expresslrs.c \
../Core/Src/flash_memory_handler.c \
../Core/Src/gpio.c \
../Core/Src/imu.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/math_util.c \
../Core/Src/motor_mixer.c \
../Core/Src/output_handler.c \
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
./Core/Src/biquadlpf.o \
./Core/Src/bmi270.o \
./Core/Src/bmi270_config_file.o \
./Core/Src/button_handler.o \
./Core/Src/com_debugging.o \
./Core/Src/dma.o \
./Core/Src/elrs_rcdata_handler.o \
./Core/Src/esc.o \
./Core/Src/expresslrs.o \
./Core/Src/flash_memory_handler.o \
./Core/Src/gpio.o \
./Core/Src/imu.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/math_util.o \
./Core/Src/motor_mixer.o \
./Core/Src/output_handler.o \
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
./Core/Src/biquadlpf.d \
./Core/Src/bmi270.d \
./Core/Src/bmi270_config_file.d \
./Core/Src/button_handler.d \
./Core/Src/com_debugging.d \
./Core/Src/dma.d \
./Core/Src/elrs_rcdata_handler.d \
./Core/Src/esc.d \
./Core/Src/expresslrs.d \
./Core/Src/flash_memory_handler.d \
./Core/Src/gpio.d \
./Core/Src/imu.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/math_util.d \
./Core/Src/motor_mixer.d \
./Core/Src/output_handler.d \
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
	-$(RM) ./Core/Src/biquadlpf.cyclo ./Core/Src/biquadlpf.d ./Core/Src/biquadlpf.o ./Core/Src/biquadlpf.su ./Core/Src/bmi270.cyclo ./Core/Src/bmi270.d ./Core/Src/bmi270.o ./Core/Src/bmi270.su ./Core/Src/bmi270_config_file.cyclo ./Core/Src/bmi270_config_file.d ./Core/Src/bmi270_config_file.o ./Core/Src/bmi270_config_file.su ./Core/Src/button_handler.cyclo ./Core/Src/button_handler.d ./Core/Src/button_handler.o ./Core/Src/button_handler.su ./Core/Src/com_debugging.cyclo ./Core/Src/com_debugging.d ./Core/Src/com_debugging.o ./Core/Src/com_debugging.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/elrs_rcdata_handler.cyclo ./Core/Src/elrs_rcdata_handler.d ./Core/Src/elrs_rcdata_handler.o ./Core/Src/elrs_rcdata_handler.su ./Core/Src/esc.cyclo ./Core/Src/esc.d ./Core/Src/esc.o ./Core/Src/esc.su ./Core/Src/expresslrs.cyclo ./Core/Src/expresslrs.d ./Core/Src/expresslrs.o ./Core/Src/expresslrs.su ./Core/Src/flash_memory_handler.cyclo ./Core/Src/flash_memory_handler.d ./Core/Src/flash_memory_handler.o ./Core/Src/flash_memory_handler.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/math_util.cyclo ./Core/Src/math_util.d ./Core/Src/math_util.o ./Core/Src/math_util.su ./Core/Src/motor_mixer.cyclo ./Core/Src/motor_mixer.d ./Core/Src/motor_mixer.o ./Core/Src/motor_mixer.su ./Core/Src/output_handler.cyclo ./Core/Src/output_handler.d ./Core/Src/output_handler.o ./Core/Src/output_handler.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/sx1280.cyclo ./Core/Src/sx1280.d ./Core/Src/sx1280.o ./Core/Src/sx1280.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su

.PHONY: clean-Core-2f-Src

