################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/keypad.c \
../lib/lcd_i2c.c \
../lib/vantay.c 

OBJS += \
./lib/keypad.o \
./lib/lcd_i2c.o \
./lib/vantay.o 

C_DEPS += \
./lib/keypad.d \
./lib/lcd_i2c.d \
./lib/vantay.d 


# Each subdirectory must supply rules for building sources it contributes
lib/%.o lib/%.su lib/%.cyclo: ../lib/%.c lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/NHHanh/Desktop/RTOS-SmartDoor/lib" -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lib

clean-lib:
	-$(RM) ./lib/keypad.cyclo ./lib/keypad.d ./lib/keypad.o ./lib/keypad.su ./lib/lcd_i2c.cyclo ./lib/lcd_i2c.d ./lib/lcd_i2c.o ./lib/lcd_i2c.su ./lib/vantay.cyclo ./lib/vantay.d ./lib/vantay.o ./lib/vantay.su

.PHONY: clean-lib

