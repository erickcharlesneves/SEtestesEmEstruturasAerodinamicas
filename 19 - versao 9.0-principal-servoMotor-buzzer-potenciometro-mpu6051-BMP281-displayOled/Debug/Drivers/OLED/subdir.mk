################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/OLED/ssd1306.c \
../Drivers/OLED/ssd1306_fonts.c \
../Drivers/OLED/ssd1306_tests.c 

OBJS += \
./Drivers/OLED/ssd1306.o \
./Drivers/OLED/ssd1306_fonts.o \
./Drivers/OLED/ssd1306_tests.o 

C_DEPS += \
./Drivers/OLED/ssd1306.d \
./Drivers/OLED/ssd1306_fonts.d \
./Drivers/OLED/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/OLED/%.o Drivers/OLED/%.su Drivers/OLED/%.cyclo: ../Drivers/OLED/%.c Drivers/OLED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/erick/STM32CubeIDE/workspace_1.16.0/19 - versao 9.0-principal-servoMotor-buzzer-potenciometro-mpu6051-BMP281-displayOled/Drivers/OLED" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-OLED

clean-Drivers-2f-OLED:
	-$(RM) ./Drivers/OLED/ssd1306.cyclo ./Drivers/OLED/ssd1306.d ./Drivers/OLED/ssd1306.o ./Drivers/OLED/ssd1306.su ./Drivers/OLED/ssd1306_fonts.cyclo ./Drivers/OLED/ssd1306_fonts.d ./Drivers/OLED/ssd1306_fonts.o ./Drivers/OLED/ssd1306_fonts.su ./Drivers/OLED/ssd1306_tests.cyclo ./Drivers/OLED/ssd1306_tests.d ./Drivers/OLED/ssd1306_tests.o ./Drivers/OLED/ssd1306_tests.su

.PHONY: clean-Drivers-2f-OLED

