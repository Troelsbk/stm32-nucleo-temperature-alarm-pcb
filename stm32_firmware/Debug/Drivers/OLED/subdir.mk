################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/OLED/oled_functions.c \
../Drivers/OLED/ssd1306.c \
../Drivers/OLED/ssd1306_fonts.c \
../Drivers/OLED/ssd1306_tests.c 

OBJS += \
./Drivers/OLED/oled_functions.o \
./Drivers/OLED/ssd1306.o \
./Drivers/OLED/ssd1306_fonts.o \
./Drivers/OLED/ssd1306_tests.o 

C_DEPS += \
./Drivers/OLED/oled_functions.d \
./Drivers/OLED/ssd1306.d \
./Drivers/OLED/ssd1306_fonts.d \
./Drivers/OLED/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/OLED/%.o Drivers/OLED/%.su Drivers/OLED/%.cyclo: ../Drivers/OLED/%.c Drivers/OLED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F042x6 -c -I../Core/Inc -I"C:/Users/troel/STM32CubeIDE/workspace_1.18.1/F04_temp_sensor/Drivers/OLED" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-OLED

clean-Drivers-2f-OLED:
	-$(RM) ./Drivers/OLED/oled_functions.cyclo ./Drivers/OLED/oled_functions.d ./Drivers/OLED/oled_functions.o ./Drivers/OLED/oled_functions.su ./Drivers/OLED/ssd1306.cyclo ./Drivers/OLED/ssd1306.d ./Drivers/OLED/ssd1306.o ./Drivers/OLED/ssd1306.su ./Drivers/OLED/ssd1306_fonts.cyclo ./Drivers/OLED/ssd1306_fonts.d ./Drivers/OLED/ssd1306_fonts.o ./Drivers/OLED/ssd1306_fonts.su ./Drivers/OLED/ssd1306_tests.cyclo ./Drivers/OLED/ssd1306_tests.d ./Drivers/OLED/ssd1306_tests.o ./Drivers/OLED/ssd1306_tests.su

.PHONY: clean-Drivers-2f-OLED

