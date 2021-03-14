################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ssd1306/ssd1306.c \
../Drivers/ssd1306/ssd1306_fonts.c 

OBJS += \
./Drivers/ssd1306/ssd1306.o \
./Drivers/ssd1306/ssd1306_fonts.o 

C_DEPS += \
./Drivers/ssd1306/ssd1306.d \
./Drivers/ssd1306/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ssd1306/ssd1306.o: ../Drivers/ssd1306/ssd1306.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Core/Inc -I../Drivers/ssd1306 -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ssd1306/ssd1306.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ssd1306/ssd1306_fonts.o: ../Drivers/ssd1306/ssd1306_fonts.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Core/Inc -I../Drivers/ssd1306 -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ssd1306/ssd1306_fonts.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

