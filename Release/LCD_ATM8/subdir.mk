################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD_ATM8/lcd44780.c 

OBJS += \
./LCD_ATM8/lcd44780.o 

C_DEPS += \
./LCD_ATM8/lcd44780.d 


# Each subdirectory must supply rules for building sources it contributes
LCD_ATM8/%.o: ../LCD_ATM8/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega8 -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


