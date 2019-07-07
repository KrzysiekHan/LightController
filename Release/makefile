################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include LCD_ATM8/subdir.mk
-include I2C_TWI/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
A_008_MAZUREK_AT8_v1.1.lss \

FLASH_IMAGE += \
A_008_MAZUREK_AT8_v1.1.hex \

EEPROM_IMAGE += \
A_008_MAZUREK_AT8_v1.1.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: A_008_MAZUREK_AT8_v1.1.elf secondary-outputs

# Tool invocations
A_008_MAZUREK_AT8_v1.1.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,A_008_MAZUREK_AT8_v1.1.map -mmcu=atmega8 -o"A_008_MAZUREK_AT8_v1.1.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

A_008_MAZUREK_AT8_v1.1.lss: A_008_MAZUREK_AT8_v1.1.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S A_008_MAZUREK_AT8_v1.1.elf  >"A_008_MAZUREK_AT8_v1.1.lss"
	@echo 'Finished building: $@'
	@echo ' '

A_008_MAZUREK_AT8_v1.1.hex: A_008_MAZUREK_AT8_v1.1.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex A_008_MAZUREK_AT8_v1.1.elf  "A_008_MAZUREK_AT8_v1.1.hex"
	@echo 'Finished building: $@'
	@echo ' '

A_008_MAZUREK_AT8_v1.1.eep: A_008_MAZUREK_AT8_v1.1.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex A_008_MAZUREK_AT8_v1.1.elf  "A_008_MAZUREK_AT8_v1.1.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: A_008_MAZUREK_AT8_v1.1.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 A_008_MAZUREK_AT8_v1.1.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) A_008_MAZUREK_AT8_v1.1.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
