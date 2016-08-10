################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include serial/subdir.mk
-include RegTest/subdir.mk
-include ParTest/subdir.mk
-include FreeRTOS/Source/portable/WinAVR/ATmega2560/subdir.mk
-include FreeRTOS/Source/portable/MemMang/subdir.mk
-include FreeRTOS/Source/subdir.mk
-include FreeRTOS/Demo/Common/Minimal/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
FreeRTOS_ArduinoMEGA-Demo_AVR323.lss \

FLASH_IMAGE += \
FreeRTOS_ArduinoMEGA-Demo_AVR323.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: FreeRTOS_ArduinoMEGA-Demo_AVR323.elf secondary-outputs

# Tool invocations
FreeRTOS_ArduinoMEGA-Demo_AVR323.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,FreeRTOS_ArduinoMEGA-Demo_AVR323.map,--cref -mrelax -Wl,--gc-sections -mmcu=atmega2560 -o "FreeRTOS_ArduinoMEGA-Demo_AVR323.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

FreeRTOS_ArduinoMEGA-Demo_AVR323.lss: FreeRTOS_ArduinoMEGA-Demo_AVR323.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S FreeRTOS_ArduinoMEGA-Demo_AVR323.elf  >"FreeRTOS_ArduinoMEGA-Demo_AVR323.lss"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_ArduinoMEGA-Demo_AVR323.hex: FreeRTOS_ArduinoMEGA-Demo_AVR323.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex FreeRTOS_ArduinoMEGA-Demo_AVR323.elf  "FreeRTOS_ArduinoMEGA-Demo_AVR323.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: FreeRTOS_ArduinoMEGA-Demo_AVR323.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega2560 FreeRTOS_ArduinoMEGA-Demo_AVR323.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(S_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) FreeRTOS_ArduinoMEGA-Demo_AVR323.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
