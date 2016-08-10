################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/Source/portable/MemMang/heap_1.c 

C_DEPS += \
./FreeRTOS/Source/portable/MemMang/heap_1.d 

OBJS += \
./FreeRTOS/Source/portable/MemMang/heap_1.o 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/Source/portable/MemMang/%.o: ../FreeRTOS/Source/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323\FreeRTOS\Source\portable\WinAVR\ATmega2560" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323\FreeRTOS\Demo\Common\include" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323\FreeRTOS\Source\include" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


