################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/Source/portable/WinAVR/ATmega2560/port.c 

C_DEPS += \
./FreeRTOS/Source/portable/WinAVR/ATmega2560/port.d 

OBJS += \
./FreeRTOS/Source/portable/WinAVR/ATmega2560/port.o 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/Source/portable/WinAVR/ATmega2560/%.o: ../FreeRTOS/Source/portable/WinAVR/ATmega2560/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323\FreeRTOS\Source\portable\WinAVR\ATmega2560" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323\FreeRTOS\Demo\Common\include" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_AVR323\FreeRTOS\Source\include" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


