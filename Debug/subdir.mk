################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main.c 

C_DEPS += \
./main.d 

OBJS += \
./main.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_Blinky" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_Blinky\FreeRTOS\Source\portable\WinAVR\ATmega2560" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_Blinky\FreeRTOS\Source\include" -I"D:\Dropbox\estudos e projetos\FreeRTOS\FreeRTOS_ArduinoMEGA-Demo_Blinky\FreeRTOS\Demo\Common\include" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


