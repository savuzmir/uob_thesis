################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LQRCtrl.cpp 

OBJS += \
./LQRCtrl.o 

CPP_DEPS += \
./LQRCtrl.d 


# Each subdirectory must supply rules for building sources it contributes
LQRCtrl.o: ../LQRCtrl.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"LQRCtrl.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


