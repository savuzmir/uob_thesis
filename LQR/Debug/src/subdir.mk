################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/ComputeK.cpp \
../src/CostFunc.cpp \
../src/CreateTrajectory.cpp \
../src/LQR.cpp \
../src/OptimalControl.cpp \
../src/Prediction.cpp \
../src/PredictionCostFunc.cpp \
../src/ReferenceChange.cpp \
../src/SolveDARE.cpp \
../src/UpdateQ.cpp \
../src/UpdateR.cpp \
../src/old_SolveDARE.cpp \
../src/util.cpp 

OBJS += \
./src/ComputeK.o \
./src/CostFunc.o \
./src/CreateTrajectory.o \
./src/LQR.o \
./src/OptimalControl.o \
./src/Prediction.o \
./src/PredictionCostFunc.o \
./src/ReferenceChange.o \
./src/SolveDARE.o \
./src/UpdateQ.o \
./src/UpdateR.o \
./src/old_SolveDARE.o \
./src/util.o 

CPP_DEPS += \
./src/ComputeK.d \
./src/CostFunc.d \
./src/CreateTrajectory.d \
./src/LQR.d \
./src/OptimalControl.d \
./src/Prediction.d \
./src/PredictionCostFunc.d \
./src/ReferenceChange.d \
./src/SolveDARE.d \
./src/UpdateQ.d \
./src/UpdateR.d \
./src/old_SolveDARE.d \
./src/util.d 


# Each subdirectory must supply rules for building sources it contributes
src/ComputeK.o: ../src/ComputeK.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/ComputeK.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/CreateTrajectory.o: ../src/CreateTrajectory.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/CreateTrajectory.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/LQR.o: ../src/LQR.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/LQR.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


