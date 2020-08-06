################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/main.cpp \
../src/pinhole_camera.cpp \
../src/visual_odometry.cpp 

OBJS += \
./src/main.o \
./src/pinhole_camera.o \
./src/visual_odometry.o 

CPP_DEPS += \
./src/main.d \
./src/pinhole_camera.d \
./src/visual_odometry.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-9.0/bin/nvcc -I/usr/local/zed/include -I/usr/include -I"/home/namikilab/cuda-workspace/stereo_100HZ_TX2/include" -I/home/nvidia/cuda-workspace/include -G -g -O0 -std=c++11 -gencode arch=compute_61,code=sm_61  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-9.0/bin/nvcc -I/usr/local/zed/include -I/usr/include -I"/home/namikilab/cuda-workspace/stereo_100HZ_TX2/include" -I/home/nvidia/cuda-workspace/include -G -g -O0 -std=c++11 --compile  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


