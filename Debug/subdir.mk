################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ble113.c \
../cmd_def.c \
../configs.c \
../main.c \
../parser.c \
../startup_gcc.c \
../stubs.c \
../uart.c 

OBJS += \
./ble113.o \
./cmd_def.o \
./configs.o \
./main.o \
./parser.o \
./startup_gcc.o \
./stubs.o \
./uart.o 

C_DEPS += \
./ble113.d \
./cmd_def.d \
./configs.d \
./main.d \
./parser.d \
./startup_gcc.d \
./stubs.d \
./uart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DPART_TM4C123GH6PM -DTARGET_IS_BLIZZARD_RB1 -DARM_MATH_CM4 -I"C:\ti\TivaWare_C_Series-2.1.2.111" -O0 -g3 -Wall -c -fmessage-length=0 -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -ffunction-sections -fdata-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


