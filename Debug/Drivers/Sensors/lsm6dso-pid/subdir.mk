################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.c 

OBJS += \
./Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.o 

C_DEPS += \
./Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Sensors/lsm6dso-pid/%.o Drivers/Sensors/lsm6dso-pid/%.su Drivers/Sensors/lsm6dso-pid/%.cyclo: ../Drivers/Sensors/lsm6dso-pid/%.c Drivers/Sensors/lsm6dso-pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xE -c -I../Core/Inc -I../Drivers/Sensors/lsm6dso-pid -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Sensors-2f-lsm6dso-2d-pid

clean-Drivers-2f-Sensors-2f-lsm6dso-2d-pid:
	-$(RM) ./Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.cyclo ./Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.d ./Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.o ./Drivers/Sensors/lsm6dso-pid/lsm6dso_reg.su

.PHONY: clean-Drivers-2f-Sensors-2f-lsm6dso-2d-pid

