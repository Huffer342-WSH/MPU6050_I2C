################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.c 

OBJS += \
./Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.o 

C_DEPS += \
./Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.o: ../Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.c Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL -DMPL_LOG_NDEBUG -DMPU6050 -DUSE_DMP -DREMOVE_LOGGING -DEMPL_TARGET_STM32F1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/mpu6050 -I"../Middlewares/MPU6050 Motion Driver/driver/eMPL" -I"../Middlewares/MPU6050 Motion Driver/driver/include" -I"../Middlewares/MPU6050 Motion Driver/driver/stm32L" -I"../Middlewares/MPU6050 Motion Driver/eMPL-hal" -I"../Middlewares/MPU6050 Motion Driver/mllite" -I"../Middlewares/MPU6050 Motion Driver/mpl" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/MPU6050 Motion Driver/eMPL-hal/eMPL_outputs.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-MPU6050-20-Motion-20-Driver-2f-eMPL-2d-hal

clean-Middlewares-2f-MPU6050-20-Motion-20-Driver-2f-eMPL-2d-hal:
	-$(RM) ./Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.d ./Middlewares/MPU6050\ Motion\ Driver/eMPL-hal/eMPL_outputs.o

.PHONY: clean-Middlewares-2f-MPU6050-20-Motion-20-Driver-2f-eMPL-2d-hal

