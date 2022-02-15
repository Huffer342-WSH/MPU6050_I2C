################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/MPU6050_Motion_Driver/mllite/data_builder.c \
../Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.c \
../Middlewares/MPU6050_Motion_Driver/mllite/message_layer.c \
../Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.c \
../Middlewares/MPU6050_Motion_Driver/mllite/mlmath.c \
../Middlewares/MPU6050_Motion_Driver/mllite/mpl.c \
../Middlewares/MPU6050_Motion_Driver/mllite/results_holder.c \
../Middlewares/MPU6050_Motion_Driver/mllite/start_manager.c \
../Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.c 

OBJS += \
./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.o \
./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.o \
./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.o \
./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.o \
./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.o \
./Middlewares/MPU6050_Motion_Driver/mllite/mpl.o \
./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.o \
./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.o \
./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.o 

C_DEPS += \
./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.d \
./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.d \
./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.d \
./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.d \
./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.d \
./Middlewares/MPU6050_Motion_Driver/mllite/mpl.d \
./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.d \
./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.d \
./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/MPU6050_Motion_Driver/mllite/%.o: ../Middlewares/MPU6050_Motion_Driver/mllite/%.c Middlewares/MPU6050_Motion_Driver/mllite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL -DMPL_LOG_NDEBUG=0 -DMPU6050 -DUSE_DMP -DREMOVE_LOGGING -DEMPL_TARGET_STM32F1 -DHAL -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/MPU6050_Motion_Driver/driver/eMPL -I../Middlewares/MPU6050_Motion_Driver/driver/include -I../Middlewares/MPU6050_Motion_Driver/eMPL-hal -I../Middlewares/MPU6050_Motion_Driver/driver/stm32L -I../Middlewares/MPU6050_Motion_Driver/mllite -I../Middlewares/MPU6050_Motion_Driver/mpl -I../Middlewares/MPU6050_Motion_Driver/porting -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-MPU6050_Motion_Driver-2f-mllite

clean-Middlewares-2f-MPU6050_Motion_Driver-2f-mllite:
	-$(RM) ./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.d ./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.o ./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.d ./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.o ./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.d ./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.o ./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.d ./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.o ./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.d ./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.o ./Middlewares/MPU6050_Motion_Driver/mllite/mpl.d ./Middlewares/MPU6050_Motion_Driver/mllite/mpl.o ./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.d ./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.o ./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.d ./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.o ./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.d ./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.o

.PHONY: clean-Middlewares-2f-MPU6050_Motion_Driver-2f-mllite

