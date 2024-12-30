################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS_SD/FATFS_SD.c 

OBJS += \
./FATFS_SD/FATFS_SD.o 

C_DEPS += \
./FATFS_SD/FATFS_SD.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS_SD/%.o FATFS_SD/%.su FATFS_SD/%.cyclo: ../FATFS_SD/%.c FATFS_SD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-FATFS_SD

clean-FATFS_SD:
	-$(RM) ./FATFS_SD/FATFS_SD.cyclo ./FATFS_SD/FATFS_SD.d ./FATFS_SD/FATFS_SD.o ./FATFS_SD/FATFS_SD.su

.PHONY: clean-FATFS_SD

