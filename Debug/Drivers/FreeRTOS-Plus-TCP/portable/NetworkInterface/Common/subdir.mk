################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.c 

OBJS += \
./Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.o 

C_DEPS += \
./Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/%.o Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/%.su Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/%.cyclo: ../Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/%.c Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H753xx -DDATA_IN_D2_SRAM -DUSE_LOG_EVENT=0 -DLOG_EVENT_COUNT=100 -DLOG_EVENT_NAME_LEN=32 -DEVENT_MAY_WRAP=0 -DSTATIC_LOG_MEMORY=1 -DipCONFIG_USE_TCP_MEM_STATS=0 -DSYMBOLS_USED=1 -c -I../Core/Inc -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/include" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/STM32Hxx" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Kernel/include" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/Utilities" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/include" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/portable/Compiler/GCC" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/tools/tcp_utilities/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FreeRTOS-2d-Plus-2d-TCP-2f-portable-2f-NetworkInterface-2f-Common

clean-Drivers-2f-FreeRTOS-2d-Plus-2d-TCP-2f-portable-2f-NetworkInterface-2f-Common:
	-$(RM) ./Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.cyclo ./Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.d ./Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.o ./Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/phyHandling.su

.PHONY: clean-Drivers-2f-FreeRTOS-2d-Plus-2d-TCP-2f-portable-2f-NetworkInterface-2f-Common

