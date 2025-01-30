################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FreeRTOS-Kernel/croutine.c \
../Drivers/FreeRTOS-Kernel/event_groups.c \
../Drivers/FreeRTOS-Kernel/list.c \
../Drivers/FreeRTOS-Kernel/queue.c \
../Drivers/FreeRTOS-Kernel/stream_buffer.c \
../Drivers/FreeRTOS-Kernel/tasks.c \
../Drivers/FreeRTOS-Kernel/timers.c 

OBJS += \
./Drivers/FreeRTOS-Kernel/croutine.o \
./Drivers/FreeRTOS-Kernel/event_groups.o \
./Drivers/FreeRTOS-Kernel/list.o \
./Drivers/FreeRTOS-Kernel/queue.o \
./Drivers/FreeRTOS-Kernel/stream_buffer.o \
./Drivers/FreeRTOS-Kernel/tasks.o \
./Drivers/FreeRTOS-Kernel/timers.o 

C_DEPS += \
./Drivers/FreeRTOS-Kernel/croutine.d \
./Drivers/FreeRTOS-Kernel/event_groups.d \
./Drivers/FreeRTOS-Kernel/list.d \
./Drivers/FreeRTOS-Kernel/queue.d \
./Drivers/FreeRTOS-Kernel/stream_buffer.d \
./Drivers/FreeRTOS-Kernel/tasks.d \
./Drivers/FreeRTOS-Kernel/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FreeRTOS-Kernel/%.o Drivers/FreeRTOS-Kernel/%.su Drivers/FreeRTOS-Kernel/%.cyclo: ../Drivers/FreeRTOS-Kernel/%.c Drivers/FreeRTOS-Kernel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H753xx -DDATA_IN_D2_SRAM -DUSE_LOG_EVENT=0 -DLOG_EVENT_COUNT=100 -DLOG_EVENT_NAME_LEN=32 -DEVENT_MAY_WRAP=0 -DSTATIC_LOG_MEMORY=1 -DipCONFIG_USE_TCP_MEM_STATS=0 -DSYMBOLS_USED=1 -c -I../Core/Inc -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/include" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/portable/NetworkInterface/STM32Hxx" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Kernel/include" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/Utilities" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/include" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/portable/Compiler/GCC" -I"C:/Users/yehen/STM32CubeIDE/workspace_1.13.2/eth-daq-h753/Drivers/FreeRTOS-Plus-TCP/tools/tcp_utilities/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FreeRTOS-2d-Kernel

clean-Drivers-2f-FreeRTOS-2d-Kernel:
	-$(RM) ./Drivers/FreeRTOS-Kernel/croutine.cyclo ./Drivers/FreeRTOS-Kernel/croutine.d ./Drivers/FreeRTOS-Kernel/croutine.o ./Drivers/FreeRTOS-Kernel/croutine.su ./Drivers/FreeRTOS-Kernel/event_groups.cyclo ./Drivers/FreeRTOS-Kernel/event_groups.d ./Drivers/FreeRTOS-Kernel/event_groups.o ./Drivers/FreeRTOS-Kernel/event_groups.su ./Drivers/FreeRTOS-Kernel/list.cyclo ./Drivers/FreeRTOS-Kernel/list.d ./Drivers/FreeRTOS-Kernel/list.o ./Drivers/FreeRTOS-Kernel/list.su ./Drivers/FreeRTOS-Kernel/queue.cyclo ./Drivers/FreeRTOS-Kernel/queue.d ./Drivers/FreeRTOS-Kernel/queue.o ./Drivers/FreeRTOS-Kernel/queue.su ./Drivers/FreeRTOS-Kernel/stream_buffer.cyclo ./Drivers/FreeRTOS-Kernel/stream_buffer.d ./Drivers/FreeRTOS-Kernel/stream_buffer.o ./Drivers/FreeRTOS-Kernel/stream_buffer.su ./Drivers/FreeRTOS-Kernel/tasks.cyclo ./Drivers/FreeRTOS-Kernel/tasks.d ./Drivers/FreeRTOS-Kernel/tasks.o ./Drivers/FreeRTOS-Kernel/tasks.su ./Drivers/FreeRTOS-Kernel/timers.cyclo ./Drivers/FreeRTOS-Kernel/timers.d ./Drivers/FreeRTOS-Kernel/timers.o ./Drivers/FreeRTOS-Kernel/timers.su

.PHONY: clean-Drivers-2f-FreeRTOS-2d-Kernel

