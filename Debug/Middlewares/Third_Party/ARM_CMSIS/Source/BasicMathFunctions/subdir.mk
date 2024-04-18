################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/BasicMathFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/BayesFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/CommonTables" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/ComplexMathFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/ControllerFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/DistanceFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/FastMathFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/FilteringFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/InterpolationFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/MatrixFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/QuaternionMathFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/StatisticsFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/SupportFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/SVMFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/TransformFunctions" -I"C:/Users/user/Documents/GitHub/LAB02-ex01/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-BasicMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-BasicMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-BasicMathFunctions

