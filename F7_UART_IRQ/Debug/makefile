################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
F7_UART_IRQ.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F7_UART_IRQ.list \


# All Target
all: F7_UART_IRQ.elf secondary-outputs

# Tool invocations
F7_UART_IRQ.elf: $(OBJS) $(USER_OBJS) C:\Projects\ARM\STM_projects\F7_UART_IRQ\STM32F746ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "F7_UART_IRQ.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Projects\ARM\STM_projects\F7_UART_IRQ\STM32F746ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F7_UART_IRQ.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F7_UART_IRQ.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F7_UART_IRQ.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
