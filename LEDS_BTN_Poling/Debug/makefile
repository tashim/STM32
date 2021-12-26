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
LEDS_BTN_Poling.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LEDS_BTN_Poling.list \


# All Target
all: LEDS_BTN_Poling.elf secondary-outputs

# Tool invocations
LEDS_BTN_Poling.elf: $(OBJS) $(USER_OBJS) C:\Projects\ARM\STM_projects\LEDS_BTN_Poling\STM32F746ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "LEDS_BTN_Poling.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Projects\ARM\STM_projects\LEDS_BTN_Poling\STM32F746ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LEDS_BTN_Poling.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LEDS_BTN_Poling.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LEDS_BTN_Poling.list"
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
