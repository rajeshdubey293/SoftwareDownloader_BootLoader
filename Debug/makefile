################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BootLoader-STM32F429.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BootLoader-STM32F429.list \

OBJCOPY_BIN += \
BootLoader-STM32F429.bin \


# All Target
all: BootLoader-STM32F429.elf secondary-outputs

# Tool invocations
BootLoader-STM32F429.elf: $(OBJS) $(USER_OBJS) D:\STM32CubeIDE\workspace_1.0.2\BootLoader-STM32F429\STM32F429ZITX_FLASH.ld
	arm-none-eabi-gcc -o "BootLoader-STM32F429.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\STM32CubeIDE\workspace_1.0.2\BootLoader-STM32F429\STM32F429ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BootLoader-STM32F429.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BootLoader-STM32F429.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BootLoader-STM32F429.list"
	@echo 'Finished building: $@'
	@echo ' '

BootLoader-STM32F429.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BootLoader-STM32F429.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
