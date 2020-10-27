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
SoftwareDownloader_BootLoader.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SoftwareDownloader_BootLoader.list \

OBJCOPY_BIN += \
SoftwareDownloader_BootLoader.bin \


# All Target
all: SoftwareDownloader_BootLoader.elf secondary-outputs

# Tool invocations
SoftwareDownloader_BootLoader.elf: $(OBJS) $(USER_OBJS) D:\STM32CubeIDE\workspace_1.0.2\SoftwareDownloader_BootLoader\STM32F429ZITX_FLASH.ld
	arm-none-eabi-gcc -o "SoftwareDownloader_BootLoader.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\STM32CubeIDE\workspace_1.0.2\SoftwareDownloader_BootLoader\STM32F429ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SoftwareDownloader_BootLoader.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SoftwareDownloader_BootLoader.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SoftwareDownloader_BootLoader.list"
	@echo 'Finished building: $@'
	@echo ' '

SoftwareDownloader_BootLoader.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SoftwareDownloader_BootLoader.bin"
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
