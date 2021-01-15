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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
blink_4leds_STM32F407.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
blink_4leds_STM32F407.list \

OBJCOPY_BIN += \
blink_4leds_STM32F407.bin \


# All Target
all: blink_4leds_STM32F407.elf secondary-outputs

# Tool invocations
blink_4leds_STM32F407.elf: $(OBJS) $(USER_OBJS) /home/lucho/STM32CubeIDE/workspaces/prueba01/blink_4leds_STM32F407/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "blink_4leds_STM32F407.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/lucho/STM32CubeIDE/workspaces/prueba01/blink_4leds_STM32F407/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="blink_4leds_STM32F407.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

blink_4leds_STM32F407.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "blink_4leds_STM32F407.list"
	@echo 'Finished building: $@'
	@echo ' '

blink_4leds_STM32F407.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "blink_4leds_STM32F407.bin"
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
