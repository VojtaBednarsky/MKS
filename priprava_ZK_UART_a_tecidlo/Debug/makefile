################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
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
priprava_ZK_UART_a_tecidlo.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
priprava_ZK_UART_a_tecidlo.list \

OBJCOPY_BIN += \
priprava_ZK_UART_a_tecidlo.bin \


# All Target
all: priprava_ZK_UART_a_tecidlo.elf secondary-outputs

# Tool invocations
priprava_ZK_UART_a_tecidlo.elf: $(OBJS) $(USER_OBJS) C:\Users\vojta\Desktop\MKS\MKS_cvika\MKS\priprava_ZK_UART_a_tecidlo\STM32F302R8TX_FLASH.ld
	arm-none-eabi-gcc -o "priprava_ZK_UART_a_tecidlo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\vojta\Desktop\MKS\MKS_cvika\MKS\priprava_ZK_UART_a_tecidlo\STM32F302R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="priprava_ZK_UART_a_tecidlo.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

priprava_ZK_UART_a_tecidlo.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "priprava_ZK_UART_a_tecidlo.list"
	@echo 'Finished building: $@'
	@echo ' '

priprava_ZK_UART_a_tecidlo.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "priprava_ZK_UART_a_tecidlo.bin"
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
