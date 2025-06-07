# Target selection (default: f103)
TARGET ?= f103

# Target-specific configuration
ifeq ($(TARGET),f103)
    $(info Configuring for STM32F103 (Cortex-M3, no FPU))
    MCU := cortex-m3
    FPU_FLAGS := -mfloat-abi=soft
    LDSCRIPT := stm32f103x8.ld
    PROBE_CHIP := STM32F103C8Tx
    TARGET_DEFINE := -DSTM32F103=1
    BIN_PREFIX := stm32-bm-f103
else ifneq (,$(filter $(TARGET),f401 f405 f411))
    $(info Configuring for STM32$(shell echo $(TARGET) | tr '[:lower:]' '[:upper:]') (Cortex-M4, hardware FPU))
    # Common F4 settings
    MCU := cortex-m4
    FPU_FLAGS := -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    TARGET_DEFINE := -DSTM32$(shell echo $(TARGET) | tr '[:lower:]' '[:upper:]')=1
    BIN_PREFIX := stm32-bm-$(TARGET)
    
    # F4-specific settings
    ifeq ($(TARGET),f401)
        $(info Using linker script: stm32f401xe.ld, probe chip: STM32F401RETx)
        LDSCRIPT := stm32f401xe.ld
        PROBE_CHIP := STM32F401RETx
    else ifeq ($(TARGET),f405)
        $(info Using linker script: stm32f405xg.ld, probe chip: STM32F405RGTx)
        LDSCRIPT := stm32f405xg.ld
        PROBE_CHIP := STM32F405RGTx
    else ifeq ($(TARGET),f411)
        $(info Using linker script: stm32f411xe.ld, probe chip: STM32F411RETx)
        LDSCRIPT := stm32f411xe.ld
        PROBE_CHIP := STM32F411RETx
    endif
else
    $(error Unsupported TARGET: $(TARGET). Use f103, f401, f405, or f411)
endif

# Enable debug (RTT logging)
DEBUG := 1

# Toolchain
TOOLCHAIN := /opt/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi
CC := $(TOOLCHAIN)/bin/arm-none-eabi-gcc
OBJDUMP := $(TOOLCHAIN)/bin/arm-none-eabi-objdump
OBJCOPY := $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
SIZE := $(TOOLCHAIN)/bin/arm-none-eabi-size

# Build dir
BUILD_DIR := build

# Output files
ELF := $(BUILD_DIR)/$(BIN_PREFIX).elf
BIN := $(BUILD_DIR)/$(BIN_PREFIX).bin
MAP := $(BUILD_DIR)/$(BIN_PREFIX).map
DISASM := $(BUILD_DIR)/$(BIN_PREFIX).dis

# Source files
SRCS := main.c vector.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

# Segger files
SEGGER_SRCS := segger-rtt/RTT/SEGGER_RTT.c segger-rtt/RTT/SEGGER_RTT_printf.c
SEGGER_OBJS := $(BUILD_DIR)/segger_rtt.o $(BUILD_DIR)/segger_rtt_printf.o
SEGGER_INC := -I segger-rtt/RTT -I segger-rtt/Config 

# Flags
COMMON_FLAGS := -mcpu=$(MCU) -mthumb $(FPU_FLAGS) $(TARGET_DEFINE)
CFLAGS := $(COMMON_FLAGS) -D DEBUG=$(DEBUG) $(SEGGER_INC) -g -nostdlib -O3 -ffunction-sections -fdata-sections -Wall -Wextra -MD -MP
LDFLAGS := $(COMMON_FLAGS) -nostdlib -specs=nosys.specs -specs=nano.specs -Wl,--no-gc-sections -Wl,-Map=$(MAP)

-include $(OBJS:.o=.d)
-include $(SEGGER_OBJS:.o=.d)

# Phony targets
.PHONY: all clean clean-segger segger size flash run

# Default target
all: $(BIN) size

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Explicit compilation rules
$(BUILD_DIR)/main.o: main.c | $(BUILD_DIR) segger-rtt include.h
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/vector.o: vector.c | $(BUILD_DIR) segger-rtt include.h
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/segger_rtt.o: segger-rtt/RTT/SEGGER_RTT.c | $(BUILD_DIR) segger-rtt
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/segger_rtt_printf.o: segger-rtt/RTT/SEGGER_RTT_printf.c | $(BUILD_DIR) segger-rtt
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Link
$(ELF): $(OBJS) $(SEGGER_OBJS) $(LDSCRIPT) | $(BUILD_DIR)
	$(CC) $(LDFLAGS) -T $(LDSCRIPT) $(OBJS) $(SEGGER_OBJS) -o $@

# Generate binary
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	$(OBJDUMP) -D -S -t -h $< > $(DISASM)
	@echo "Build complete: $@ ($(TARGET))"

# Clone the segger repo
segger-rtt:
	git clone https://github.com/piersfinlayson/segger-rtt.git

# Size information
size: $(ELF)
	$(SIZE) $(ELF)
	@ls -ltr $(BIN) 

# Flash target
flash: $(ELF)
	probe-rs download --chip $(PROBE_CHIP) $<

# Run target
run: $(ELF)
	probe-rs run --probe 2e8a:000c --chip $(PROBE_CHIP) $<

# Clean segger
clean-segger-src:
	rm -rf segger-rtt/

clean:
	rm -rf $(BUILD_DIR)