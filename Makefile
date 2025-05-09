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
BIN_PREFIX := stm32-bm
ELF := $(BUILD_DIR)/$(BIN_PREFIX).elf
BIN := $(BUILD_DIR)/$(BIN_PREFIX).bin
MAP := $(BUILD_DIR)/$(BIN_PREFIX).map
DISASM := $(BUILD_DIR)/$(BIN_PREFIX).dis

# Linker script
LDSCRIPT := stm32f103x8.ld

# Source files
SRCS := main.c vector.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

# Segger files
SEGGER_SRCS := segger-rtt/RTT/SEGGER_RTT.c segger-rtt/RTT/SEGGER_RTT_printf.c
SEGGER_OBJS := $(BUILD_DIR)/segger_rtt.o $(BUILD_DIR)/segger_rtt_printf.o
SEGGER_INC := -I segger-rtt/RTT -I segger-rtt/Config 

# Flags
COMMON_FLAGS := -mcpu=cortex-m3 -mthumb
CFLAGS := $(COMMON_FLAGS) -D DEBUG=$(DEBUG) $(SEGGER_INC) -g -nostdlib -mfloat-abi=soft -O3 -ffunction-sections -fdata-sections -Wall -Wextra -MD -MP
LDFLAGS := $(COMMON_FLAGS) -nostdlib -specs=nosys.specs -specs=nano.specs -Wl,--no-gc-sections -Wl,-Map=$(MAP)

-include $(OBJS:.o=.d)
-include $(SEGGER_OBJS:.o=.d)

# Phony targets
.PHONY: all clean clean-segger segger size flash

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
	@echo "Build complete: $@"

# Clone the segger repo
segger-rtt:
	git clone https://github.com/piersfinlayson/segger-rtt.git

# Size information
size: $(ELF)
	$(SIZE) $(ELF)
	@ls -ltr $(BIN) 

# Flash target
flash: $(ELF)
	probe-rs download --chip STM32F103C8Tx $<

# Run target
run: $(ELF)
#	probe-rs run --probe 2e8a:000c --chip STM32F103C8Tx $<
	probe-rs run --chip STM32F103C8Tx $<

# Clean segger
clean-segger-src:
	rm -rf segger-rtt/

clean:
	rm -rf $(BUILD_DIR)
