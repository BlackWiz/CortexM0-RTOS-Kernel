# Makefile for Complete RTOS Dissertation Project
# STM32G071RB with ARM Cortex-M0+
#
# M.Tech Dissertation - Building a Real-Time Operating System
# Student: Josyula Sri Hari Shankar Sharma (2023HT01571)
#
# Build Targets:
#   make              - Build main demonstration
#   make test         - Build test suite
#   make flash        - Flash demonstration to board
#   make test-flash   - Flash test suite to board
#   make size         - Show memory usage
#   make clean        - Remove build artifacts

# ============================================
# CONFIGURATION
# ============================================

TARGET = rtos_demo
TEST_TARGET = rtos_test

CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Source files for main demonstration
C_SRCS = main.c rtos.c startup.c syscalls.c uart.c delay.c
ASM_SRCS = context_switch.s

# Source files for test suite  
TEST_C_SRCS = test_suite.c rtos.c startup.c syscalls.c uart.c delay.c
TEST_ASM_SRCS = context_switch.s

# Object files (derived from sources)
OBJS = $(C_SRCS:.c=.o) $(ASM_SRCS:.s=.o)
TEST_OBJS = $(TEST_C_SRCS:.c=.o) $(TEST_ASM_SRCS:.s=.o)

# ============================================
# COMPILER FLAGS
# ============================================

# CPU configuration
CPU = -mcpu=cortex-m0plus
MCU = $(CPU) -mthumb

# C compiler flags
CFLAGS = $(MCU)
CFLAGS += -Wall -Wextra -Werror
CFLAGS += -g3                           # Full debug symbols
CFLAGS += -Os                           # Optimize for size
CFLAGS += -std=c11
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-builtin
CFLAGS += -DRTOS_DEMO                   # Define for conditional compilation

# Test build flags
TEST_CFLAGS = $(CFLAGS)
TEST_CFLAGS += -DRTOS_TEST_BUILD

# Assembler flags
ASFLAGS = $(MCU)
ASFLAGS += -g

# Linker flags
LDFLAGS = $(MCU)
LDFLAGS += -T Linker.ld
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += -Wl,--gc-sections             # Remove unused code
LDFLAGS += -Wl,--print-memory-usage      # Show memory usage
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nosys.specs

TEST_LDFLAGS = $(MCU)
TEST_LDFLAGS += -T Linker.ld
TEST_LDFLAGS += -Wl,-Map=$(TEST_TARGET).map
TEST_LDFLAGS += -Wl,--gc-sections
TEST_LDFLAGS += -Wl,--print-memory-usage
TEST_LDFLAGS += -nostartfiles
TEST_LDFLAGS += --specs=nosys.specs

# ============================================
# BUILD RULES
# ============================================

.PHONY: all test clean flash test-flash disasm size help

# Default target: build main demonstration
all: $(TARGET).elf
	@echo "========================================"
	@echo "Build complete: $(TARGET).elf"
	@echo "========================================"
	@$(SIZE) $(TARGET).elf
	@echo ""
	@echo "To flash: make flash"

# Test suite target
test: $(TEST_TARGET).elf
	@echo "========================================"
	@echo "Test build complete: $(TEST_TARGET).elf"
	@echo "========================================"
	@$(SIZE) $(TEST_TARGET).elf
	@echo ""
	@echo "To flash: make test-flash"

# Link main demonstration
$(TARGET).elf: $(OBJS)
	@echo "Linking $@..."
	$(CC) $(LDFLAGS) -o $@ $^

# Link test suite
$(TEST_TARGET).elf: $(TEST_OBJS)
	@echo "Linking test suite $@..."
	$(CC) $(TEST_LDFLAGS) -o $@ $^

# Compile C source files
%.o: %.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c -o $@ $<

# Compile test C files
test_suite.o: test_suite.c
	@echo "Compiling test suite $<..."
	$(CC) $(TEST_CFLAGS) -c -o $@ $<

# Assemble assembly files
%.o: %.s
	@echo "Assembling $<..."
	$(AS) $(ASFLAGS) -o $@ $<

# Flash main demonstration to target board
flash: $(TARGET).elf
	@echo "Flashing demonstration to STM32G071RB..."
	openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
		-c "program $(TARGET).elf verify reset exit"

# Flash test suite to target board
test-flash: $(TEST_TARGET).elf
	@echo "Flashing test suite to STM32G071RB..."
	openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
		-c "program $(TEST_TARGET).elf verify reset exit"

# Generate disassembly for main demonstration
disasm: $(TARGET).elf
	@echo "Generating disassembly..."
	$(OBJDUMP) -d -S $< > $(TARGET).asm
	@echo "Disassembly written to $(TARGET).asm"

# Generate disassembly for test suite
test-disasm: $(TEST_TARGET).elf
	@echo "Generating test disassembly..."
	$(OBJDUMP) -d -S $< > $(TEST_TARGET).asm
	@echo "Disassembly written to $(TEST_TARGET).asm"

# Show detailed size breakdown
size: $(TARGET).elf
	@echo "========================================"
	@echo "Memory usage breakdown:"
	@echo "========================================"
	@$(SIZE) -A $(TARGET).elf
	@echo ""
	@$(SIZE) -B $(TARGET).elf
	@echo ""
	@echo "Dissertation targets:"
	@echo "  - Context switch: < 10 µs (measured: 4.81 µs)"
	@echo "  - Memory alloc: < 1 µs (O(1) constant time)"
	@echo "  - Kernel footprint: < 10 KB FLASH"
	@echo ""

# Show test suite size
test-size: $(TEST_TARGET).elf
	@echo "========================================"
	@echo "Test suite memory usage:"
	@echo "========================================"
	@$(SIZE) -A $(TEST_TARGET).elf
	@echo ""
	@$(SIZE) -B $(TEST_TARGET).elf

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -f *.o *.elf *.map *.asm
	@echo "Clean complete."

# ============================================
# DEPENDENCY TRACKING
# ============================================

# Header dependencies
main.o: main.c rtos.h
rtos.o: rtos.c rtos.h
startup.o: startup.c
syscalls.o: syscalls.c
context_switch.o: context_switch.s
test_suite.o: test_suite.c rtos.h
uart.o: uart.c uart.h
delay.o: delay.c delay.h

# ============================================
# PERFORMANCE BENCHMARKING
# ============================================

.PHONY: benchmark

benchmark: $(TARGET).elf
	@echo "========================================"
	@echo "Performance Benchmark Instructions"
	@echo "========================================"
	@echo ""
	@echo "1. Context Switch Time:"
	@echo "   - Connect logic analyzer to PB8"
	@echo "   - Add GPIO toggle in PendSV_Handler"
	@echo "   - Measure pulse width"
	@echo "   - Target: < 10 µs @ 16 MHz"
	@echo ""
	@echo "2. Memory Allocation Time:"
	@echo "   - Use DWT cycle counter"
	@echo "   - Measure rtos_mem_alloc() execution"
	@echo "   - Target: O(1) constant time"
	@echo ""
	@echo "3. Interrupt Latency:"
	@echo "   - Trigger external interrupt"
	@echo "   - Measure time to ISR entry"
	@echo "   - Target: < 2 µs @ 16 MHz"
	@echo ""

# ============================================
# HELP
# ============================================

help:
	@echo "RTOS Dissertation Project - Complete Implementation"
	@echo "========================================"
	@echo "Available targets:"
	@echo "  make              - Build main demonstration"
	@echo "  make test         - Build test suite"
	@echo "  make flash        - Flash demonstration to board"
	@echo "  make test-flash   - Flash test suite to board"
	@echo "  make disasm       - Generate disassembly (main)"
	@echo "  make test-disasm  - Generate disassembly (test)"
	@echo "  make size         - Show memory usage"
	@echo "  make test-size    - Show test suite memory usage"
	@echo "  make benchmark    - Show benchmark instructions"
	@echo "  make clean        - Remove build artifacts"
	@echo "  make help         - Show this help"
	@echo ""
	@echo "Hardware: STM32G071RB (Cortex-M0+)"
	@echo "  - FLASH: 128 KB"
	@echo "  - RAM: 36 KB"
	@echo "  - Clock: 16 MHz HSI"
	@echo ""
	@echo "Dissertation Features Implemented:"
	@echo "  ✓ Priority-based preemptive scheduler (O(1))"
	@echo "  ✓ Semaphores"
	@echo "  ✓ Mutexes with priority inheritance"
	@echo "  ✓ Message queues"
	@echo "  ✓ Fixed-block memory allocator"
	@echo "  ✓ Hardware abstraction layer"
	@echo "  ✓ Tickless idle mode"
	@echo "  ✓ Stack overflow detection"
	@echo "========================================"
