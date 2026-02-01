# Makefile for cocotb testbench
# DVS Gesture Accelerator with Single Sliding-Window Ring Buffer

TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(wildcard ../rtl/*.sv)

# Default: test UART top module (full system)
TOPLEVEL ?= uart_gesture_top
MODULE ?= test_dvs_gesture

SIM = icarus

# Enable SystemVerilog
COMPILE_ARGS += -g2012

# Override CYCLES_PER_BIN for fast simulation
# Use 600 cycles (enough for basic testing, classification will be truncated)
COMPILE_ARGS += -Puart_gesture_top.CYCLES_PER_BIN=600

ifeq ($(OS),Windows_NT)
COCOTB_MAKEFILES := $(shell C:/Users/jason/Documents/FPGA-DVS-Gesture-Classifier/.venv/Scripts/cocotb-config.exe --makefiles)
include $(COCOTB_MAKEFILES)/Makefile.sim
else
include $(shell cocotb-config --makefiles)/Makefile.sim
endif

# Convenience targets
.PHONY: test test_full clean_all

test:
	$(MAKE) TOPLEVEL=uart_gesture_top MODULE=test_dvs_gesture

test_full:
	$(MAKE) clean
	$(MAKE) test

clean_all:
	rm -rf sim_build results.xml __pycache__
