# Makefile for cocotb testbench

TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(wildcard ../rtl/*.sv)

# Default: test gesture accelerator core
TOPLEVEL ?= dvs_gesture_accel
MODULE ?= test_gesture

SIM = icarus

# Enable SystemVerilog
COMPILE_ARGS += -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim

# Convenience targets
.PHONY: test_core test_uart

test_core:
	$(MAKE) TOPLEVEL=dvs_gesture_accel MODULE=test_gesture

test_uart:
	$(MAKE) TOPLEVEL=uart_gesture_top MODULE=test_uart_top
