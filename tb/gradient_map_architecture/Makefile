TOPLEVEL_LANG = verilog

# RTL files for Gradient-Map architecture
RTL_DIR = ../../rtl
VERILOG_SOURCES = $(RTL_DIR)/uart_tx.sv \
                  $(RTL_DIR)/uart_rx.sv \
                  $(RTL_DIR)/uart_debug.sv \
                  $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                  $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                  $(RTL_DIR)/gradient_map_architecture/time_surface_memory.sv \
                  $(RTL_DIR)/gradient_map_architecture/time_surface_encoder.sv \
                  $(RTL_DIR)/gradient_map_architecture/flatten_buffer.sv \
                  $(RTL_DIR)/gradient_map_architecture/weight_rom.sv \
                  $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                  $(RTL_DIR)/gradient_map_architecture/spatio_temporal_classifier.sv \
                  $(RTL_DIR)/gradient_map_architecture/gesture_top.sv

TOPLEVEL = gesture_top
MODULE = test_gradient_map

SIM = icarus

# Enable SystemVerilog
COMPILE_ARGS += -g2012

# Faster simulation: reduce frame period for testing
COMPILE_ARGS += -Pgesture_top.FRAME_PERIOD_MS=1
COMPILE_ARGS += -Pgesture_top.MIN_MASS_THRESH=20
COMPILE_ARGS += -Pgesture_top.UART_RX_ENABLE=1
COMPILE_ARGS += -Pgesture_top.DECAY_SHIFT=12

ifeq ($(OS),Windows_NT)
COCOTB_MAKEFILES := $(shell C:/Users/jason/Documents/FPGA-DVS-Gesture-Classifier/.venv/Scripts/cocotb-config.exe --makefiles)
include $(COCOTB_MAKEFILES)/Makefile.sim
else
include $(shell cocotb-config --makefiles)/Makefile.sim
endif

.PHONY: test clean_all

test:
	$(MAKE) sim

clean_all:
	rm -rf sim_build results.xml __pycache__
