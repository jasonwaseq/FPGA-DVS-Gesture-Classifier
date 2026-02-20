TOPLEVEL_LANG = verilog

# RTL sources for voxel-bin architecture (UART top + accelerator)
RTL_DIR = ../../rtl
VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/dvs_gesture_accel.sv \
                  $(RTL_DIR)/voxel_bin_architecture/TemporalAccumulator.sv \
                  $(RTL_DIR)/voxel_bin_architecture/SpatialCompressor.sv \
                  $(RTL_DIR)/voxel_bin_architecture/OutputRegister.sv \
                  $(RTL_DIR)/voxel_bin_architecture/MotionComputer.sv \
                  $(RTL_DIR)/voxel_bin_architecture/InputFIFO.sv \
                  $(RTL_DIR)/voxel_bin_architecture/TimeSurfaceBinning.sv \
                  $(RTL_DIR)/voxel_bin_architecture/WeightROM.sv \
                  $(RTL_DIR)/voxel_bin_architecture/SystolicMatrixMultiply.sv \
                  $(RTL_DIR)/voxel_bin_architecture/voxel_bin_top.sv \
                  $(RTL_DIR)/uart_rx.sv \
                  $(RTL_DIR)/uart_tx.sv

# Default: test UART top module (full voxel-bin system)
TOPLEVEL ?= voxel_bin_top
MODULE ?= test_voxel_bin

SIM = icarus

# Enable SystemVerilog
COMPILE_ARGS += -g2012

# Override CYCLES_PER_BIN for fast simulation
# Use 600 cycles (enough for basic testing, classification will be truncated)
COMPILE_ARGS += -Pvoxel_bin_top.CYCLES_PER_BIN=600

ifeq ($(OS),Windows_NT)
COCOTB_MAKEFILES := $(shell C:/Users/jason/Documents/FPGA-DVS-Gesture-Classifier/.venv/Scripts/cocotb-config.exe --makefiles)
include $(COCOTB_MAKEFILES)/Makefile.sim
else
include $(shell cocotb-config --makefiles)/Makefile.sim
endif

.PHONY: test test_full clean_all

test:
	$(MAKE) TOPLEVEL=voxel_bin_top MODULE=test_voxel_bin

test_full:
	$(MAKE) clean
	$(MAKE) test

clean_all:
	rm -rf sim_build results.xml __pycache__

