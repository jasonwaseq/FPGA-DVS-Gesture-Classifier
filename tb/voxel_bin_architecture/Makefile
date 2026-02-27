TOPLEVEL_LANG = verilog

RTL_DIR = ../../rtl

VOXEL_BIN_SOURCES = $(RTL_DIR)/voxel_bin_architecture/voxel_bin_core.sv \
                    $(RTL_DIR)/voxel_bin_architecture/input_fifo.sv \
                    $(RTL_DIR)/voxel_bin_architecture/evt2_decoder.sv \
                    $(RTL_DIR)/voxel_bin_architecture/voxel_binning.sv \
                    $(RTL_DIR)/voxel_bin_architecture/systolic_array.sv \
                    $(RTL_DIR)/voxel_bin_architecture/gesture_classifier.sv \
                    $(RTL_DIR)/voxel_bin_architecture/weight_ram.sv \
                    $(RTL_DIR)/uart_rx.sv \
                    $(RTL_DIR)/uart_tx.sv

# Default target: raw (UART sensor input)
TARGET ?= raw

ifeq ($(TARGET),raw)
  VERILOG_SOURCES = $(VOXEL_BIN_SOURCES) \
                    $(RTL_DIR)/voxel_bin_architecture/voxel_bin_top.sv
  TOPLEVEL = voxel_bin_top
  MODULE   = test_voxel_bin_raw
  COMPILE_ARGS += -Pvoxel_bin_top.CYCLES_PER_BIN=2000
  COMPILE_ARGS += -Pvoxel_bin_top.BAUD_RATE=3000000
  COMPILE_ARGS += -Pvoxel_bin_top.WINDOW_MS=40
else
  $(error Unknown TARGET=$(TARGET). Only TARGET=raw is supported)
endif

SIM = icarus

COMPILE_ARGS += -g2012

# Suppress waveform dumping and reduce log verbosity for faster runs
export WAVES ?= 0
export COCOTB_LOG_LEVEL ?= WARNING
export COCOTB_REDUCED_LOG_FMT ?= 1

# On Windows, locate the venv and OSS CAD Suite relative to the project root,
# falling back to any installation already on PATH.
ifeq ($(OS),Windows_NT)
  PROJECT_ROOT := $(realpath ../..)
  VENV_SCRIPTS := $(PROJECT_ROOT)/.venv/Scripts
  OSS_CAD_LOCAL := $(PROJECT_ROOT)/oss-cad-suite/bin
  ifneq ($(wildcard $(OSS_CAD_LOCAL)/iverilog.exe),)
    OSS_CAD_BIN := $(OSS_CAD_LOCAL)
  else
    OSS_CAD_BIN :=
  endif
  export PATH := $(VENV_SCRIPTS):$(OSS_CAD_BIN):$(PATH)
  COCOTB_MAKEFILES := $(shell "$(VENV_SCRIPTS)/cocotb-config.exe" --makefiles 2>/dev/null)
  include $(COCOTB_MAKEFILES)/Makefile.sim
  SHELL := $(shell where bash.exe 2>/dev/null | head -1)
  ifeq ($(SHELL),)
    SHELL := cmd.exe
  endif
else
  include $(shell cocotb-config --makefiles)/Makefile.sim
endif

.PHONY: test test_raw test_processed test_full clean_all

test: test_raw

test_raw:
	$(MAKE) sim TARGET=raw

test_processed:
	$(MAKE) sim TARGET=processed

test_full:
	$(MAKE) clean && $(MAKE) sim TARGET=raw
	$(MAKE) clean && $(MAKE) sim TARGET=processed

clean_all:
	rm -rf sim_build results.xml __pycache__
