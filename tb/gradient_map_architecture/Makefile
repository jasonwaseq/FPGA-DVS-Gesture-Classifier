TOPLEVEL_LANG = verilog

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

COMPILE_ARGS += -g2012
COMPILE_ARGS += -Pgesture_top.FRAME_PERIOD_MS=1
COMPILE_ARGS += -Pgesture_top.MIN_MASS_THRESH=20
COMPILE_ARGS += -Pgesture_top.UART_RX_ENABLE=1
COMPILE_ARGS += -Pgesture_top.DECAY_SHIFT=12

# Suppress waveform dumping and reduce log verbosity for faster runs
export WAVES ?= 0
export COCOTB_LOG_LEVEL ?= WARNING
export COCOTB_REDUCED_LOG_FMT ?= 1

# On Windows, locate the venv and OSS CAD Suite relative to the project root,
# falling back to any installation already on PATH.
ifeq ($(OS),Windows_NT)
  PROJECT_ROOT := $(realpath ../..)
  VENV_SCRIPTS := $(PROJECT_ROOT)/.venv/Scripts
  # Prefer the project-local OSS CAD Suite; fall back to a system install.
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

.PHONY: test test_full clean_all

test:
	$(MAKE) sim

test_full:
	$(MAKE) clean
	$(MAKE) sim

clean_all:
	rm -rf sim_build results.xml __pycache__
