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

# Default test target
TEST ?=

ifeq ($(TEST),)
  VERILOG_SOURCES = $(VOXEL_BIN_SOURCES) \
                    $(RTL_DIR)/voxel_bin_architecture/voxel_bin_top.sv
  TOPLEVEL = voxel_bin_top
  COCOTB_TEST_MODULES   = test_voxel_bin
    COMPILE_ARGS += -Pvoxel_bin_top.CYCLES_PER_BIN=2000
    COMPILE_ARGS += -Pvoxel_bin_top.BAUD_RATE=3000000
    COMPILE_ARGS += -Pvoxel_bin_top.WINDOW_MS=40
  else ifeq ($(TEST),core)
    VERILOG_SOURCES = $(VOXEL_BIN_SOURCES)
    TOPLEVEL = voxel_bin_core
    COCOTB_TEST_MODULES   = test_voxel_bin_core
    COMPILE_ARGS += -Pvoxel_bin_core.CYCLES_PER_BIN=64
    COMPILE_ARGS += -Pvoxel_bin_core.WINDOW_MS=40
  else ifeq ($(TEST),input_fifo)
    VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/input_fifo.sv
    TOPLEVEL = input_fifo
    COCOTB_TEST_MODULES   = test_input_fifo
    COMPILE_ARGS += -Pinput_fifo.DEPTH=8
    COMPILE_ARGS += -Pinput_fifo.PTR_BITS=3
  else ifeq ($(TEST),evt2_decoder)
    VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/evt2_decoder.sv
    TOPLEVEL = evt2_decoder
    COCOTB_TEST_MODULES   = test_evt2_decoder
  else ifeq ($(TEST),voxel_binning)
    VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/voxel_binning.sv
    TOPLEVEL = voxel_binning
    COCOTB_TEST_MODULES   = test_voxel_binning
    COMPILE_ARGS += -Pvoxel_binning.CYCLES_PER_BIN=100
  else ifeq ($(TEST),systolic_array)
    VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/systolic_array.sv
    TOPLEVEL = systolic_array
    COCOTB_TEST_MODULES   = test_systolic_array
    COMPILE_ARGS += -Psystolic_array.NUM_CELLS=16
  else ifeq ($(TEST),gesture_classifier)
    VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/gesture_classifier.sv
    TOPLEVEL = gesture_classifier
    COCOTB_TEST_MODULES   = test_gesture_classifier
else ifeq ($(TEST),weight_ram)
  VERILOG_SOURCES = $(RTL_DIR)/voxel_bin_architecture/weight_ram.sv
  TOPLEVEL = weight_ram
  COCOTB_TEST_MODULES   = test_weight_ram
  COMPILE_ARGS += -Pweight_ram.CLASS_IDX=0
else
  $(error Unknown TEST=$(TEST). Use: core, input_fifo, evt2_decoder, voxel_binning, systolic_array, gesture_classifier, weight_ram)
endif

SIM = icarus
# Keep per-target build artifacts separate so switching TEST values does not
# reuse an incompatible sim.vvp from a previous run.
SIM_BUILD ?= sim_build_$(if $(TEST),$(TEST),default)

COMPILE_ARGS += -g2012

# Suppress waveform dumping and reduce log verbosity for faster runs
export WAVES ?= 0
export COCOTB_LOG_LEVEL ?= INFO
export COCOTB_REDUCED_LOG_FMT ?= 1
export GPI_LOG_LEVEL ?= CRITICAL

# On Windows, locate the venv and OSS CAD Suite relative to the project root,
# falling back to any installation already on PATH.
ifeq ($(OS),Windows_NT)
  # GNU Make defaults to sh.exe which is often absent on Windows PATH.
  # Bootstrap a POSIX shell + utilities from Git-for-Windows (via 8.3 path
  # to avoid spaces) before any $(shell) call.
  _GIT_USR_BIN := $(firstword $(wildcard C:/PROGRA~1/Git/usr/bin) \
                               $(wildcard C:/PROGRA~2/Git/usr/bin))
  ifneq ($(_GIT_USR_BIN),)
    SHELL := $(_GIT_USR_BIN)/sh.exe
  endif

  PROJECT_ROOT := $(realpath ../..)
  VENV_SCRIPTS := $(PROJECT_ROOT)/.venv/Scripts

  # Detect oss-cad-suite: project-local, nested project-local, or user-level
  _OSS_PROJ     := $(PROJECT_ROOT)/oss-cad-suite
  _OSS_NESTED   := $(PROJECT_ROOT)/oss-cad-suite/oss-cad-suite
  _OSS_USER     := $(subst \,/,$(USERPROFILE))/Documents/oss-cad-suite
  ifneq ($(wildcard $(_OSS_PROJ)/bin/iverilog.exe),)
    _OSS_ROOT := $(_OSS_PROJ)
  else ifneq ($(wildcard $(_OSS_NESTED)/bin/iverilog.exe),)
    _OSS_ROOT := $(_OSS_NESTED)
  else ifneq ($(wildcard $(_OSS_USER)/bin/iverilog.exe),)
    _OSS_ROOT := $(_OSS_USER)
  else
    _OSS_ROOT :=
  endif

  ifneq ($(_OSS_ROOT),)
    OSS_CAD_BIN := $(_OSS_ROOT)/bin
    OSS_CAD_LIB := $(_OSS_ROOT)/lib
  else
    OSS_CAD_BIN :=
    OSS_CAD_LIB :=
  endif

  export PYTHONNOUSERSITE := 1
  export PYTHONPATH := $(PROJECT_ROOT)/.venv/Lib/site-packages
  export PATH := $(VENV_SCRIPTS):$(OSS_CAD_BIN):$(OSS_CAD_LIB):$(_GIT_USR_BIN):$(PATH)

  COCOTB_MAKEFILES := $(shell "$(VENV_SCRIPTS)/cocotb-config.exe" --makefiles 2>/dev/null)
  include $(COCOTB_MAKEFILES)/Makefile.sim
else
  include $(shell cocotb-config --makefiles)/Makefile.sim
endif

.PHONY: test test_full test_unit clean_all

test:
	$(MAKE) sim

test_unit:
	@for t in core input_fifo evt2_decoder voxel_binning systolic_array gesture_classifier weight_ram; do \
		echo "======== Running TEST=$$t ========"; \
		$(MAKE) clean > /dev/null 2>&1; \
		$(MAKE) sim TEST=$$t || exit 1; \
	done

test_full:
	$(MAKE) clean && $(MAKE) sim

clean_all:
	rm -rf sim_build* results.xml __pycache__


