.PHONY: help setup doctor test synth flash clean

ifeq ($(OS),Windows_NT)
PYTHON ?= python
else
PYTHON ?= python3
endif
WORKFLOW ?= setup.py

SKIP_FPGA ?= 0
PORT ?=
SERIAL ?=
VID ?=
PID ?=

PRIMARY_TARGET := $(firstword $(MAKECMDGOALS))
TEST_MODULE := $(word 2,$(MAKECMDGOALS))
ifneq ($(PRIMARY_TARGET),test)
TEST_MODULE :=
endif

FLASH_ARGS :=
ifneq ($(strip $(PORT)),)
FLASH_ARGS += --port $(PORT)
endif
ifneq ($(strip $(SERIAL)),)
FLASH_ARGS += --serial $(SERIAL)
endif
ifneq ($(strip $(VID)),)
FLASH_ARGS += --vid $(VID)
endif
ifneq ($(strip $(PID)),)
FLASH_ARGS += --pid $(PID)
endif

help:
	@echo Top-level workflow targets:
	@echo   make setup [SKIP_FPGA=1]
	@echo   make doctor
	@echo   make test [module_name]   # default module=all
	@echo   make synth
	@echo   make flash [PORT=COMx] [SERIAL=id] [VID=0x0403] [PID=0x6010]
	@echo   make clean
	@echo
	@echo Examples:
	@echo   make setup
	@echo   make setup SKIP_FPGA=1
	@echo   make doctor
	@echo   make test
	@echo   make test input_fifo
	@echo   make synth
	@echo   make flash PORT=COM3

setup:
ifeq ($(SKIP_FPGA),1)
	$(PYTHON) $(WORKFLOW) setup --skip-fpga
else
	$(PYTHON) $(WORKFLOW) setup
endif

doctor:
	$(PYTHON) $(WORKFLOW) doctor

test:
	$(PYTHON) $(WORKFLOW) test $(if $(TEST_MODULE),$(TEST_MODULE),all)

synth:
	$(PYTHON) $(WORKFLOW) synth voxel_bin

flash:
	$(PYTHON) $(WORKFLOW) flash voxel_bin $(FLASH_ARGS)

clean:
	$(PYTHON) $(WORKFLOW) clean

ifeq ($(PRIMARY_TARGET),test)
%:
	@:
endif
