# Ice40 Synthesis Makefile for DVS Gesture Accelerator

PROJ = uart_gesture_top
PCF = icebreaker.pcf
DEVICE = up5k
PACKAGE = sg48

RTL_DIR = ../rtl
RTL_FILES = $(RTL_DIR)/uart_rx.sv \
            $(RTL_DIR)/uart_tx.sv \
            $(RTL_DIR)/event_accumulator.sv \
            $(RTL_DIR)/gesture_classifier.sv \
            $(RTL_DIR)/dvs_gesture_accel.sv \
            $(RTL_DIR)/uart_gesture_top.sv

.PHONY: all clean prog

all: $(PROJ).bit

# Convert SystemVerilog to Verilog for Yosys
$(PROJ).v: $(RTL_FILES)
	sv2v $(RTL_FILES) -w $@

# Synthesis
$(PROJ).json: $(PROJ).v
	yosys -p "read_verilog $<; synth_ice40 -top $(PROJ) -json $@"

# Place and Route
$(PROJ).asc: $(PROJ).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PCF) --asc $@

# Bitstream generation
$(PROJ).bit: $(PROJ).asc
	icepack $< $@

# Program the FPGA
prog: $(PROJ).bit
	iceprog $<

# Resource utilization report
report: $(PROJ).json
	yosys -p "read_json $<; stat"

clean:
	rm -f $(PROJ).v $(PROJ).json $(PROJ).asc $(PROJ).bit
