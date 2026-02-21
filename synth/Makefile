PROJ = gradient_map_top
PCF = icebreaker.pcf
DEVICE = up5k
PACKAGE = sg48

RTL_DIR = ../rtl
RTL_FILES = $(RTL_DIR)/uart_tx.sv \
            $(RTL_DIR)/uart_rx.sv \
            $(RTL_DIR)/uart_debug.sv \
            $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
            $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
            $(RTL_DIR)/gradient_map_architecture/time_surface_memory.sv \
            $(RTL_DIR)/gradient_map_architecture/time_surface_encoder.sv \
            $(RTL_DIR)/gradient_map_architecture/weight_rom.sv \
            $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
            $(RTL_DIR)/gradient_map_architecture/spatio_temporal_classifier.sv \
            $(RTL_DIR)/gradient_map_architecture/gesture_top.sv \
            $(RTL_DIR)/gradient_map_architecture/gradient_map_top.sv

.PHONY: all clean prog report timing

all: $(PROJ).bit

$(PROJ).v: $(RTL_FILES)
	sv2v $(RTL_FILES) -w $@

$(PROJ).json: $(PROJ).v
	yosys -p "read_verilog $<; synth_ice40 -top $(PROJ) -json $@"

$(PROJ).asc: $(PROJ).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PCF) --asc $@ --freq 12

$(PROJ).bit: $(PROJ).asc
	icepack $< $@

prog: $(PROJ).bit
	iceprog $<

report: $(PROJ).json
	yosys -p "read_json $<; stat"

timing: $(PROJ).asc
	icetime -d $(DEVICE) -m -r $(PROJ).timing $<

clean:
	rm -f $(PROJ).v $(PROJ).json $(PROJ).asc $(PROJ).bit $(PROJ).timing

