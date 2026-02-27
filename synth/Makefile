PCF    = icebreaker.pcf
DEVICE = up5k
PACKAGE = sg48

RTL_DIR = ../rtl

# Select architecture:
#   ARCH=gradient_map  -> gradient_map_top (UART EVT2.0 words)
#   ARCH=voxel_bin     -> voxel_bin_top    (UART EVT2.0 words)
ARCH   ?= gradient_map
TARGET ?= raw

# ── gradient_map sources ────────────────────────────────────────────────────
# Raw pipeline: UART RX -> 5-byte assembler -> gradient_map_core -> UART TX
GRADIENT_MAP_CORE = $(RTL_DIR)/uart_tx.sv \
                    $(RTL_DIR)/uart_rx.sv \
                    $(RTL_DIR)/uart_debug.sv \
                    $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                    $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                    $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv \
                    $(RTL_DIR)/gradient_map_architecture/gradient_map_core.sv \
                    $(RTL_DIR)/gradient_map_architecture/weight_ram.sv \
                    $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                    $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv

# Processed pipeline: preprocessed events -> gradient_map_core -> UART TX
GRADIENT_MAP_PROCESSED_CORE = $(RTL_DIR)/uart_tx.sv \
                               $(RTL_DIR)/uart_rx.sv \
                               $(RTL_DIR)/uart_debug.sv \
                               $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                               $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                               $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv \
                               $(RTL_DIR)/gradient_map_architecture/gradient_map_core.sv \
                               $(RTL_DIR)/gradient_map_architecture/weight_ram.sv \
                               $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                               $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv

# ── voxel_bin sources ───────────────────────────────────────────────────────
VOXEL_BIN_CORE = $(RTL_DIR)/uart_tx.sv \
                 $(RTL_DIR)/uart_rx.sv \
                 $(RTL_DIR)/voxel_bin_architecture/input_fifo.sv \
                 $(RTL_DIR)/voxel_bin_architecture/evt2_decoder.sv \
                 $(RTL_DIR)/voxel_bin_architecture/voxel_binning.sv \
                 $(RTL_DIR)/voxel_bin_architecture/systolic_array.sv \
                 $(RTL_DIR)/voxel_bin_architecture/gesture_classifier.sv \
                 $(RTL_DIR)/voxel_bin_architecture/weight_ram.sv \
                 $(RTL_DIR)/voxel_bin_architecture/voxel_bin_core.sv

# ── select top module ────────────────────────────────────────────────────────
ifeq ($(ARCH),gradient_map)
  ifeq ($(TARGET),raw)
    PROJ              = gradient_map_top
    PCF               = icebreaker.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(GRADIENT_MAP_CORE) \
                $(RTL_DIR)/gradient_map_architecture/gradient_map_top.sv
  else
    $(error Unknown TARGET=$(TARGET). Only TARGET=raw is supported)
  endif
else ifeq ($(ARCH),voxel_bin)
  ifeq ($(TARGET),raw)
    PROJ              = voxel_bin_top
    PCF               = icebreaker.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(VOXEL_BIN_CORE) \
                $(RTL_DIR)/voxel_bin_architecture/voxel_bin_top.sv
  else
    $(error Unknown TARGET=$(TARGET). Only TARGET=raw is supported)
  endif
else
  $(error Unknown ARCH=$(ARCH). Use ARCH=gradient_map or ARCH=voxel_bin)
endif

.PHONY: all clean prog report timing

all: $(PROJ).bit

$(PROJ).json: $(RTL_FILES)
	yosys -p "read_verilog -sv $(RTL_FILES); synth_ice40 -top $(PROJ) -json $@"

$(PROJ).asc: $(PROJ).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PCF) --asc $@ --freq 12 $(ALLOW_UNCONSTRAINED)

$(PROJ).bit: $(PROJ).asc
	icepack $< $@

prog: $(PROJ).bit
	iceprog $<

report: $(PROJ).json
	yosys -p "read_json $<; stat"

timing: $(PROJ).asc
	icetime -d $(DEVICE) -m -r $(PROJ).timing $<

clean:
	rm -f $(PROJ).json $(PROJ).asc $(PROJ).bit $(PROJ).timing
