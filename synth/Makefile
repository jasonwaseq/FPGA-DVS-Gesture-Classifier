PCF    = icebreaker.pcf
DEVICE = up5k
PACKAGE = sg48

RTL_DIR = ../rtl

# Select architecture and input variant:
#   ARCH=gradient_map  TARGET=raw        -> gradient_map_raw_top (UART sensor input)
#   ARCH=gradient_map  TARGET=processed  -> gradient_map_processed_top (EVT2 word bus)
#   ARCH=voxel_bin     TARGET=raw        -> voxel_bin_raw_top (UART sensor input)
#   ARCH=voxel_bin     TARGET=processed  -> voxel_bin_processed_top (decoded event bus)
ARCH   ?= gradient_map
TARGET ?= raw

# ── gradient_map sources ────────────────────────────────────────────────────
# Raw pipeline: UART RX → 5-byte assembler → FIFO → EVT2 decoder → time-surface → classifier
GRADIENT_MAP_CORE = $(RTL_DIR)/uart_tx.sv \
                    $(RTL_DIR)/uart_rx.sv \
                    $(RTL_DIR)/uart_debug.sv \
                    $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                    $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                    $(RTL_DIR)/gradient_map_architecture/time_surface_memory.sv \
                    $(RTL_DIR)/gradient_map_architecture/time_surface_encoder.sv \
                    $(RTL_DIR)/gradient_map_architecture/weight_rom.sv \
                    $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                    $(RTL_DIR)/gradient_map_architecture/spatio_temporal_classifier.sv

# Lean pipeline (processed top bypasses FIFO/decoder/gesture_top)
GRADIENT_MAP_PROCESSED_CORE = $(RTL_DIR)/uart_tx.sv \
                               $(RTL_DIR)/uart_rx.sv \
                               $(RTL_DIR)/uart_debug.sv \
                               $(RTL_DIR)/gradient_map_architecture/time_surface_memory.sv \
                               $(RTL_DIR)/gradient_map_architecture/time_surface_encoder.sv \
                               $(RTL_DIR)/gradient_map_architecture/weight_rom.sv \
                               $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                               $(RTL_DIR)/gradient_map_architecture/spatio_temporal_classifier.sv

# ── voxel_bin sources ───────────────────────────────────────────────────────
VOXEL_BIN_CORE = $(RTL_DIR)/uart_tx.sv \
                 $(RTL_DIR)/uart_rx.sv \
                 $(RTL_DIR)/voxel_bin_architecture/dvs_gesture_accel.sv \
                 $(RTL_DIR)/voxel_bin_architecture/TemporalAccumulator.sv \
                 $(RTL_DIR)/voxel_bin_architecture/SpatialCompressor.sv \
                 $(RTL_DIR)/voxel_bin_architecture/OutputRegister.sv \
                 $(RTL_DIR)/voxel_bin_architecture/MotionComputer.sv \
                 $(RTL_DIR)/voxel_bin_architecture/InputFIFO.sv \
                 $(RTL_DIR)/voxel_bin_architecture/TimeSurfaceBinning.sv \
                 $(RTL_DIR)/voxel_bin_architecture/WeightROM.sv \
                 $(RTL_DIR)/voxel_bin_architecture/SystolicMatrixMultiply.sv

# ── select top module ────────────────────────────────────────────────────────
ifeq ($(ARCH),gradient_map)
  ifeq ($(TARGET),raw)
    PROJ              = gradient_map_raw_top
    PCF               = icebreaker.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(GRADIENT_MAP_CORE) \
                $(RTL_DIR)/gradient_map_architecture/gradient_map_raw_top.sv
  else ifeq ($(TARGET),processed)
    PROJ              = gradient_map_processed_top
    PCF               = icebreaker_processed_gradient_map.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(GRADIENT_MAP_PROCESSED_CORE) \
                $(RTL_DIR)/gradient_map_architecture/gradient_map_processed_top.sv
  else
    $(error Unknown TARGET=$(TARGET). Use TARGET=raw or TARGET=processed)
  endif
else ifeq ($(ARCH),voxel_bin)
  ifeq ($(TARGET),raw)
    PROJ              = voxel_bin_raw_top
    PCF               = icebreaker.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(VOXEL_BIN_CORE) \
                $(RTL_DIR)/voxel_bin_architecture/voxel_bin_raw_top.sv
  else ifeq ($(TARGET),processed)
    PROJ              = voxel_bin_processed_top
    PCF               = icebreaker_processed_voxel_bin.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(VOXEL_BIN_CORE) \
                $(RTL_DIR)/voxel_bin_architecture/voxel_bin_processed_top.sv
  else
    $(error Unknown TARGET=$(TARGET). Use TARGET=raw or TARGET=processed)
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
