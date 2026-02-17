# DVS Gesture Classifier for iCE40 FPGA

Real-time gesture recognition (UP, DOWN, LEFT, RIGHT) from DVS events on iCE40 UP5K.

## Quick start (Linux/WSL)

```bash
git clone https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier.git
cd FPGA-DVS-Gesture-Classifier
sudo apt update
sudo apt install -y python3-venv libgl1 libglib2.0-0 libusb-1.0-0 iverilog
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python setup.py

# Verify, synthesize, flash
.venv/bin/python setup.py test
.venv/bin/python setup.py synth
.venv/bin/python setup.py flash
```

## Project layout

- Current RTL: [rtl](rtl)
- Legacy RTL: [rtl/old_architecture](rtl/old_architecture)
- Tests: [tb](tb)
- Scripts and tools: [tools](tools)
- Generated build outputs: [synth](synth)

## Architecture overview (current RTL)

The current RTL is a streaming, single-pass pipeline that converts EVT 2.0 DVS events into a gesture label. Events arrive as 32-bit EVT2 words (or via optional 5-byte UART mock input) and are processed in order on a single clock domain.

**High-level pipeline**: EVT2 words -> FIFO -> EVT2 decoder -> Time-surface memory (decay-on-read) -> Moment scan -> Direction classifier -> UART debug output

### Module map (current RTL)

- Top-level integration and control: [rtl/gesture_top.sv](rtl/gesture_top.sv)
- EVT2 decoder: [rtl/evt2_decoder.sv](rtl/evt2_decoder.sv)
- Input FIFO: [rtl/input_fifo.sv](rtl/input_fifo.sv)
- Time surface memory: [rtl/time_surface_memory.sv](rtl/time_surface_memory.sv)
- Moment scan + classifier: [rtl/feature_extractor.sv](rtl/feature_extractor.sv)
- UART debug output: [rtl/uart_debug.sv](rtl/uart_debug.sv)
- Optional UART event input: [rtl/uart_rx.sv](rtl/uart_rx.sv)

### Input capture + FIFO

- External EVT2 words are accepted when `evt_ready` is high and written into `input_fifo` (256 x 32-bit BRAM).
- Optional UART event input can be enabled (parameter `UART_RX_ENABLE`) to synthesize EVT2 CD events from 5-byte packets for testing.

### EVT2 decode + spatial downsample

- `evt2_decoder` parses EVT2 words, reconstructs a 16-bit timestamp, and emits CD events.
- X and Y are downsampled from 320x320 into a 16x16 grid using bit slicing (`x_raw[8:5]`, `y_raw[8:5]`) with clamping.

### Time-surface memory (decay-on-read)

- `time_surface_memory` stores the last-event timestamp for each 16x16 cell in BRAM.
- A global 16-bit timestamp counter provides `t_now` for decay.
- On read, the decayed value is computed lazily: `value = MAX - (t_now - t_last) >> DECAY_SHIFT`, clamped to [0, MAX]. This produces a time-surface without explicit sweeping.

### Moment scan + classification

- `feature_extractor` runs on a fixed frame period (`FRAME_PERIOD_MS`).
- It scans all 256 cells, accumulating spatial moments: `M00`, `M10`, `M01`.
- The centroid offset from the center (8,8) is derived as `M10 - M00*8` and `M01 - M00*8`.
- A simple directional classifier compares offsets to determine UP/DOWN/LEFT/RIGHT. `MIN_MASS_THRESH` gates low-activity frames.

### Output and indicators

- `uart_debug` outputs gesture class and confidence over UART.
- LEDs reflect heartbeat, activity, and detected direction in [rtl/gesture_top.sv](rtl/gesture_top.sv).

## Old architecture (legacy)

The previous pipeline is preserved in [rtl/old_architecture](rtl/old_architecture). It uses the earlier module split and naming scheme (for example `SpatialCompressor`, `TemporalAccumulator`, `MotionComputer`) and is kept for reference only. It is not maintained and may not match current tests or tools.

## UART protocol

**Configuration**: 115200 baud, 8N1

### Send events to FPGA (5 bytes per event)

| Byte | Contents |
|------|----------|
| 0 | X_HI: `X[8]` (bit 0) |
| 1 | X_LO: `X[7:0]` |
| 2 | Y_HI: `Y[8]` (bit 0) |
| 3 | Y_LO: `Y[7:0]` |
| 4 | POL: Polarity (0=OFF, 1=ON) |

X, Y coordinates range from 0-319.

### Receive from FPGA

**Gesture response** (2 bytes):

| Byte | Contents |
|------|----------|
| 0 | `0xA0 | gesture` (0=UP, 1=DOWN, 2=LEFT, 3=RIGHT) |
| 1 | `confidence[3:0] | event_count_hi[3:0]` |

**Status response** (1 byte): `0xBx`

- Bits [4:2]: FSM state
- Bit 1: FIFO full
- Bit 0: FIFO empty

**Config response** (2 bytes):

- Byte 0: MIN_EVENT_THRESH
- Byte 1: MOTION_THRESH

### Special commands

| Send | Response | Description |
|------|----------|-------------|
| 0xFF | 0x55 | Echo test (verify UART) |
| 0xFE | 0xBx | Status query |
| 0xFD | 2 bytes | Config query |
| 0xFC | (none) | Soft reset |

## Common commands

| Command | Description |
|---------|-------------|
| `python setup.py` | Setup venv, install packages, detect OSS CAD Suite |
| `python setup.py test` | Run verification tests (iverilog + cocotb) |
| `python setup.py synth` | Synthesize (Yosys -> nextpnr -> icepack) |
| `python setup.py flash` | Program FPGA via iceprog |
| `python setup.py clean` | Remove build artifacts |

## Tools

### DVS camera emulator

```bash
.venv/bin/python tools/dvs_camera_emulator.py --simulate --preview
.venv/bin/python tools/dvs_camera_emulator.py --port port#
.venv/bin/python tools/dvs_event_player.py events.bin --preview
```

### FPGA hardware validator

```bash
.venv/bin/python tools/fpga_gesture_validator.py --list-ports
.venv/bin/python tools/fpga_gesture_validator.py --port port# --test all
.venv/bin/python tools/fpga_gesture_validator.py --port port# --interactive
```

### Raw event capture and alignment

```bash
.venv/bin/python tools/capture_evt_stream.py port# trial_run.bin --duration #
```
