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

## Architecture overview

The FPGA implements a streaming, single-pass pipeline that converts raw DVS events into a gesture label. Events arrive over UART as X, Y, and polarity and are processed in order without revisiting past events. The design runs in one clock domain and is built to keep up with continuous input.

**High-level pipeline**: DVS events (320x320) -> Input FIFO -> Spatial compression (16x16) -> Time-surface accumulation (early/late windows) -> Motion vector -> Gesture classifier -> UART response

### Input capture + FIFO

- UART RX deserializes the 5-byte EVT2-like packet and emits a valid event when complete.
- The FIFO absorbs UART bursts and provides `full`/`empty` status for flow control and diagnostics.

### Spatial compression (320x320 -> 16x16)

Each event is binned into a 16x16 grid by dropping lower coordinate bits. This reduces memory and arithmetic while preserving gross motion direction.

### Time-surface accumulation (dual windows)

Two heatmaps are maintained over a fixed event-count window:

- Early window accumulates activity at the start of the window.
- Late window accumulates activity at the end of the window.

The window advances on event count rather than wall-clock time so performance remains stable across variable event rates.

### Motion vector + gesture classification

At window end, each heatmap is summarized into a centroid. The motion vector is the delta between early and late centroids. The classifier selects the dominant axis (horizontal vs vertical) and uses thresholds to suppress low-activity or low-motion windows.

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
.venv/bin/python tools/dvs_camera_emulator.py --port /dev/ttyUSB1
.venv/bin/python tools/dvs_event_player.py events.bin --preview
```

### FPGA hardware validator

```bash
.venv/bin/python tools/fpga_gesture_validator.py --list-ports
.venv/bin/python tools/fpga_gesture_validator.py --port /dev/ttyUSB# --test all
.venv/bin/python tools/fpga_gesture_validator.py --port /dev/ttyUSB# --interactive
```

### Raw event capture and alignment

```bash
.venv/bin/python tools/capture_evt_stream.py COM3 trial_run.bin --duration 30
.venv/bin/python tools/capture_evt_stream.py /dev/ttyACM0 trial_run.bin --duration 30
```
