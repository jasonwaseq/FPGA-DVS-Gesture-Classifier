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

# Verify, synthesize, flash (specify architecture: voxel_bin or gradient_map)
.venv/bin/python setup.py test gradient_map
.venv/bin/python setup.py synth gradient_map
.venv/bin/python setup.py flash gradient_map
```

## Flashing the FPGA

Synthesize and flash by architecture name. Both architectures are actively maintained.

### Prerequisites

- iCEBreaker FPGA board connected via USB
- OSS CAD Suite installed (detected automatically by `setup.py`)
- Python virtual environment activated (`.venv/bin/python`)

### Build and Flash Steps

**Option 1: Using setup.py (recommended)**

```bash
# Synthesize by architecture (voxel_bin or gradient_map)
python setup.py synth voxel_bin      # Build voxel_bin_top.bit
python setup.py synth gradient_map  # Build gradient_map_top.bit

# Flash by architecture
python setup.py flash voxel_bin      # Flash voxel_bin_top.bit
python setup.py flash gradient_map   # Flash gradient_map_top.bit
```

**Option 2: Using Makefile directly**

```bash
cd synth
# Voxel-bin architecture
make -f Makefile clean && make -f Makefile && make -f Makefile prog

# Gradient-map architecture
make -f Makefile.uart clean && make -f Makefile.uart && make -f Makefile.uart prog
```

### Configuration Parameters (Gradient-Map)

The gradient-map architecture uses these default parameters (configurable in `rtl/gradient_map_architecture/gradient_map_top.sv`):

- `MIN_MASS_THRESH = 2000`: Minimum accumulated mass threshold for gesture detection (tuned for webcam emulation)
- `FRAME_PERIOD_MS = 50`: Classification frame period in milliseconds
- `DECAY_SHIFT = 6`: Time-surface decay rate
- `BAUD_RATE = 115200`: UART communication speed

### Verifying the Flash

After flashing, you can verify the FPGA is running:

1. **Check LEDs**: `led_heartbeat` should blink at ~1.5 Hz
2. **Test with emulator**: Use the DVS camera emulator to send events and observe gesture detections
3. **Use validator tool**: Run `python tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --test all`

### Using with DVS Camera Emulator

For webcam-based testing (Windows PowerShell):

```powershell
cd C:\Users\jason\Documents\FPGA-DVS-Gesture-Classifier\tools
python dvs_camera_emulator.py --preview --max-events 5000 --port COM3 --no-noise --contrast 0.25
```

The FPGA will output ASCII gesture classifications (`UP\r\n`, `DOWN\r\n`, etc.) which are displayed in the emulator preview window.

## Project layout

- Common RTL and wrappers: [rtl](rtl)
- Voxel-Bin architecture RTL: [rtl/voxel_bin_architecture](rtl/voxel_bin_architecture)
- Gradient-Map architecture RTL: [rtl/gradient_map_architecture](rtl/gradient_map_architecture)
- Tests: [tb](tb)
  - Voxel-bin cocotb tests: [tb/voxel_bin_architecture](tb/voxel_bin_architecture) (main: `test_voxel_bin.py`)
  - Gradient-map cocotb tests: [tb/gradient_map_architecture](tb/gradient_map_architecture) (main: `test_gradient_map.py`)
- Scripts and tools: [tools](tools)
- Generated build outputs: [synth](synth)

## Architecture overview (current RTL)

The current RTL is a streaming, single-pass pipeline that converts EVT 2.0 DVS events into a gesture label. Events arrive as 32-bit EVT2 words (or via optional 5-byte UART mock input) and are processed in order on a single clock domain.

**High-level pipeline**: EVT2 words -> FIFO -> EVT2 decoder -> Time-surface memory (decay-on-read) -> Moment scan -> Direction classifier -> UART debug output

### Module map (current RTL)

- Top-level integration and control: [rtl/gesture_top.sv](rtl/gesture_top.sv)
- UART validation wrapper: [rtl/gesture_uart_top.sv](rtl/gesture_uart_top.sv)
- EVT2 decoder: [rtl/evt2_decoder.sv](rtl/evt2_decoder.sv)
- Input FIFO: [rtl/input_fifo.sv](rtl/input_fifo.sv)
- Time surface memory: [rtl/time_surface_memory.sv](rtl/time_surface_memory.sv)
- Moment scan + classifier: [rtl/feature_extractor.sv](rtl/feature_extractor.sv)
- UART debug output: [rtl/uart_debug.sv](rtl/uart_debug.sv)
- UART receiver: [rtl/uart_rx.sv](rtl/uart_rx.sv)
- UART transmitter: [rtl/uart_tx.sv](rtl/uart_tx.sv)

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

## Gradient-Map architecture

The Gradient-Map architecture is a streaming, single-pass pipeline that converts EVT 2.0 DVS events into a gesture label using a time-surface plus spatio-temporal classifier. Events arrive as 32-bit EVT2 words (or via optional 5-byte UART mock input) and are processed in order on a single clock domain.

**High-level pipeline**: EVT2 words -> FIFO -> EVT2 decoder -> Time-surface encoder (exponential decay) -> Spatio-temporal classifier (gradient-map / systolic array) -> UART debug output

### Module map (Gradient-Map architecture)

- Top-level integration and control: `rtl/gradient_map_architecture/gesture_top.sv`
- UART validation wrapper: `rtl/gradient_map_architecture/gradient_map_top.sv`
- EVT2 decoder: `rtl/gradient_map_architecture/evt2_decoder.sv`
- Input FIFO: `rtl/gradient_map_architecture/input_fifo.sv`
- Time-surface memory core: `rtl/gradient_map_architecture/time_surface_memory.sv`
- Time-surface encoder: `rtl/gradient_map_architecture/time_surface_encoder.sv`
- Gradient-map classifier core (flatten + systolic array + weights): files under `rtl/gradient_map_architecture`
- UART debug output: `rtl/uart_debug.sv`
- UART receiver: `rtl/uart_rx.sv`
- UART transmitter: `rtl/uart_tx.sv`

### How to test, synthesize, and flash (Gradient-Map)

```bash
# Run main cocotb tests
python setup.py test gradient_map
# Or from tb folder:
cd tb/gradient_map_architecture && make -f Makefile.gesture test

# Synthesize and flash
python setup.py synth gradient_map   # builds gradient_map_top.bit
python setup.py flash gradient_map  # flashes gradient_map_top.bit
```

## Voxel-Bin architecture

The voxel-bin architecture uses a dual-accumulator sliding-window design with spatial compression (320×320 → 16×16) and temporal binning. It is actively maintained alongside the gradient-map architecture.

**Key modules**: `SpatialCompressor`, `TemporalAccumulator`, `MotionComputer`, `GestureClassifier`

### How to test, synthesize, and flash (Voxel-Bin)

```bash
# Run main cocotb tests
python setup.py test voxel_bin
# Or from tb folder:
cd tb/voxel_bin_architecture && make test

# Synthesize and flash
python setup.py synth voxel_bin   # builds voxel_bin_top.bit
python setup.py flash voxel_bin  # flashes voxel_bin_top.bit
```

**Note**: The voxel-bin architecture uses binary gesture responses (not ASCII) and supports echo/status/config UART commands. The `fpga_gesture_validator.py` tool is designed for this architecture.

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

### Receive from FPGA (current architecture - `gradient_map_top`)

**Gesture response** (ASCII): `uart_debug` sends the gesture name followed by `\r\n`:

| Gesture | ASCII output |
|---------|-------------|
| UP | `UP\r\n` |
| DOWN | `DOWN\r\n` |
| LEFT | `LEFT\r\n` |
| RIGHT | `RIGHT\r\n` |

### Receive from FPGA (`voxel_bin_top`)

**Gesture response** (2 bytes):

| Byte | Contents |
|------|----------|
| 0 | `0xA0 \| gesture` (0=UP, 1=DOWN, 2=LEFT, 3=RIGHT) |
| 1 | `confidence[3:0] \| event_count_hi[3:0]` |

**Status response** (1 byte): `0xBx`

- Bits [4:2]: FSM state
- Bit 1: FIFO full
- Bit 0: FIFO empty

### Special commands (voxel_bin architecture only)

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
| `python setup.py test voxel_bin` | Run voxel-bin cocotb tests (`test_voxel_bin.py`) |
| `python setup.py test gradient_map` | Run gradient-map cocotb tests (`test_gradient_map.py`) |
| `python setup.py synth voxel_bin` | Synthesize voxel-bin design (`voxel_bin_top.bit`) |
| `python setup.py synth gradient_map` | Synthesize gradient-map design (`gradient_map_top.bit`) |
| `python setup.py flash voxel_bin` | Flash voxel-bin bitstream |
| `python setup.py flash gradient_map` | Flash gradient-map bitstream |
| `python setup.py clean` | Remove build artifacts |

## Hardware validation pipeline (GenX320 + STM32 + iCEBreaker)

End-to-end hardware validation of gesture classification using a live DVS sensor.

**Setup**: Prophesee GenX320 sensor -> STM32 board -> USB CDC -> Laptop -> UART -> iCEBreaker FPGA

### Step 1: Build and flash the gradient-map bitstream

The `gradient_map_top` wrapper enables UART event input (`UART_RX_ENABLE=1`), ties off the unused parallel bus, and provides an internal power-on reset.

```bash
python setup.py synth gradient_map
python setup.py flash gradient_map
```

### Step 2: Capture a raw EVT stream from the GenX320

Connect the STM32 board (running the GenX320 firmware) to a USB port. Identify the serial port (e.g. `COM5` on Windows, `/dev/ttyACM0` on Linux).

```bash
python tools/capture_evt_stream.py COM5 gesture_trial.bin --duration 10
```

This saves the raw EVT2 byte stream and produces an aligned `.bin` file.

### Step 3a: Replay the captured file to the FPGA

Connect the iCEBreaker FPGA to a second USB port (e.g. `COM3` / `/dev/ttyUSB1`). Replay the captured events:

```bash
python tools/replay_evt_to_fpga.py --file aligned.bin --fpga COM3
```

The script decodes the raw EVT2 words, converts CD events to 5-byte UART packets, sends them to the FPGA, and displays gesture classifications in real-time.

### Step 3b: Live relay (STM32 -> Laptop -> FPGA)

For real-time classification, relay events directly from the sensor to the FPGA:

```bash
python tools/replay_evt_to_fpga.py --dvs COM5 --fpga COM3
```

Optionally save the raw capture while relaying:

```bash
python tools/replay_evt_to_fpga.py --dvs COM5 --fpga COM3 --save capture.bin --duration 30
```

### EVT2 bit layout

The default bit layout matches the standard EVT2 format (`x[21:11]`, `y[10:0]`). If the GenX320 firmware uses a different packing, override with:

```bash
python tools/replay_evt_to_fpga.py --file aligned.bin --fpga COM3 \
    --x-shift 17 --y-shift 6 --swap-xy
```

### What the FPGA outputs

The `uart_debug` module sends ASCII gesture labels over UART TX whenever the classifier detects a gesture: `UP\r\n`, `DOWN\r\n`, `LEFT\r\n`, or `RIGHT\r\n`. The replay tool parses these and reports a summary with per-gesture counts.

Direction LEDs on the iCEBreaker also flash for ~500ms on each detection:
- `led_up` (pin 26), `led_down` (pin 27), `led_left` (pin 25), `led_right` (pin 23)
- `led_heartbeat` (pin 39): ~1.5 Hz blink confirms the FPGA is alive
- `led_activity` (pin 40): pulses when events are being received

### Bandwidth note

At 115200 baud, the UART can deliver ~2300 events/sec (5 bytes per event). The GenX320 may produce far more events, so the relay throttles naturally. Gesture classification still works at reduced rates because the time-surface accumulates events over the `FRAME_PERIOD_MS` window (default 10ms).

## Tools

### DVS camera emulator

```bash
.venv/bin/python tools/dvs_camera_emulator.py --simulate --preview
.venv/bin/python tools/dvs_camera_emulator.py --port port#
.venv/bin/python tools/dvs_event_player.py events.bin --preview
```

### FPGA hardware validator (synthetic gestures)

```bash
.venv/bin/python tools/fpga_gesture_validator.py --list-ports
.venv/bin/python tools/fpga_gesture_validator.py --port port# --test all
.venv/bin/python tools/fpga_gesture_validator.py --port port# --interactive
```

### Raw event capture and alignment

```bash
.venv/bin/python tools/capture_evt_stream.py port# trial_run.bin --duration #
```

### EVT2 replay / live relay to FPGA

```bash
.venv/bin/python tools/replay_evt_to_fpga.py --file aligned.bin --fpga port#
.venv/bin/python tools/replay_evt_to_fpga.py --dvs port# --fpga port#
.venv/bin/python tools/replay_evt_to_fpga.py --list-ports
```
