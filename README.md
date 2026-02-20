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

Synthesize and flash by architecture name. 

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

### iCE40 UP5K resource fit

Both architectures are tuned to fit the Lattice iCE40 UP5K (≈5280 LUT4s, 30×4 Kbit BRAM):

- **Gradient-map**: 16×16 grid (256 cells), 128-entry input FIFO, time-surface and weight ROMs sized for 256 cells; `flatten_buffer` removed from build (classifier streams directly from BRAM).
- **Voxel-bin**: 5 temporal bins (was 8), 16×16 grid, 1280-cell feature vector and 4×1280 weight ROMs unchanged; bin RAM reduced to 5×256×8-bit.

### Configuration Parameters (Gradient-Map)

The gradient-map architecture uses these default parameters (configurable in `rtl/gradient_map_architecture/gradient_map_top.sv`):

- `MIN_MASS_THRESH = 2000`: Minimum accumulated mass threshold for gesture detection (tuned for webcam emulation)
- `FRAME_PERIOD_MS = 50`: Classification frame period in milliseconds
- `DECAY_SHIFT = 6`: Time-surface decay rate
- `BAUD_RATE = 115200`: UART communication speed
- Grid: **16×16** (256 cells) for UP5K fit; decoder downsamples 320×320 to 16×16

Cocotb tests under `tb/gradient_map_architecture/` that assume 32×32 (e.g. `GRID_SIZE=32`, `NUM_CELLS=1024`) may need to be updated to 16×16 / 256 to match the current RTL.

### Verifying the Flash

After flashing, you can verify the FPGA is running:

1. **Check LEDs**: `led_heartbeat` should blink at ~1.5 Hz
2. **Test with emulator**: Use the DVS camera emulator to send events and observe gesture detections
3. **Use validator tool**: Run `python tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --arch gradient_map --test all` (or `--arch voxel_bin` for voxel-bin).

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

## Architecture overview

The two provided architectures are both streaming, single-pass DVS pipelines that run on a single clock domain, but they implement **different feature representations and classifiers**:

- **Gradient-Map**: EVT2 → time-surface (exponential decay) → learned linear classifier (systolic MAC array over the 2D surface).
- **Voxel-Bin**: EVT2 → spatial compression → 3D histogram (time × x × y) → learned linear classifier (systolic MAC over flattened bins).

Neither architecture computes an explicit centroid; both use **learned weights over feature maps** rather than hand-coded centroid logic.

## Gradient-Map architecture

The Gradient-Map architecture is a streaming, frame-based pipeline that converts EVT 2.0 DVS events into a gesture label using a decaying time-surface plus a systolic-array classifier. Events arrive as 32-bit EVT2 words (or via optional 5-byte UART mock input) and are processed in order on a single clock domain. Default grid is **16×16** (256 cells) for iCE40 UP5K fit.

**High-level pipeline**: EVT2 words → FIFO → EVT2 decoder → Time-surface encoder (exponential decay) → Spatio-temporal classifier (systolic MAC + argmax) → UART debug output

### Module map (Gradient-Map architecture)

- Top-level integration and control: `rtl/gradient_map_architecture/gesture_top.sv`
- UART validation wrapper: `rtl/gradient_map_architecture/gradient_map_top.sv`
- EVT2 decoder + spatial downsample: `rtl/gradient_map_architecture/evt2_decoder.sv`
- Input FIFO: `rtl/gradient_map_architecture/input_fifo.sv`
- Time-surface memory core: `rtl/gradient_map_architecture/time_surface_memory.sv`
- Time-surface encoder (exponential decay front-end): `rtl/gradient_map_architecture/time_surface_encoder.sv`
- Spatio-temporal classifier (frame timer + scan + systolic array + weights):
  - `rtl/gradient_map_architecture/spatio_temporal_classifier.sv`
  - `rtl/gradient_map_architecture/systolic_array.sv`
  - `rtl/gradient_map_architecture/weight_rom.sv`
- UART debug output: `rtl/uart_debug.sv`
- UART receiver: `rtl/uart_rx.sv`
- UART transmitter: `rtl/uart_tx.sv`

### Input capture + FIFO (Gradient-Map)

- External EVT2 words are accepted when `evt_ready` is high and written into the architecture-local `input_fifo` (128 x 32-bit BRAM in the current config).
- Optional UART event input can be enabled (parameter `UART_RX_ENABLE`) to synthesize EVT2 CD events from 5-byte packets for testing.

### EVT2 decode + spatial downsample (Gradient-Map)

- `evt2_decoder` parses EVT2 words, reconstructs a 16-bit timestamp, and emits CD events.
- X and Y are downsampled from 320×320 into a 16×16 grid using bit slicing with clamping (see `GRID_BITS` parameter in the gradient-map decoder).

### Time-surface encoder (exponential decay)

- `time_surface_memory` stores the last-event timestamp for each 16×16 cell in an inlined dual-port BRAM.
- `time_surface_encoder` wraps `time_surface_memory` and converts timestamps into an exponentially decaying value:
  - For each cell, it computes Δt = `t_now - t_last_event`, then `decay_steps = Δt >> DECAY_SHIFT`.
  - The surface value is `MAX_VALUE >> decay_steps`, i.e. exact binary half-lives (255, 127, 63, …) until it decays to zero.
- Reads are pipelined so that the classifier can stream one decayed cell per cycle.

### Spatio-temporal classification (systolic array)

- `spatio_temporal_classifier` runs on a fixed frame period (`FRAME_PERIOD_MS`) driven by a frame counter.
- Each frame, it linearly scans all `GRID_SIZE × GRID_SIZE` cells, reading decayed values from the time-surface into:
  - An **energy accumulator** (`debug_m00`) used to gate low-activity frames via `MIN_MASS_THRESH`.
  - A **feature stream**: one cell per cycle into `systolic_array`.
- `systolic_array` and four parallel `weight_rom` instances implement a 4-way learned linear classifier:
  - For each cell index `i`, all 4 class weights `w_k[i]` are fetched in parallel and multiplied by the feature value.
  - Accumulators integrate these products over all cells to produce 4 class scores.
  - An argmax stage selects the best class and exposes scores for debug.
- The final output logic in `spatio_temporal_classifier`:
  - Asserts `gesture_valid` only when the frame energy exceeds `MIN_MASS_THRESH`.
  - Outputs `gesture_class` from the systolic argmax.
  - Derives a coarse `gesture_confidence` from the accumulated energy, **not** from centroid geometry.

### Output and indicators (Gradient-Map)

- `uart_debug` outputs gesture class and confidence as ASCII strings (`UP\r\n`, `DOWN\r\n`, etc.).
- `gesture_top.sv` in the gradient-map folder also drives LEDs for heartbeat, activity, and detected direction.

### How to test, synthesize, and flash (Gradient-Map)

```bash
# Run main cocotb tests
python setup.py test gradient_map
# Or from tb folder:
cd tb/gradient_map_architecture && make test

# Synthesize and flash
python setup.py synth gradient_map   # builds gradient_map_top.bit
python setup.py flash gradient_map   # flashes gradient_map_top.bit
```

## Voxel-Bin architecture

The voxel-bin architecture is an event-driven pipeline that compresses the 320×320 sensor into a 16×16 grid and accumulates activity into a **3D histogram of recent events** (time bins × x × y). A readout FSM periodically flattens the most recent bins into a feature vector and feeds a systolic classifier. Default is **5 temporal bins** (read out all 5) for iCE40 UP5K fit.

**High-level pipeline**: DVS events → InputFIFO → SpatialCompressor (320×320 → 16×16) → TimeSurfaceBinning (NUM_BINS × GRID × GRID counters) → SystolicMatrixMultiply + WeightROM (learned classifier) → OutputRegister (persistence + confidence) → UART

**Key modules** (see `rtl/voxel_bin_architecture`):

- `dvs_gesture_accel.sv`: top-level accelerator (wraps the full voxel-bin pipeline).
- `InputFIFO.sv`: shallow event FIFO with ready/valid handshaking.
- `SpatialCompressor.sv`: maps raw sensor coordinates to the GRID_SIZE×GRID_SIZE grid.
- `TimeSurfaceBinning.sv`: ring buffer of 2D histograms over time; rotates bins based on a cycle counter and flattens the latest `READOUT_BINS` bins into a 1-D feature stream (`readout_data`, `readout_valid`).
- `SystolicMatrixMultiply.sv`: systolic MAC array that multiplies the flattened bin vector by 4 learned weight vectors (one per gesture class) and produces 4 scores plus `best_class`.
- `WeightROM.sv`: per-class weight ROMs addressed by feature index, providing weights to the systolic array.
- `OutputRegister.sv`: persistence filter and confidence mapping; converts classifier outputs into a stable gesture code plus 4-bit confidence, with simple activity/motion gating.

In the voxel-bin path, motion is inferred from **learned weights over the spatio-temporal histogram**, not from explicit centroid or geometric computations. Legacy centroid-based modules (`TemporalAccumulator`, `MotionComputer`, `GestureClassifier`) remain in the tree for reference and experiments but are not used in the current `dvs_gesture_accel` top.

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

**Note**: The voxel-bin architecture uses binary gesture responses and supports echo/status/config UART commands. The gradient-map architecture outputs ASCII only (`UP\r\n`, etc.) and has no echo/status/config. Use the validator with `--arch gradient_map` for gradient-map or `--arch voxel_bin` (default) for voxel-bin:  
`python tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --arch gradient_map --test all`

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
.venv/bin/python tools/fpga_gesture_validator.py --port port# --arch gradient_map --test all
.venv/bin/python tools/fpga_gesture_validator.py --port port# --arch voxel_bin --test all
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
