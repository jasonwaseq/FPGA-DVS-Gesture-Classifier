# DVS Gesture Classifier for iCE40 FPGA

Real-time gesture recognition (UP, DOWN, LEFT, RIGHT) using Dynamic Vision Sensor events on iCE40 UP5K.

## Quick Start

```bash
# Clone and setup
git clone https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier.git
cd FPGA-DVS-Gesture-Classifier
sudo apt update
sudo apt install -y python3.12-venv libgl1 libglib2.0-0 libusb-1.0-0 iverilog
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
python setup.py

# Verify, synthesize, flash
.venv/bin/python setup.py test     # Run 18 cocotb tests
.venv/bin/python setup.py synth    # Synthesize to bitstream
.venv/bin/python setup.py flash  # Program FPGA via iceprog
```

## Devcontainer Notes

These commands are verified for the devcontainer in this repo. They ensure all
Python tools can run (tests, synth, flash, and utility scripts):

```bash
sudo apt update
sudo apt install -y python3-venv libgl1 libglib2.0-0 libusb-1.0-0 iverilog
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python setup.py
```

Run verification, synthesis, and flashing:

```bash
.venv/bin/python setup.py test
.venv/bin/python setup.py synth
.venv/bin/python setup.py flash
```

Run the Python tools with the venv interpreter so dependencies resolve:

```bash
.venv/bin/python tools/dvs_camera_emulator.py --simulate --preview
.venv/bin/python tools/dvs_event_player.py events.bin --preview
.venv/bin/python tools/fpga_gesture_validator.py --list-ports
.venv/bin/python tools/fpga_diagnostic.py
```

## Commands

| Command | Description |
|---------|-------------|
| `python setup.py` | Setup venv, install packages, detect OSS CAD Suite |
| `python setup.py test` | Run verification tests (iverilog + cocotb) |
| `python setup.py synth` | Synthesize (Yosys → nextpnr → icepack) |
| `python setup.py flash` | Program FPGA via iceprog (auto-detect) |
| `python setup.py clean` | Remove build artifacts |

## Architecture

### End‑to‑End Dataflow
The FPGA implements a streaming, single‑pass pipeline that converts raw DVS events into a discrete gesture label. Events arrive over UART as $(x,y,polarity)$ tuples with $x,y\in[0,319]$. Each event is processed in order and never revisited. The pipeline runs on a single clock domain and is designed to sustain continuous input with bounded latency.

**High‑level pipeline**: DVS Events (320×320) → Input FIFO → Spatial Compression (16×16) → Temporal Accumulation (early/late windows) → Motion Vector → Gesture Classification → UART Response

### Module‑Level Architecture

#### 1) Input Capture + FIFO
- **UART RX** deserializes the 5‑byte event packet and emits a valid event when the packet is complete and the polarity bit is valid.
- **Input FIFO** buffers events to decouple UART bursts from internal processing. The FIFO provides `full`/`empty` status for flow control and reporting in the status byte. The FIFO depth is sized to absorb short bursts without stalling UART reception.

**Key behavior**:
- Events are accepted as soon as a full packet is decoded.
- When FIFO is full, additional events are dropped and `full` is asserted to make overflow observable.

#### 2) Spatial Compression (320×320 → 16×16)
Each event’s $(x,y)$ coordinate is down‑sampled into a coarse 16×16 grid by discarding lower coordinate bits. Conceptually:
$$
x_c = \left\lfloor\frac{x}{20}\right\rfloor,\quad y_c = \left\lfloor\frac{y}{20}\right\rfloor
$$
This yields a fixed‑size spatial binning that reduces memory and arithmetic cost while preserving motion direction at gesture scale. The output is a compressed coordinate $(x_c,y_c)$ and its polarity.

#### 3) Temporal Accumulation (Dual Windows)
The system maintains two independent accumulators over a sliding event window:
- **Early window**: counts events from the first half of the window.
- **Late window**: counts events from the second half of the window.

Each window accumulates into a 16×16 grid of counters. When an event with compressed coordinate $(x_c,y_c)$ arrives, the corresponding counter in the active window is incremented. Windows advance based on event count, not wall‑clock time, which makes the algorithm robust to variable event rates.

**Windowing details**:
- A global event counter tracks the position inside the window.
- At the midpoint, the accumulator toggles from early to late.
- At the end of the window, both accumulators are frozen for analysis and then cleared for the next window.

#### 4) Motion Vector Computation
At window completion, the design computes the centroid of activity for both early and late windows:
$$
\vec{c}_{early} = \left(\frac{\sum x_c\,N_{early}(x_c,y_c)}{\sum N_{early}},\ \frac{\sum y_c\,N_{early}(x_c,y_c)}{\sum N_{early}}\right)
$$
$$
\vec{c}_{late} = \left(\frac{\sum x_c\,N_{late}(x_c,y_c)}{\sum N_{late}},\ \frac{\sum y_c\,N_{late}(x_c,y_c)}{\sum N_{late}}\right)
$$
The **motion vector** is then:
$$
\Delta\vec{c} = \vec{c}_{late} - \vec{c}_{early}
$$
In hardware, the centroid is computed via integer accumulations of $x_c$ and $y_c$ weighted by event counts, followed by integer division by total events. Division is performed once per window, so it does not limit per‑event throughput.

#### 5) Gesture Classification
The classifier uses the sign and magnitude of $\Delta x$ and $\Delta y$ to determine direction:
- If $|\Delta x| > |\Delta y|$ and $\Delta x > T_{motion}$ → **RIGHT**
- If $|\Delta x| > |\Delta y|$ and $\Delta x < -T_{motion}$ → **LEFT**
- If $|\Delta y| > |\Delta x|$ and $\Delta y > T_{motion}$ → **DOWN**
- If $|\Delta y| > |\Delta x|$ and $\Delta y < -T_{motion}$ → **UP**

Two thresholds gate the decision:
- **MIN_EVENT_THRESH**: minimum total events required in the window to consider a gesture valid.
- **MOTION_THRESH**: minimum centroid displacement to reject small jitter or stationary activity.

If thresholds are not met, no gesture is emitted and the system advances to the next window.

#### 6) UART Response + Status
When a gesture is detected, a 2‑byte response is generated containing the gesture ID and a compact confidence indicator derived from event count and centroid displacement. Status and configuration bytes are available via query commands, allowing the host to monitor FIFO health and active thresholds.

### Timing, Throughput, and Latency
- **Throughput**: One event accepted per cycle when FIFO is not empty and internal logic is ready; per‑event logic is constant time.
- **Latency**: Gesture output is produced once per window, so latency is bounded by the event count needed to fill the window plus a small fixed analysis time.
- **Determinism**: The event‑count windowing makes detection independent of absolute time, which is desirable for DVS data with variable activity.

### Hardware Footprint (iCE40 UP5K)
The design fits within the UP5K budget with headroom for UART, FIFO, and control logic.
- **Utilization**: 1163 LCs (22%), 2 BRAM (6%)
- **Max Frequency**: 42.76 MHz

### RTL Mapping (for reference)
The pipeline is implemented across these RTL modules:
- Input and UART: [rtl/uart_rx.sv](rtl/uart_rx.sv), [rtl/InputFIFO.sv](rtl/InputFIFO.sv)
- Spatial compression and event routing: [rtl/SpatialCompressor.sv](rtl/SpatialCompressor.sv)
- Temporal accumulation and window control: [rtl/TemporalAccumulator.sv](rtl/TemporalAccumulator.sv)
- Motion computation: [rtl/MotionComputer.sv](rtl/MotionComputer.sv)
- Gesture decision and output: [rtl/GestureClassifier.sv](rtl/GestureClassifier.sv), [rtl/OutputRegister.sv](rtl/OutputRegister.sv)
- Top‑level integration: [rtl/uart_gesture_top.sv](rtl/uart_gesture_top.sv)

## UART Protocol

**Configuration**: 115200 baud, 8N1

### Send Events to FPGA (5 bytes per event)

| Byte | Contents |
|------|----------|
| 0 | X_HI: `X[8]` (bit 0) |
| 1 | X_LO: `X[7:0]` |
| 2 | Y_HI: `Y[8]` (bit 0) |
| 3 | Y_LO: `Y[7:0]` |
| 4 | POL: Polarity (0=OFF, 1=ON) |

X, Y coordinates range from 0-319.

### Receive from FPGA

**Gesture Response** (2 bytes):
| Byte | Contents |
|------|----------|
| 0 | `0xA0 \| gesture` (0=UP, 1=DOWN, 2=LEFT, 3=RIGHT) |
| 1 | `confidence[3:0] \| event_count_hi[3:0]` |

**Status Response** (1 byte): `0xBx`
- Bits [4:2]: FSM state
- Bit 1: FIFO full
- Bit 0: FIFO empty

**Config Response** (2 bytes):
- Byte 0: MIN_EVENT_THRESH
- Byte 1: MOTION_THRESH

### Special Commands

| Send | Response | Description |
|------|----------|-------------|
| 0xFF | 0x55 | Echo test (verify UART) |
| 0xFE | 0xBx | Status query |
| 0xFD | 2 bytes | Config query |
| 0xFC | (none) | Soft reset |

## Tools

### DVS Camera Emulator

The `tools/dvs_camera_emulator.py` script converts your laptop webcam into a DVS-like sensor:

1. **Captures frames** from webcam at 30 FPS
2. **Computes log intensity** changes between frames
3. **Generates events** where intensity changed beyond threshold
4. **Transmits events** via UART to the FPGA

| Mode | Command | Description |
|------|---------|-------------|
| Preview | `--preview` | Show camera feed with DVS events overlay |
| Simulate | `--simulate` | Synthetic gestures (no camera needed) |
| FPGA | `--port /dev/ttyUSB1` | Send events to FPGA |
| Record | `--save events.bin` | Save events to file |

### FPGA Hardware Validator

The `tools/fpga_gesture_validator.py` script validates the FPGA implementation:

```bash
# List available serial ports
python fpga_gesture_validator.py --list-ports

# Basic connection test
python fpga_gesture_validator.py --port /dev/ttyUSB# --test echo

# Test all gestures
python fpga_gesture_validator.py --port /dev/ttyUSB# --test all

# Send specific gesture
python fpga_gesture_validator.py --port /dev/ttyUSB# --gesture right

# Interactive mode
python fpga_gesture_validator.py --port /dev/ttyUSB# --interactive

# Continuous monitoring
python fpga_gesture_validator.py --port /dev/ttyUSB# --continuous --duration 60
```
