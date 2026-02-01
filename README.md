# DVS Gesture Accelerator - Asynchronous Spatiotemporal Motion-Energy Classifier

FPGA-based real-time gesture recognition accelerator for Dynamic Vision Sensor (DVS) events on iCE40 UP5K.

**One-Script Workflow**: Single `setup.py` handles environment setup, cocotb verification, synthesis, and FPGA programming across all platforms (Windows/Linux/macOS).

## Features

- **Asynchronous Event-Driven Processing**: Each DVS event processed individually as it arrives
- **Shallow FIFO Buffer**: 16-entry FIFO tolerates bursty activity while maintaining low latency
- **400× Spatial Compression**: 320×320 → 16×16 grid using shift-based arithmetic (no dividers)
- **Dual-Accumulator Sliding Window**: ~400ms observation window with early/late centroid tracking
- **Motion Vector Extraction**: Explicit geometric motion computation (Δx, Δy)
- **Threshold-Based Classification**: Cardinal gestures (UP, DOWN, LEFT, RIGHT) without multipliers
- **Activity Gate**: Minimum event threshold suppresses noise-driven false detections
- **Persistence Filter**: Requires consistent classification across consecutive windows
- **Ultra-Low Power**: Dynamic power consumed only in response to incoming events

## Quick Start

### Setup (All Platforms)

The project uses a unified Python setup script that works identically on Windows, Linux, and macOS. The script:
- Creates a Python virtual environment
- Installs dependencies (cocotb, pytest, numpy, opencv-python, pyserial)
- Detects or downloads OSS CAD Suite (Yosys, nextpnr, Icarus Verilog)
- Configures the complete toolchain automatically

```bash
# Clone the repo
git clone https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier.git
cd FPGA-DVS-Gesture-Classifier

# Run setup - automatically detects existing OSS CAD Suite or downloads it
python setup.py

# That's it! Now run tests or synthesis directly:
python setup.py test    # Run cocotb verification
python setup.py synth   # Synthesize for iCE40
python setup.py flash   # Program FPGA
```

**Note**: You don't need to manually activate the virtual environment or source OSS CAD Suite environment scripts - `setup.py` handles all environment configuration internally.

### Run Verification Tests (cocotb)

```bash
python setup.py test
```

### Synthesize for iCE40 UP5K

```bash
python setup.py synth
```

### Flash to FPGA

```bash
python setup.py flash
```

### All Commands

| Command | Description |
|---------|-------------|
| `python setup.py` | Initial setup - creates venv, installs packages, detects/downloads OSS CAD Suite |
| `python setup.py --skip-fpga` | Setup without downloading OSS CAD Suite (use existing installation) |
| `python setup.py test` | Run cocotb verification tests (compiles RTL with iverilog, runs testbench) |
| `python setup.py synth` | Full synthesis flow (Yosys → nextpnr → icepack) |
| `python setup.py flash` | Program bitstream to connected FPGA via iceprog |
| `python setup.py clean` | Remove build artifacts (sim_build, synthesis outputs) |

**Environment Handling**: All commands automatically configure the OSS CAD Suite environment (PATH, DLL paths, SSL certs, etc.) - no need to source environment scripts manually.

### Dev Container (Alternative)

For GitHub Codespaces or VS Code Dev Containers, open the project and click "Reopen in Container" when prompted.

## Architecture Overview

This is a **real-time gesture recognition system** that processes Dynamic Vision Sensor (DVS) events on an **iCE40 UP5K FPGA**. The system captures motion events from a camera (emulated via webcam), transmits them over UART, and classifies gestures (UP, DOWN, LEFT, RIGHT) using an asynchronous spatiotemporal motion-energy algorithm.

### System Block Diagram

```
┌─────────────────────┐         UART           ┌──────────────────────────────────────┐
│   Laptop/PC         │     (115200 8N1)       │         iCE40 UP5K FPGA              │
│                     │                        │                                      │
│ ┌─────────────────┐ │    5-byte packets      │  ┌──────────┐    ┌────────────────┐ │
│ │ DVS Camera      │ │ ───────────────────►   │  │ uart_rx  │───►│ Packet State   │ │
│ │ Emulator        │ │   [X_HI,X_LO,Y_HI,     │  │ (8N1)    │    │ Machine        │ │
│ │ (Python/OpenCV) │ │    Y_LO,POL]           │  └──────────┘    └───────┬────────┘ │
│ └─────────────────┘ │                        │                          │          │
│         ▲           │                        │                          ▼          │
│         │           │                        │  ┌────────────────────────────────┐ │
│ ┌───────┴─────────┐ │   2-byte gesture       │  │     dvs_gesture_accel          │ │
│ │ Webcam          │ │ ◄───────────────────   │  │  ┌────────────┐ ┌───────────┐  │ │
│ │                 │ │   [0xA0|gest, conf]    │  │  │ 16-entry   │ │ Spatial   │  │ │
│ └─────────────────┘ │                        │  │  │ FIFO       │ │ Compress  │  │ │
│                     │                        │  │  └──────┬─────┘ └─────┬─────┘  │ │
│ ┌─────────────────┐ │                        │  │         │             │        │ │
│ │ FPGA Validator  │ │                        │  │         ▼             ▼        │ │
│ │ (Python)        │ │                        │  │  ┌─────────────────────────┐   │ │
│ └─────────────────┘ │                        │  │  │ Dual Accumulators       │   │ │
│                     │                        │  │  │ (Early/Late windows)    │   │ │
│                     │                        │  │  │ sum_x, sum_y, count     │   │ │
│                     │                        │  │  └───────────┬─────────────┘   │ │
│                     │                        │  │              │                  │ │
│                     │                        │  │              ▼                  │ │
│                     │                        │  │  ┌─────────────────────────┐   │ │
│                     │                        │  │  │ Motion Vector           │   │ │
│                     │                        │  │  │ Δx = late_x - early_x   │   │ │
│                     │                        │  │  │ Δy = late_y - early_y   │   │ │
│                     │                        │  │  └───────────┬─────────────┘   │ │
│                     │                        │  │              │                  │ │
│                     │                        │  │              ▼                  │ │
│                     │                        │  │  ┌─────────────────────────┐   │ │
│                     │                        │  │  │ Threshold Classifier    │   │ │
│                     │                        │  │  │ + Activity Gate         │   │ │
│                     │                        │  │  │ + Persistence Filter    │   │ │
│                     │                        │  │  └───────────┬─────────────┘   │ │
│                     │                        │  └──────────────┼────────────────┘ │
│                     │                        │                 ▼                  │
│                     │                        │  ┌──────────┐  ┌────────┐          │
│                     │                        │  │ uart_tx  │◄─┤ TX FSM │          │
│                     │                        │  └──────────┘  └────────┘          │
│                     │                        │                                      │
│                     │                        │  LEDs: led_heartbeat (~3Hz)         │
│                     │                        │        led_gesture_valid (pulse)    │
│                     │                        │        led_activity (event blink)   │
└─────────────────────┘                        └──────────────────────────────────────┘
```

### Hardware Modules

| Module | File | Description |
|--------|------|-------------|
| `uart_gesture_top` | `rtl/uart_gesture_top.sv` | Top-level wrapper, UART protocol, packet parsing |
| `dvs_gesture_accel` | `rtl/dvs_gesture_accel.sv` | Asynchronous spatiotemporal motion-energy classifier |
| `uart_rx` | `rtl/uart_rx.sv` | UART receiver (8N1, 115200 baud) |
| `uart_tx` | `rtl/uart_tx.sv` | UART transmitter (8N1, 115200 baud) |

### Processing Pipeline

```
DVS Events (320×320) → FIFO Buffer → Spatial Compression (16×16)
                    → Dual Accumulators (Early/Late) → Motion Vector Extraction
                    → Threshold Classification → Persistence Filter → Gesture Output
```

#### 1. Asynchronous FIFO (16 entries)

A shallow FIFO buffers incoming events to tolerate bursty DVS activity while maintaining continuous event-driven operation.

#### 2. Spatial Compression (320×320 → 16×16)

Events are mapped to a 16×16 grid using shift-based arithmetic (no dividers):
```
grid_x = (event_x × 13) >> 8   // Approximates x/20 (320/16 = 20)
grid_y = (event_y × 13) >> 8
```

This achieves 400× spatial dimensionality reduction while retaining coarse motion structure.

#### 3. Dual-Accumulator Sliding Window

Instead of storing per-pixel counters, each accumulator tracks motion statistics:
- `sum_x`, `sum_y` - Summed positions (signed, relative to center)
- `count` - Total event count
- Polarity-separated variants for advanced processing

The ~400ms observation window is split into:
- **Early half** (~200ms): First portion of events
- **Late half** (~200ms): Second portion of events

This enables direct centroid computation: `centroid = sum / count`

#### 4. Motion Vector Extraction

At window completion, the motion vector is computed:
```
Δx = late_sum_x - early_sum_x
Δy = late_sum_y - early_sum_y
```

This directly encodes dominant direction and magnitude without template matching.

#### 5. Threshold Classification

```
if total_events >= MIN_EVENT_THRESH:
    if max(|Δx|, |Δy|) >= MOTION_THRESH:
        if |Δy| > |Δx|:
            gesture = DOWN if Δy > 0 else UP
        else:
            gesture = RIGHT if Δx > 0 else LEFT
```

#### 6. Persistence Filter

Requires the same classification decision to hold across multiple consecutive window evaluations before asserting a valid gesture output.

### Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_FREQ_HZ` | 12 MHz | iCEBreaker oscillator |
| `WINDOW_MS` | 400 ms | Total observation window |
| `MIN_EVENT_THRESH` | 20 | Minimum events for valid gesture |
| `MOTION_THRESH` | 8 | Minimum motion magnitude |
| `PERSISTENCE_COUNT` | 2 | Consecutive windows needed |
| `FIFO_DEPTH` | 16 | Event buffer size |

### Resource Utilization (iCE40 UP5K)

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| Logic Cells | ~800 | 5,280 | **~15%** |
| Block RAM | 0 | 30 | **0%** |
| I/O Pins | 6 | 96 | 6% |
| Global Buffers | 8 | 8 | 100% |

The dual-accumulator design is extremely lightweight - no BRAM needed!

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
python fpga_gesture_validator.py --port COM3 --test echo

# Test all gestures
python fpga_gesture_validator.py --port COM3 --test all

# Send specific gesture
python fpga_gesture_validator.py --port COM3 --gesture right

# Interactive mode
python fpga_gesture_validator.py --port COM3 --interactive

# Continuous monitoring
python fpga_gesture_validator.py --port COM3 --continuous --duration 60
```

### Tips for Best Gesture Detection

- Use consistent, even lighting
- Plain background reduces noise events
- Move hand across ~1/3 of the frame
- Moderate speed (not too fast or slow)
- Adjust `--threshold` if too many/few events

## File Structure

```
├── rtl/                        # RTL source files (SystemVerilog)
│   ├── uart_gesture_top.sv     # Top-level module (UART + accelerator)
│   ├── dvs_gesture_accel.sv    # Asynchronous spatiotemporal classifier
│   ├── InputFIFO.sv            # 16-entry event FIFO
│   ├── SpatialCompressor.sv    # 320×320 → 16×16 spatial mapping
│   ├── TemporalAccumulator.sv  # Dual-window accumulation
│   ├── MotionComputer.sv       # Motion vector extraction
│   ├── GestureClassifier.sv    # Threshold-based classification
│   ├── OutputRegister.sv       # Persistence filter & output
│   ├── uart_rx.sv              # UART receiver (8N1)
│   └── uart_tx.sv              # UART transmitter (8N1)
│
├── tb/                         # Cocotb testbenches
│   ├── test_spatiotemporal_classifier.py  # Main cocotb testbench (18 tests)
│   ├── test_dvs_gesture.py     # Legacy testbench
│   ├── Makefile                # Legacy make-based cocotb runner (superseded by setup.py)
│   └── sim_build/              # Iverilog compilation artifacts (auto-generated)
│
├── synth/                      # FPGA synthesis
│   ├── icebreaker.pcf          # iCEBreaker pin constraints
│   ├── Makefile                # Legacy make-based synthesis (superseded by setup.py)
│   ├── uart_gesture_top.json   # Yosys output (auto-generated)
│   ├── uart_gesture_top.asc    # nextpnr output (auto-generated)
│   └── uart_gesture_top.bit    # Final bitstream (auto-generated)
│
├── tools/                      # Python utilities
│   ├── dvs_camera_emulator.py  # Webcam → DVS event converter
│   ├── dvs_event_player.py     # Replay saved DVS events
│   ├── fpga_gesture_validator.py  # Hardware test & validation
│   └── README.md               # Tools documentation
│
├── .devcontainer/              # VS Code Dev Container
│   ├── Dockerfile              # Pre-configured environment
│   └── devcontainer.json       # Container settings
│
├── setup.py                    # **Unified workflow script** (setup/test/synth/flash)
└── README.md                   # This file
```

**Workflow Philosophy**: The legacy Makefiles in `tb/` and `synth/` are preserved for reference but superseded by `setup.py`, which provides a consistent cross-platform interface to the same tools (cocotb + Icarus Verilog for verification, Yosys + nextpnr for synthesis).

## Running Tests and Synthesis

All verification and synthesis tasks are managed through the unified `setup.py` script, which automatically configures the toolchain environment.

### Run Cocotb Tests

```bash
python setup.py test
```

**What happens:**
1. Detects OSS CAD Suite installation and configures environment
2. Compiles all RTL files with Icarus Verilog (iverilog)
3. Runs cocotb testbench with VPI interface
4. Executes 18 verification tests:
   - UART echo test
   - Status query test
   - Config query test
   - Event injection test
   - FIFO backpressure test
   - Soft reset test
   - Coordinate range tests
   - All four gesture tests (UP, DOWN, LEFT, RIGHT)
   - Burst event handling
   - Temporal window verification
5. Reports pass/fail status for each test

### Synthesize for iCE40 UP5K

```bash
python setup.py synth
```

**What happens:**
1. Configures OSS CAD Suite environment (sets PATH, SSL_CERT_FILE, Qt paths, etc.)
2. Runs **Yosys** synthesis:
   - Reads all SystemVerilog sources
   - Performs logic synthesis
   - Maps to iCE40 primitives (LUTs, DFFs, BRAMs, carries)
   - Outputs JSON netlist
3. Runs **nextpnr-ice40** place and route:
   - Reads JSON netlist and pin constraints (icebreaker.pcf)
   - Places cells and routes signals
   - Optimizes timing for 12 MHz clock
   - Outputs ASCII bitstream (.asc)
4. Runs **icepack** bitstream generation:
   - Converts .asc to binary bitstream (.bit)
   - Final output: `synth/uart_gesture_top.bit`

**Resource Usage** (from last synthesis):
- Logic Cells: 1163/5280 (22%)
- Block RAM: 2/30 (6%)
- Max Frequency: 42.76 MHz (>3× headroom at 12 MHz target)

### Flash to iCEBreaker FPGA

```bash
python setup.py flash
```

**What happens:**
1. Checks for bitstream at `synth/uart_gesture_top.bit`
2. Runs **iceprog** to program via USB
3. FPGA immediately boots with new configuration

**Note**: Requires FPGA to be connected via USB and drivers installed (FTDI on Windows/Linux, libusb on macOS).

### Run DVS Camera Emulator

The camera emulator requires the virtual environment's Python packages (opencv-python, numpy, pyserial):

```bash
# Activate virtual environment first (only needed for tools, not for setup.py commands)
# Windows:
.venv\Scripts\activate
# Linux/macOS:
source .venv/bin/activate

cd tools

# Preview mode (no FPGA needed)
python dvs_camera_emulator.py --preview

# Simulation mode (synthetic gestures)
python dvs_camera_emulator.py --simulate --preview

# With FPGA connected
python dvs_camera_emulator.py --port /dev/ttyUSB1 --preview    # Linux
python dvs_camera_emulator.py --port COM3 --preview            # Windows
```

### Validate on Hardware

```bash
# Use the validator tool for comprehensive FPGA testing
cd tools
python fpga_gesture_validator.py --port COM3 --test all

# Other validation options:
python fpga_gesture_validator.py --port COM3 --test echo        # Quick connectivity test
python fpga_gesture_validator.py --port COM3 --interactive      # Manual control
python fpga_gesture_validator.py --list-ports                   # Find serial ports
```

**Why activate venv for tools?** The `setup.py` commands manage their own environment internally, but standalone Python scripts in `tools/` need the venv activated to access installed packages.

## Target Hardware

**FPGA**: Lattice iCE40 UP5K (iCEBreaker board)

| Pin | Signal | Description |
|-----|--------|-------------|
| 35 | `clk` | 12 MHz oscillator |
| 6 | `uart_rx` | FTDI TX → FPGA RX |
| 9 | `uart_tx` | FPGA TX → FTDI RX |
| 39 | `led_heartbeat` | Activity LED (~3Hz blink) |
| 40 | `led_gesture_valid` | Gesture detected LED |

## License

MIT License - See LICENSE file for details.
