# DVS Gesture Accelerator with Temporal Voxel Binning

FPGA-based gesture recognition accelerator for Dynamic Vision Sensor (DVS) events on iCE40 UP5K.

## Quick Start

### Option 1: Use Dev Container (Recommended for Codespaces)

1. Open in GitHub Codespaces or VS Code with Dev Containers extension
2. Click "Reopen in Container" when prompted
3. All tools are pre-installed and ready to use

### Option 2: Local Setup (Windows)

```powershell
# Clone the repo
git clone https://github.com/yourusername/FPGA-Matrix-Multiplication-Accelerator.git
cd FPGA-Matrix-Multiplication-Accelerator

# Run setup script (downloads ~300MB FPGA tools)
.\setup.ps1

# Activate environment
. .\activate.ps1

# Test the camera emulator
cd tools
python dvs_camera_emulator.py --preview
```

### Option 3: Local Setup (Linux/macOS)

```bash
# Clone the repo
git clone https://github.com/yourusername/FPGA-Matrix-Multiplication-Accelerator.git
cd FPGA-Matrix-Multiplication-Accelerator

# Run setup script (downloads ~1GB FPGA tools on Linux)
chmod +x setup.sh
./setup.sh

# Activate environment
source ./activate.sh

# Test the camera emulator
cd tools
python dvs_camera_emulator.py --preview
```

### Option 4: Manual Setup

If you already have tools installed:

```bash
# Python dependencies only
pip install opencv-python numpy pyserial cocotb pytest

# FPGA tools (install separately):
# - Yosys: https://github.com/YosysHQ/yosys
# - nextpnr: https://github.com/YosysHQ/nextpnr
# - Icarus Verilog: https://github.com/steveicarus/iverilog
# Or use OSS CAD Suite: https://github.com/YosysHQ/oss-cad-suite-build
```

## Architecture Overview

This is a **real-time gesture recognition system** that processes Dynamic Vision Sensor (DVS) events on an **iCE40 UP5K FPGA**. The system captures motion events from a camera (emulated via webcam), transmits them over UART, and classifies gestures (UP, DOWN, LEFT, RIGHT) using a lightweight centroid-based algorithm.

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
│ ┌───────┴─────────┐ │   1-byte gesture       │  │     dvs_gesture_accel          │ │
│ │ Webcam          │ │ ◄───────────────────   │  │  ┌───────────────────────────┐ │ │
│ │                 │ │   [0xA0 | gesture]     │  │  │ Centroid Accumulators     │ │ │
│ └─────────────────┘ │                        │  │  │ (Early/Late halves)       │ │ │
│                     │                        │  │  └───────────────────────────┘ │ │
│                     │                        │  │              │                  │ │
│                     │                        │  │              ▼                  │ │
│                     │                        │  │  ┌───────────────────────────┐ │ │
│                     │                        │  │  │ Gesture Classifier        │ │ │
│                     │                        │  │  │ (delta comparison)        │ │ │
│                     │                        │  │  └───────────────────────────┘ │ │
│                     │                        │  └─────────────────┬──────────────┘ │
│                     │                        │                    ▼                │
│                     │                        │  ┌──────────┐  ┌────────┐          │
│                     │                        │  │ uart_tx  │◄─┤ TX FSM │          │
│                     │                        │  └──────────┘  └────────┘          │
│                     │                        │                                      │
│                     │                        │  LEDs: led_heartbeat (~3Hz)         │
│                     │                        │        led_gesture_valid (pulse)    │
└─────────────────────┘                        └──────────────────────────────────────┘
```

### Hardware Modules

| Module | File | Description |
|--------|------|-------------|
| `uart_gesture_top` | `rtl/uart_gesture_top.sv` | Top-level wrapper, UART protocol, packet parsing |
| `dvs_gesture_accel` | `rtl/dvs_gesture_accel.sv` | Core accelerator with centroid accumulators |
| `uart_rx` | `rtl/uart_rx.sv` | UART receiver (8N1, 115200 baud) |
| `uart_tx` | `rtl/uart_tx.sv` | UART transmitter (8N1, 115200 baud) |

### Processing Pipeline

```
DVS Events (320×320) → Spatial Compression (16×16) → Temporal Binning (8 bins)
                    → Centroid Accumulation → Delta Comparison → Gesture Output
```

#### 1. Spatial Compression (320×320 → 16×16)

Events are mapped to a 16×16 grid using fast integer math:
```
grid_x = (event_x × 13) >> 8   // Approximates x/20
grid_y = (event_y × 13) >> 8
```

#### 2. Temporal Binning (8 bins, 50ms each)

The 400ms sliding window is divided into 8 bins:
- **Early half** (bins 0-3): First 200ms of events
- **Late half** (bins 4-7): Last 200ms of events

#### 3. Centroid Accumulators

Instead of storing full voxel tensors (which would require BRAM), we accumulate:
- `early_sum_x`, `early_sum_y` - Sum of positions in early half
- `late_sum_x`, `late_sum_y` - Sum of positions in late half
- `early_count`, `late_count` - Event counts

This allows gesture detection with **zero BRAM usage**.

#### 4. Gesture Classification

When the temporal window completes:
```
delta_x = late_sum_x - early_sum_x
delta_y = late_sum_y - early_sum_y

if |delta_y| > |delta_x|:
    gesture = DOWN if delta_y > 0 else UP
else:
    gesture = LEFT if delta_x > 0 else RIGHT
```

### Timing Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Clock | 12 MHz | iCEBreaker oscillator |
| Bin Period | 50 ms | 600,000 cycles per bin |
| Window Period | 400 ms | 8 bins total |
| UART Baud | 115200 | ~87 µs per byte |

### Resource Utilization (iCE40 UP5K)

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| Logic Cells | 685 | 5,280 | **12%** |
| Block RAM | 0 | 30 | **0%** |
| I/O Pins | 5 | 96 | 5% |
| Global Buffers | 8 | 8 | 100% |

The centroid-based design is extremely lightweight - no BRAM needed!

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

### Receive from FPGA (1 byte)

**Gesture Response**: `0xA0 | gesture`
- 0xA0 = UP
- 0xA1 = DOWN  
- 0xA2 = LEFT
- 0xA3 = RIGHT

**Status Response**: `0xB0 | bin`
- Current temporal bin (0-7)

### Special Commands

| Send | Response | Description |
|------|----------|-------------|
| 0xFF | 0x55 | Echo test (verify UART) |
| 0xFE | 0xBx | Status query (current bin) |

## DVS Camera Emulator

The `tools/dvs_camera_emulator.py` script converts your laptop webcam into a DVS-like sensor:

1. **Captures frames** from webcam at 30 FPS
2. **Computes log intensity** changes between frames
3. **Generates events** where intensity changed beyond threshold
4. **Transmits events** via UART to the FPGA

### Modes

| Mode | Command | Description |
|------|---------|-------------|
| Preview | `--preview` | Show camera feed with DVS events overlay |
| Simulate | `--simulate` | Synthetic gestures (no camera needed) |
| FPGA | `--port /dev/ttyUSB1` | Send events to FPGA |
| Record | `--save events.bin` | Save events to file |

### Tips for Best Gesture Detection

- Use consistent, even lighting
- Plain background reduces noise events
- Move hand across ~1/3 of the frame
- Moderate speed (not too fast or slow)
- Adjust `--threshold` if too many/few events

## File Structure

```
├── rtl/                        # RTL source files
│   ├── uart_gesture_top.sv     # Top-level module (UART + accelerator)
│   ├── dvs_gesture_accel.sv    # Gesture accelerator core
│   ├── uart_rx.sv              # UART receiver (8N1)
│   └── uart_tx.sv              # UART transmitter (8N1)
│
├── tb/                         # Testbenches
│   ├── Makefile                # Cocotb simulation makefile
│   ├── test_dvs_gesture.py     # Main cocotb testbench
│   └── sim_build/              # Simulation build artifacts
│
├── synth/                      # Synthesis files
│   ├── Makefile                # Yosys + nextpnr flow
│   ├── icebreaker.pcf          # iCEBreaker pin constraints
│   └── uart_gesture_top.bit    # Generated bitstream
│
├── tools/                      # Python tools
│   ├── dvs_camera_emulator.py  # Webcam → DVS emulator
│   ├── dvs_event_player.py     # Replay saved events
│   └── README.md               # Tools documentation
│
├── .devcontainer/              # VS Code Dev Container config
│   ├── Dockerfile              # Container with all tools
│   └── devcontainer.json       # Dev container settings
│
├── setup.ps1                   # Windows setup script
├── setup.sh                    # Linux/macOS setup script
├── validate_ice40.py           # Hardware validation script
└── README.md                   # This file
```

## Quick Start

### Simulate with Cocotb

```bash
cd tb
make clean
make
```

This runs all tests including:
- UART echo test
- Status query test
- Coordinate range test
- All four gesture tests (UP, DOWN, LEFT, RIGHT)
- Burst event handling
- Temporal binning verification

### Synthesize for iCE40


```bash
cd synth

# Using Makefile (requires sv2v)
make clean && make

# Or directly with Yosys (no sv2v needed)
yosys -p "read_verilog -sv ../rtl/*.sv; synth_ice40 -top uart_gesture_top -json uart_gesture_top.json"
nextpnr-ice40 --up5k --package sg48 --json uart_gesture_top.json --pcf icebreaker.pcf --asc uart_gesture_top.asc
icepack uart_gesture_top.asc uart_gesture_top.bit
```

### Program iCEBreaker

```bash
# Linux
iceprog uart_gesture_top.bit

# Windows with WSL (attach USB first)
usbipd attach --wsl --busid <BUS_ID>
wsl -e iceprog uart_gesture_top.bit
```

### Run DVS Camera Emulator

```bash
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
python validate_ice40.py /dev/ttyUSB0
```

Options:
- `-v`: Verbose output
- `-n 500`: Use 500 events per gesture test
- `--echo-only`: Quick UART connectivity test
- `--reset`: Toggle DTR to reset FPGA

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
