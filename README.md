# DVS Gesture Classifier for iCE40 FPGA

Real-time gesture recognition (UP, DOWN, LEFT, RIGHT) using Dynamic Vision Sensor events on iCE40 UP5K.

## Quick Start

```bash
# Clone and setup
git clone https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier.git
cd FPGA-DVS-Gesture-Classifier
python setup.py

# Verify, synthesize, flash
python setup.py test              # Run 18 cocotb tests
python setup.py synth             # Synthesize to bitstream
python setup.py flash             # Program FPGA via iceprog
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

**Pipeline**: DVS Events (320×320) → FIFO → Spatial Compression (16×16) → Dual Accumulators → Motion Vector → Gesture Classification

**Algorithm**: Compares early vs. late event centroids in a sliding window to extract motion direction.

**Hardware**: 1163 LCs (22%), 2 BRAM (6%), 42.76 MHz max freq on iCE40 UP5K

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

### Tips for Best Gesture Detection

- Use consistent, even lighting
- Plain background reduces noise events
- Move hand across ~1/3 of the frame
- Moderate speed (not too fast or slow)
- Adjust `--threshold` if too many/few events

## Project Structure

```
rtl/          # SystemVerilog modules
tb/           # Cocotb testbenches (18 tests)
synth/        # Synthesis outputs & constraints
tools/        # Python utilities (camera emulator, validator)
setup.py      # Main workflow script
```

## Hardware

**Target**: iCEBreaker (iCE40 UP5K)  
**UART**: 115200 baud, pins 6 (RX), 9 (TX)  
**Protocol**: 5-byte events `[X_HI, X_LO, Y_HI, Y_LO, POL]`, 2-byte gestures `[0xA0|gesture, confidence]`

## License

MIT