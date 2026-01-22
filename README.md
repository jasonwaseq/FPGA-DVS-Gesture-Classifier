# DVS Gesture Accelerator for Ice40 FPGA

A lightweight matrix multiplication accelerator for classifying gesture movements from Dynamic Vision Sensor (DVS) event data. Includes UART interface for testing without real DVS hardware.

## Overview

This accelerator processes DVS events (x, y, polarity) and classifies them into 4 gestures: **UP**, **DOWN**, **LEFT**, **RIGHT**.

### How It Works

1. **Event Accumulation**: DVS events are accumulated over a window of N events
   - Each event contributes its position weighted by polarity (+1 for ON, -1 for OFF)
   - Tracks sum of weighted positions: `sum_x += x * polarity`, `sum_y += y * polarity`

2. **Delta Computation**: After N events, compute movement delta
   - `delta_x = sum_x - previous_sum_x`
   - `delta_y = sum_y - previous_sum_y`

3. **Matrix Multiplication**: Multiply delta vector by weight matrix
   ```
   [score_up   ]   [ 0   64] [delta_x]
   [score_down ] = [ 0  -64] [delta_y]
   [score_left ]   [-64   0]
   [score_right]   [64    0]
   ```

4. **Classification**: Output gesture with highest score (argmax)

## File Structure

```
├── README.md
├── rtl/
│   ├── uart_gesture_top.sv     # Top module with UART
│   ├── uart_rx.sv              # UART receiver
│   ├── uart_tx.sv              # UART transmitter
│   ├── dvs_gesture_accel.sv    # Gesture accelerator
│   ├── event_accumulator.sv    # Event accumulation
│   └── gesture_classifier.sv   # Matrix multiply + argmax
├── tb/
│   ├── test_gesture.py         # Cocotb testbench
│   └── Makefile
├── synth/
│   ├── Makefile                # Ice40 synthesis
│   └── icebreaker.pcf          # Pin constraints
└── validate_ice40.py           # Hardware validation script
```

## UART Protocol

**Baud Rate**: 115200, 8N1

**Send Event to FPGA** (4 bytes):
| Byte | Description |
|------|-------------|
| 0 | X coordinate (0-127) |
| 1 | Y coordinate (0-127) |
| 2 | Polarity (1=ON, 0=OFF) |
| 3 | Timestamp (unused, can be 0) |

**Receive from FPGA** (1 byte when gesture detected):
| Bits | Description |
|------|-------------|
| [7:4] | Marker (0xA) |
| [1:0] | Gesture: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT |

## Quick Start

### 1. Simulation
```bash
cd tb
make
```

### 2. Synthesis
```bash
cd synth
make
```

### 3. Program iCEBreaker
```bash
cd synth
make prog
```

### 4. Validate on Hardware
```bash
python validate_ice40.py /dev/ttyUSB0
```

## Resource Usage (Ice40 UP5K)

- ~500 LUTs
- ~250 FFs
- 0 BRAM

## License

MIT
