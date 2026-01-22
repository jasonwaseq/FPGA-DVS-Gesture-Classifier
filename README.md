# DVS Gesture Accelerator

Matrix multiplication accelerator for classifying DVS gestures on iCE40 FPGA.

## How It Works

Accumulates 100 DVS events, computes movement delta [Δx, Δy], multiplies by a 4×2 weight matrix, and outputs the gesture with the highest score.

```
DVS Events → Accumulate → Matrix Multiply → Argmax → Gesture
```

**Weight Matrix:**
```
         Δx    Δy
UP     [  0   64 ]
DOWN   [  0  -64 ]
LEFT   [-64    0 ]
RIGHT  [ 64    0 ]
```


## UART Protocol

115200 baud, 8N1, 12 MHz clock

**Send to FPGA (4 bytes):**
| Byte | Value |
|------|-------|
| 0 | X (0-127) |
| 1 | Y (0-127) |
| 2 | Polarity (0/1) |
| 3 | Timestamp (unused) |

**Receive from FPGA (1 byte):**
- Format: `0xA0 | gesture`
- Gesture: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT

## Quick Start

**Simulate:**
```bash
cd tb && make
```

**Synthesize & Program:**
```bash
cd synth && make && make prog
```

**Test on Hardware:**
```bash
python3 validate_ice40.py /dev/ttyUSB1
```
