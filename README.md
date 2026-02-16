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

## Current architecture (short)

DVS events stream over UART and are processed in one pass:

DVS events -> Input FIFO -> Spatial compression (320x320 to 16x16) -> Time surface accumulation -> Motion vector -> Gesture classifier -> UART response

## Old architecture (legacy)

The previous pipeline lives in [rtl/old_architecture](rtl/old_architecture). It contains an earlier module split and naming scheme (for example `SpatialCompressor`, `TemporalAccumulator`, `MotionComputer`) and is kept for reference only. It is not maintained and may not match current tests or tools.

## Common commands

| Command | Description |
|---------|-------------|
| `python setup.py` | Setup venv, install packages, detect OSS CAD Suite |
| `python setup.py test` | Run verification tests (iverilog + cocotb) |
| `python setup.py synth` | Synthesize (Yosys -> nextpnr -> icepack) |
| `python setup.py flash` | Program FPGA via iceprog |
| `python setup.py clean` | Remove build artifacts |

## Useful tools

```bash
.venv/bin/python tools/dvs_camera_emulator.py --simulate --preview
.venv/bin/python tools/fpga_gesture_validator.py --list-ports
.venv/bin/python tools/fpga_diagnostic.py
```
