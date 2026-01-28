# DVS Camera Emulator Tools

This directory contains tools for emulating a Dynamic Vision Sensor (DVS) using a standard webcam, enabling testing of the DVS Gesture Accelerator without actual DVS hardware.

## Overview

Dynamic Vision Sensors (DVS) are neuromorphic cameras that detect changes in pixel intensity asynchronously, outputting events only when brightness changes beyond a threshold. This is fundamentally different from traditional frame-based cameras.

These tools convert webcam frames into DVS-like events by:
1. Computing log intensity of consecutive frames
2. Detecting pixels where intensity changed beyond a threshold
3. Generating ON events (brightness increased) and OFF events (brightness decreased)
4. Transmitting events via UART to the FPGA

## Files

| File | Description |
|------|-------------|
| `dvs_camera_emulator.py` | Main emulator - captures webcam and generates DVS events in real-time |
| `dvs_event_player.py` | Replays saved event files to FPGA |

## Installation

```bash
# Ubuntu/Debian - install all dependencies system-wide
sudo apt install python3-opencv python3-serial

# That's it! No virtual environment needed.
```

For other systems (Windows/macOS), use pip:
```bash
pip install opencv-python numpy pyserial
```

## Usage

### DVS Camera Emulator

**Simulation mode (no camera required - great for testing):**
```bash
python dvs_camera_emulator.py --simulate --preview
```
Press keys 1-4 to trigger gestures (UP/DOWN/LEFT/RIGHT).

**Use a video file as input:**
```bash
python dvs_camera_emulator.py --video gesture_video.mp4 --preview
```

**Preview mode with camera:**
```bash
python dvs_camera_emulator.py --preview
```

**Send events to FPGA:**
```bash
# Linux
python dvs_camera_emulator.py --port /dev/ttyUSB0 --preview

# Windows
python dvs_camera_emulator.py --port COM3 --preview

# macOS
python dvs_camera_emulator.py --port /dev/cu.usbserial-* --preview
```

**Save events to file for later replay:**
```bash
python dvs_camera_emulator.py --save recording.bin --preview
```

**Adjust sensitivity:**
```bash
# More sensitive (detects smaller changes)
python dvs_camera_emulator.py --threshold 10 --preview

# Less sensitive (only large changes)
python dvs_camera_emulator.py --threshold 25 --preview
```

**All options:**
```bash
python dvs_camera_emulator.py --help
```

### Event Player

**Replay saved events:**
```bash
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --preview
```

**Adjust playback speed:**
```bash
# 2x speed
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --speed 2.0

# Slow motion
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --speed 0.5
```

**Loop playback:**
```bash
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --loop
```

## Keyboard Controls (Preview Mode)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `+` / `-` | Increase / decrease threshold |
| `r` | Reset statistics |
| `Space` | Pause / resume |

## UART Protocol

Events are sent as 5 bytes matching the `uart_gesture_top.sv` protocol:

| Byte | Content | Description |
|------|---------|-------------|
| 0 | `X_HI` | X coordinate bit 8 (MSB) |
| 1 | `X_LO` | X coordinate bits 7:0 |
| 2 | `Y_HI` | Y coordinate bit 8 (MSB) |
| 3 | `Y_LO` | Y coordinate bits 7:0 |
| 4 | `POL` | Polarity (1=ON, 0=OFF) |

Coordinates range from 0-319 to match the 320x320 sensor resolution.

## Tips for Best Results

1. **Lighting**: Consistent, even lighting produces cleaner events. Avoid direct sunlight or flickering lights.

2. **Threshold tuning**: 
   - Start with default threshold (15)
   - Lower values = more events, may include noise
   - Higher values = fewer events, misses subtle motion

3. **Background**: A plain, non-moving background reduces noise events.

4. **Camera placement**: Mount camera steadily to avoid whole-frame events from camera shake.

5. **Gestures**: For gesture detection:
   - Move hand across camera view in clear UP/DOWN/LEFT/RIGHT motions
   - Speed should be moderate (not too fast or slow)
   - Cover ~1/3 to 1/2 of the frame with your gesture

## Event File Format

Binary files saved by the emulator use this format:

```
Header: "DVS1" (4 bytes)
Events: [x(2), y(2), polarity(1), timestamp_us(4)] = 9 bytes each
```

All values are little-endian.

## Troubleshooting

**No camera found:**
- Check camera is connected and not in use by another application
- Try different camera ID: `--camera 1`

**No events generated:**
- Lower threshold: `--threshold 10`
- Add more motion in front of camera
- Check camera is not covered or in complete darkness

**Too many events (noisy):**
- Increase threshold: `--threshold 25`
- Enable noise filter: `--noise-filter 5`
- Improve lighting conditions

**UART connection issues:**
- Verify correct port name
- Check baud rate matches FPGA (115200)
- Ensure FPGA is programmed and running
- Test with echo command: send `0xFF`, should receive `0x55`
