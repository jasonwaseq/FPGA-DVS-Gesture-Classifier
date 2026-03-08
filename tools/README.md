# DVS Gesture Classifier — Tools

This directory contains four tools for working with the FPGA gesture classification pipeline.

python tools/dvs_camera_emulator.py --port COM3 --baud 115200 --preview --no-noise --contrast 0.20 --max-events 86

## Tools at a Glance

| Tool | Purpose |
|------|---------|
| [`dvs_camera_emulator.py`](#dvs_camera_emulatorpy) | Webcam → DVS event emulator; sends EVT 2.0 to FPGA or saves to file |
| [`evt_stream_to_fpga.py`](#evt_stream_to_fpgapy) | Live relay (GenX320 → FPGA) and .raw file replay |
| [`fpga_gesture_validator.py`](#fpga_gesture_validatorpy) | Automated validation: injects synthetic gestures, checks FPGA output |
| [`capture_evt_stream.py`](#capture_evt_streampy) | Capture raw EVT 2.0 bytes from DVS serial port to file |

---

## Installation

```bash
pip install opencv-python numpy pyserial
```

---

## dvs_camera_emulator.py

Converts a standard webcam (or video file, or built-in synthetic gestures) into a stream of
Prophesee EVT 2.0 events, sending them over UART to the FPGA or saving to a `.raw` file.

### How the DVS Model Works

Each pixel maintains a **per-pixel log-intensity reference**. An ON event fires when the
current log-intensity exceeds the reference by the contrast threshold θ; an OFF event fires
when it drops below −θ. Only pixels that fire update their reference. A refractory period
prevents the same pixel from re-firing immediately.

A **reference leak** slowly pulls each pixel's reference toward the current scene intensity,
preventing stale references from generating spurious events when motion stops.

### Input Modes

| Mode | Flag | Description |
|------|------|-------------|
| Camera | *(default)* | Live webcam input |
| Video file | `--video FILE` | Process a video file as DVS input |
| Simulation | `--simulate` | Built-in synthetic gesture generator (no camera needed) |

### Quick Start

**Synthetic simulation (no camera or FPGA needed):**
```bash
python tools/dvs_camera_emulator.py --simulate --preview
```

**Camera → FPGA (Windows):**
```bash
python tools/dvs_camera_emulator.py --port COM3 --baud 115200 --preview --flip-both
```

**Camera → save to file:**
```bash
python tools/dvs_camera_emulator.py --save recording.raw --preview --flip-both
```

**Video file → FPGA:**
```bash
python tools/dvs_camera_emulator.py --video myvideo.mp4 --port COM3 --baud 115200 --preview
```

### Axis Orientation

The FPGA weights were trained with a camera mounted with **both axes physically inverted**.
When using a real webcam or video for live gesture input, you must flip both axes to match:

```bash
python tools/dvs_camera_emulator.py --port COM3 --preview --flip-both
```

`--simulate` mode uses hardware-calibrated trajectories automatically — no flip flags needed.

### All Options

| Option | Default | Description |
|--------|---------|-------------|
| `--camera N` | `0` | Camera device ID |
| `--video FILE` | — | Use video file instead of camera |
| `--simulate` | — | Synthetic gesture mode (no camera) |
| `--port PORT` | — | FPGA serial port (e.g. `COM3`, `/dev/ttyUSB0`) |
| `--baud N` | `115200` | FPGA UART baud rate |
| `--contrast F` | `0.15` | Log-domain contrast threshold per event (~15% intensity change) |
| `--refractory N` | `200` | Refractory period in µs (GenX320 spec: ~200 µs) |
| `--fps N` | `30` | Camera/simulation frame rate |
| `--resolution N` | `320` | DVS output resolution (NxN) |
| `--preview` | — | Show OpenCV preview window |
| `--save FILE` | — | Save events to EVT 2.0 `.raw` file |
| `--max-events N` | `1000` | Max events/frame for UART send path (auto-clamped to link budget) |
| `--subsample-mode` | `spatial` | UART downsampling: `spatial` (preserves directional structure) or `uniform` |
| `--aspect-mode` | `crop` | Input frame resize: `crop` (square, no distortion) or `stretch` |
| `--roi-scale F` | `1.0` | Center crop scale (0.2–1.0); lower values suppress peripheral background |
| `--flip-x` | — | Flip input horizontally before DVS conversion |
| `--flip-y` | — | Flip input vertically before DVS conversion |
| `--flip-both` | — | Flip both axes (equivalent to `--flip-x --flip-y`); use for real webcam gestures |
| `--noise-filter N` | `3` | Gaussian blur kernel size for noise reduction (0 = disabled) |
| `--no-noise` | — | Disable shot noise and hot-pixel background noise model |
| `--leak-rate F` | `0.05` | Reference leak rate per second toward current scene intensity |
| `--shot-noise F` | `0.0001` | Shot noise probability per pixel per frame |
| `--loop` | — | Loop video file playback |

### Keyboard Controls (Preview Window)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `+` / `=` | Increase contrast threshold |
| `-` | Decrease contrast threshold |
| `n` | Toggle noise model on/off |
| `r` | Reset event/frame statistics |
| `Space` | Pause / resume |
| `1` | Queue UP gesture *(simulation mode)* |
| `2` | Queue DOWN gesture *(simulation mode)* |
| `3` | Queue LEFT gesture *(simulation mode)* |
| `4` | Queue RIGHT gesture *(simulation mode)* |

### UART Budget

At 115200 baud / 30 FPS the UART can sustain ~96 events/frame (4 bytes/event, 10 bits/byte,
90% margin). `--max-events` is automatically clamped to this limit. Increase `--baud` for
higher throughput — the iCEBreaker UART supports up to 1 Mbaud at 12 MHz.

### Gesture Encoding (FPGA → PC)

Gesture packets received from the FPGA: `[0xA0 | class, conf_byte]`

| Class | Gesture |
|-------|---------|
| `0` | Down |
| `1` | Left |
| `2` | Right |
| `3` | Up |

---

## evt_stream_to_fpga.py

Relays a live Prophesee GenX320 EVT 2.0 stream (via STM32 CDC UART) to the iCE40 FPGA, or
replays a captured `.raw` / `.bin` file at the original capture rate.

The GenX320 emits **little-endian** 32-bit words. `voxel_bin_top` expects **big-endian**
(MSB-first) words. This script byte-swaps every word before writing to the FPGA port.

### Operating Modes

**Live relay** — GenX320 connected via STM32 CDC USB to laptop:
```bash
python tools/evt_stream_to_fpga.py --dvs COM4 --fpga COM3 --dvs-baud 3000000 --fpga-baud 115200
```

**File replay** — replay a previously captured `.raw` at real-time speed:
```bash
python tools/evt_stream_to_fpga.py --file capture.raw --fpga COM3 --fpga-baud 115200
```

**File replay at 2× speed:**
```bash
python tools/evt_stream_to_fpga.py --file capture.raw --fpga COM3 --replay-rate 2.0
```

**File replay as fast as UART allows (no pacing):**
```bash
python tools/evt_stream_to_fpga.py --file capture.raw --fpga COM3 --replay-rate 0
```

**Loop file replay continuously:**
```bash
python tools/evt_stream_to_fpga.py --file capture.raw --fpga COM3 --loop
```

**Save live stream to file while relaying:**
```bash
python tools/evt_stream_to_fpga.py --dvs COM4 --fpga COM3 --save-raw session.raw
```

### Axis Swap

If the camera is mounted 90° rotated and LEFT/RIGHT are misclassified as UP/DOWN, use:
```bash
python tools/evt_stream_to_fpga.py --dvs COM4 --fpga COM3 --swap-xy
```

Run `evt2_layout_probe.py` first to confirm the axis layout before using this flag.

### All Options

| Option | Default | Description |
|--------|---------|-------------|
| `--dvs PORT` | `/dev/ttyACM0` | DVS / STM32 CDC serial port (live mode) |
| `--fpga PORT` | `/dev/ttyUSB1` | FPGA UART serial port |
| `--dvs-baud N` | `3000000` | DVS port baud rate (GenX320/STM32 CDC typically 2–5 Mbps) |
| `--fpga-baud N` | `115200` | FPGA UART baud rate |
| `--file FILE` | — | Replay a `.raw` or `.bin` file instead of live relay |
| `--replay-rate F` | `1.0` | File replay speed multiplier (1.0=real-time, 0=no pacing) |
| `--loop` | — | Loop file replay continuously |
| `--save-raw FILE` | — | Save live DVS stream to file while relaying |
| `--swap-xy` | — | Swap X/Y fields in CD words (for 90°-rotated camera) |
| `--duration F` | `0` | Stop after N seconds (0 = run until Ctrl+C) |
| `--chunk N` | `4096` | Read chunk size in bytes |
| `--probe-bytes N` | `8192` | Bytes used for byte-alignment auto-detection |
| `--max-write-bytes N` | `512` | Max bytes per FPGA write call |
| `--no-echo-check` | — | Skip 0xFF echo probe on startup |
| `--debug` | — | Print gesture detections and extra diagnostics |

### Status Output

While running, a status line prints every second:
```
  7.0s  words=84123  valid=0.998  cd=81045  th=3078  sent=84123  wr_err=0  Down=0 Left=2 Right=3 Up=0
```

| Field | Description |
|-------|-------------|
| `words` | Total EVT2 words seen |
| `valid` | Fraction with a known EVT2 type code |
| `cd` | CD_ON + CD_OFF event words |
| `th` | TIME_HIGH words |
| `sent` | Words forwarded to FPGA |
| `wr_err` | FPGA write timeouts |
| `Down/Left/Right/Up` | Gesture detections since start |

---

## fpga_gesture_validator.py

Automated end-to-end validation tool. Injects synthetic gesture events (trajectory-based,
hardware-calibrated) directly to the FPGA over UART and verifies that the correct gesture
class is returned.

### Quick Start

**Test a single gesture:**
```bash
python tools/fpga_gesture_validator.py --port COM3 --test down
python tools/fpga_gesture_validator.py --port COM3 --test left
python tools/fpga_gesture_validator.py --port COM3 --test right
python tools/fpga_gesture_validator.py --port COM3 --test up
```

**Test all 4 gestures, 5 trials each:**
```bash
python tools/fpga_gesture_validator.py --port COM3 --test all --trials 5
```

**Higher event density (more confident classifications):**
```bash
python tools/fpga_gesture_validator.py --port COM3 --test all --trials 5 --events 400
```

### How It Works

1. Sends a **soft reset** (`0xFC`) to the FPGA to zero the bin timer.
2. Waits **1 second** (one full window) so all 4 voxel bins cycle through clear, flushing
   stale data from any previous trial.
3. Sends synthetic trajectory events across 12 consecutive bin slots, distributing events
   uniformly across all 4 time bins.
4. Collects `[0xA0|class, conf]` packets from the FPGA and reports the dominant detected class.

The trajectories are calibrated to the hardware axis convention (both axes inverted vs
physical camera space) — the same trajectories that `fpga_gesture_validator.py` uses are
documented in the Camera Coordinate System section of the project MEMORY.

### All Options

| Option | Default | Description |
|--------|---------|-------------|
| `--port PORT` | *(required)* | FPGA serial port |
| `--baud N` | `115200` | FPGA UART baud rate |
| `--test GESTURE` | `all` | Gesture to test: `down`, `left`, `right`, `up`, or `all` |
| `--trials N` | `3` | Trials per gesture |
| `--events N` | `200` | Synthetic events per gesture window |
| `--timeout F` | `3.0` | Seconds to wait for FPGA gesture packet per trial |

### Expected Output

```
Testing DOWN (trial 1/5) ... PASS (got Down, expected Down)
Testing DOWN (trial 2/5) ... PASS (got Down, expected Down)
...
Results: 20/20 passed (100.0%)
```

---

## capture_evt_stream.py

Captures raw EVT 2.0 bytes from a serial port to a binary file. Automatically detects
byte alignment and produces an aligned `.bin` alongside a validation report.

### Usage

**Capture until ENTER is pressed:**
```bash
python tools/capture_evt_stream.py COM4 output.raw
```

**Capture for a fixed duration:**
```bash
python tools/capture_evt_stream.py COM4 output.raw --duration 10.0
```

**Specify aligned output filename:**
```bash
python tools/capture_evt_stream.py COM4 output.raw --aligned output_aligned.bin
```

### What It Produces

After capture completes, the tool:
1. Detects the correct 4-byte word alignment offset (0–3 bytes) by scanning up to
   `--sample-words` words and picking the offset that yields the highest fraction of
   valid EVT2 type codes.
2. Writes an aligned `.bin` file (default: `aligned.bin`) starting at the detected offset.
3. Prints an **EVT2 validation report**: total words, valid-type ratio, CD event count,
   coordinate range violations, TIME_HIGH count, timestamp regressions.

### All Options

| Option | Default | Description |
|--------|---------|-------------|
| `port` | *(required)* | Serial port (e.g. `COM4`, `/dev/ttyACM0`) |
| `outfile` | *(required)* | Raw output file path |
| `--baud N` | `115200` | Baud rate (USB CDC ignores this, but pyserial requires it) |
| `--chunk N` | `4096` | Read chunk size in bytes |
| `--aligned FILE` | `aligned.bin` | Aligned output file path |
| `--sample-words N` | `50000` | Words to sample for alignment detection |
| `--duration F` | `0` | Auto-stop after N seconds (0 = press ENTER to stop) |
| `--max-words N` | `0` | Max words to analyze in report (0 = all) |
| `--max-x N` | `319` | Max valid X coordinate for GenX320 |
| `--max-y N` | `319` | Max valid Y coordinate for GenX320 |

The EVT2 field mask/shift options (`--x-shift`, `--x-mask`, `--ts-shift`, etc.) default to
standard GenX320 EVT 2.0 values and rarely need changing.

---

## EVT 2.0 Word Format

All tools use this 32-bit little-endian word layout (Prophesee EVT 2.0):

```
 31       28 27     22 21        11 10         0
 ┌──────────┬─────────┬───────────┬─────────────┐
 │  type[3:0]│ ts_lsb  │  x[10:0]  │   y[10:0]   │
 └──────────┴─────────┴───────────┴─────────────┘
```

| Type | Value | Description |
|------|-------|-------------|
| CD_OFF | `0x0` | Brightness decreased at (x, y) |
| CD_ON | `0x1` | Brightness increased at (x, y) |
| TIME_HIGH | `0x8` | Upper 28-bit timestamp; full µs = `{time_high[27:0], ts_lsb[5:0]}` |

**UART byte order:** The FPGA (`voxel_bin_top`) expects words **big-endian** (MSB first).
Files from the GenX320 / `capture_evt_stream.py` are **little-endian**. All relay and
emulator tools perform the swap automatically before writing to the FPGA port.

---

## Troubleshooting

**FPGA echo check fails (`WARNING: echo probe did not return 0x55`):**
- Verify the correct COM port and baud rate.
- Confirm the FPGA is programmed with the `voxel_bin_top` bitstream.
- Use `--no-echo-check` to bypass if you know the FPGA is running.

**No events from webcam:**
- Lower contrast threshold: `--contrast 0.08`
- Check camera is not covered or in darkness.
- Ensure sufficient lighting and motion in front of the camera.

**Too many events / UART overflow:**
- Increase threshold: `--contrast 0.25`
- Lower max events: `--max-events 64`
- Use spatial subsampling: `--subsample-mode spatial`

**Wrong gesture classifications from webcam:**
- Add `--flip-both` — the training camera had both axes inverted.
- Use `--simulate` to verify the FPGA classifies the built-in synthetic gestures correctly
  before testing with a real webcam.

**LEFT/RIGHT classified as UP/DOWN from GenX320:**
- Camera is likely mounted 90° rotated. Add `--swap-xy` to `evt_stream_to_fpga.py`.

**Low EVT2 valid ratio warning in `evt_stream_to_fpga.py`:**
- Check `--dvs-baud` matches the STM32 CDC output rate (typically 2–5 Mbps).
- The GenX320 default is 3 Mbps — use `--dvs-baud 3000000`.

**File replay completes instantly without FPGA responding:**
- Add `--replay-rate 1.0` (default) to pace replay using TIME_HIGH timestamps.
- Without pacing (`--replay-rate 0`) the FPGA's 115200-baud input FIFO overflows.
