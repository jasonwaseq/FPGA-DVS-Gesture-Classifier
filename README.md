# DVS Gesture Classifier for iCE40 FPGA

Real-time gesture recognition (UP, DOWN, LEFT, RIGHT) from DVS events on iCE40 UP5K.

## Getting started

There are two ways to use this project: inside the **VS Code Dev Container** (recommended, zero-install) or as a **local clone** on Linux, macOS, or Windows.

---

## Option 1 — Dev Container

The repo ships a `.devcontainer/` that provides a fully configured Ubuntu environment with OSS CAD Suite (Yosys, nextpnr, Icarus Verilog), sv2v, GTKWave, and all Python dependencies pre-installed.

> **What works in the devcontainer vs. locally**
>
> | Task | Devcontainer | Local |
> |------|:---:|:---:|
> | Simulation / cocotb tests | ✅ | ✅ |
> | Synthesis (Yosys + nextpnr) | ✅ | ✅ |
> | Flash FPGA (`iceprog`) | ❌ | ✅ |
> | DVS camera emulator / UART tools | ❌ | ✅ |
>
> GitHub Codespaces and local Dev Containers do not have access to USB devices, so `iceprog` and any tool that opens a serial port to the FPGA must be run on your local machine. Synthesize inside the container, copy the `.bit` file out, and flash locally.

### Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (or Docker Engine on Linux)
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Open in container

1. Clone the repo and open it in VS Code:
   ```bash
   git clone https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier.git
   code FPGA-DVS-Gesture-Classifier
   ```
2. When VS Code prompts **"Reopen in Container"**, click it (or run **Dev Containers: Reopen in Container** from the command palette).
3. The container builds once and then runs `python3 setup.py --skip-fpga` automatically to create the `.venv` and install Python packages.

### Run tests (devcontainer)

```bash
# Unit testbenches via Makefiles
cd tb/voxel_bin_architecture && make test_unit
cd tb/gradient_map_architecture && make test_unit
cd tb/uart && make all
```

Waveforms are enabled by default for all cocotb runs. Each simulation writes an `.fst` trace under its `sim_build_*` directory, and you can open it with `make view_wave TEST=<name>`.

### Synthesize (devcontainer)

```bash
python3 setup.py synth voxel_bin      # -> synth/voxel_bin/voxel_bin_top.bit
python3 setup.py synth gradient_map   # -> synth/gradient_map/gradient_map_top.bit
```

### Flash and FPGA tools (devcontainer — local machine required)

The devcontainer cannot access USB devices, so flashing and any tool that communicates with the FPGA over UART must be done on your **local machine**.

1. After synthesizing inside the container, copy the bitstream to your local machine (VS Code's Explorer panel lets you right-click and download files, or use `scp`/`gh codespace cp`).
2. Follow the [local flash instructions](#4-flash-local) below using your local Python environment.

---

## Option 2 — Local setup

### Prerequisites

| Tool | Purpose | Install |
|------|---------|---------|
| Python 3.9+ | `setup.py`, testbenches | system package manager |
| OSS CAD Suite | Yosys, nextpnr, iceprog, Icarus Verilog | auto-downloaded by `setup.py`, or [manual](https://github.com/YosysHQ/oss-cad-suite-build/releases) |
| Git | clone | system package manager |

**Linux (Debian/Ubuntu)** — additional system libraries needed by OpenCV and iceprog:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip libgl1 libglib2.0-0 libusb-1.0-0
```

**macOS** — no extra system packages required beyond Xcode command-line tools.

**Windows** — [Git for Windows](https://git-scm.com/download/win) (provides `bash.exe` used by the Makefiles) and Python 3.9+ from [python.org](https://www.python.org/downloads/).

### 1. Clone and run setup

```bash
git clone https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier.git
cd FPGA-DVS-Gesture-Classifier
python3 setup.py        # creates .venv, installs packages, downloads OSS CAD Suite
```

`setup.py` with no arguments:
1. Creates a `.venv/` virtual environment
2. Installs all packages from `requirements.txt` (`cocotb`, `pytest`, `numpy`, `opencv-python-headless`, `pyserial`)
3. Downloads and extracts OSS CAD Suite for your platform (Windows x64, Linux x64/arm64, macOS x64/arm64) if it is not already present

If OSS CAD Suite is already installed system-wide (e.g. via Homebrew on macOS or a manual install on Linux), `setup.py` will detect and use it automatically without downloading anything.

### 2. Run tests (local)

The current verification flow is cocotb unit testbenches per module.

```bash
# Unit suites (POSIX shells, including WSL)
cd tb/voxel_bin_architecture && make test_unit
cd tb/gradient_map_architecture && make test_unit
cd tb/uart && make all

# Run one module testbench explicitly
cd tb/voxel_bin_architecture && make sim TEST=evt2_decoder
cd tb/gradient_map_architecture && make sim TEST=gradient_mapping
cd tb/uart && make sim TEST=uart_rx
```

```powershell
# PowerShell from repo root
make -C tb/voxel_bin_architecture test_unit
make -C tb/gradient_map_architecture test_unit
make -C tb/uart all

# One module testbench
make -C tb/voxel_bin_architecture sim TEST=evt2_decoder
make -C tb/gradient_map_architecture sim TEST=gradient_mapping
make -C tb/uart sim TEST=uart_rx
```

Waveforms are enabled by default for all cocotb runs. For each simulation:
- The trace file is `sim_build_<test>/<toplevel>.fst`.
- Open it with `make view_wave TEST=<name>` from the same testbench directory.

### 3. Synthesize (local)

```bash
python3 setup.py synth voxel_bin      # -> synth/voxel_bin/voxel_bin_top.bit
python3 setup.py synth gradient_map   # -> synth/gradient_map/gradient_map_top.bit

# Or via the synth Makefile
cd synth && make ARCH=gradient_map TARGET=raw
cd synth && make ARCH=voxel_bin    TARGET=raw
cd synth && make all_arch                    # both architectures
```

```powershell
# PowerShell from repo root
python setup.py synth voxel_bin
python setup.py synth gradient_map

# Or via Makefile
make -C synth ARCH=gradient_map TARGET=raw
make -C synth ARCH=voxel_bin TARGET=raw
make -C synth all_arch
```

### 4. Flash (local)

Connect the iCEBreaker FPGA via USB, then:

```bash
python3 setup.py flash voxel_bin
python3 setup.py flash gradient_map
```

**Windows**: run PowerShell as Administrator if `iceprog` reports an access error.

**Linux**: add your user to `plugdev` if `iceprog` reports a permission error:

```bash
sudo usermod -aG plugdev $USER   # then log out and back in
```

### Install Python packages only (optional)

If you prefer to manage the venv yourself:

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

---

## Common commands reference

| Command | Description |
|---------|-------------|
| `python3 setup.py` | Setup venv, install packages, detect/download OSS CAD Suite |
| `cd tb/voxel_bin_architecture && make test_unit` | Run voxel-bin unit testbenches |
| `cd tb/gradient_map_architecture && make test_unit` | Run gradient-map unit testbenches |
| `cd tb/uart && make all` | Run UART unit testbenches |
| `cd tb/voxel_bin_architecture && make view_wave TEST=core` | Open voxel-bin waveform in GTKWave |
| `cd tb/gradient_map_architecture && make view_wave TEST=core` | Open gradient-map waveform in GTKWave |
| `cd tb/uart && make view_wave TEST=uart_rx` | Open UART waveform in GTKWave |
| `python3 setup.py synth voxel_bin` | Synthesize → `voxel_bin_top.bit` |
| `python3 setup.py synth gradient_map` | Synthesize → `gradient_map_top.bit` |
| `python3 setup.py flash voxel_bin` | Flash voxel-bin bitstream |
| `python3 setup.py flash gradient_map` | Flash gradient-map bitstream |
| `python3 setup.py clean` | Remove all build artifacts |

---

## Verifying the flash

After flashing:

1. **Heartbeat LED** (`led_heartbeat`, pin 39) should blink at ~1.5 Hz.
2. **Validator tool** — sends synthetic gestures over UART and checks responses:
   ```bash
   python3 tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --arch gradient_map --test all
   # Windows: replace /dev/ttyUSB0 with COM3 (or whichever port the iCEBreaker enumerates on)
   ```
3. **DVS camera emulator** — uses a webcam to generate synthetic DVS events:
   ```bash
   python3 tools/dvs_camera_emulator.py --preview --max-events 5000 --port /dev/ttyUSB0 --no-noise --contrast 0.25
   ```

---

## Project layout

```
.
├── rtl/
│   ├── uart_rx.sv, uart_tx.sv, uart_debug.sv   # shared UART modules
│   ├── gradient_map_architecture/               # time-surface + systolic MAC
│   └── voxel_bin_architecture/                  # 3D histogram + systolic MAC
├── tb/
│   ├── gradient_map_architecture/               # cocotb tests + Makefile
│   └── voxel_bin_architecture/                  # cocotb tests + Makefile
├── synth/
│   ├── icebreaker.pcf                           # pin constraints (iCEBreaker)
│   ├── gradient_map/                            # synthesis outputs (gitignored)
│   └── voxel_bin/                               # synthesis outputs (gitignored)
├── tools/                                       # host-side Python utilities
├── .devcontainer/                               # VS Code Dev Container config
├── setup.py                                     # build/test/flash orchestrator
└── requirements.txt                             # Python dependencies
```

---

## Architecture overview

Both architectures are streaming, single-pass DVS pipelines on a single clock domain. They implement different feature representations but share the same UART I/O and iCEBreaker pin mapping.

| | Gradient-Map | Voxel-Bin |
|---|---|---|
| Feature | Decaying time-surface (16x16) | Voxel histogram (4 bins x 16x16) |
| Classifier | Systolic MAC over 256-cell surface | Systolic MAC over 1024-cell flattened bins |
| UART output | ASCII (`UP\r\n`, etc.) | Binary 2-byte gesture packet |
| Synthesis top | `gradient_map_top` | `voxel_bin_top` |
| Core module | `gradient_map_core` | `voxel_bin_core` |

---

## Gradient-Map architecture

**Dataflow**: UART RX byte stream -> word assembler in `gradient_map_top` -> `gradient_map_core` -> `input_fifo` -> `evt2_decoder` -> `gradient_mapping` (decaying surface) -> `gesture_classifier` + `systolic_array` -> `uart_debug` ASCII output + LEDs

Default grid is **16x16** (256 cells).

### Module map

| Module | File |
|--------|------|
| Top-level UART + LED wrapper | `rtl/gradient_map_architecture/gradient_map_top.sv` |
| Core pipeline wrapper | `rtl/gradient_map_architecture/gradient_map_core.sv` |
| EVT2 decoder + 320×320 → 16×16 downsample | `rtl/gradient_map_architecture/evt2_decoder.sv` |
| Input FIFO (128 × 32-bit BRAM) | `rtl/gradient_map_architecture/input_fifo.sv` |
| Decaying surface mapper | `rtl/gradient_map_architecture/gradient_mapping.sv` |
| Frame-based classifier | `rtl/gradient_map_architecture/gesture_classifier.sv` |
| Parallel MAC engine | `rtl/gradient_map_architecture/systolic_array.sv` |
| Per-class weight RAM/ROM | `rtl/gradient_map_architecture/weight_ram.sv` |
| ASCII UART output | `rtl/uart_debug.sv` |

### Configuration parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MIN_MASS_THRESH` | 2000 | Minimum accumulated energy to trigger gesture |
| `FRAME_PERIOD_MS` | 50 | Classification frame period (ms) |
| `DECAY_SHIFT` | 6 | Time-surface exponential decay rate |
| `BAUD_RATE` | 115200 | UART baud rate |


### How to test, synthesize, and flash

```bash
cd tb/gradient_map_architecture && make test_unit
python3 setup.py synth gradient_map
python3 setup.py flash gradient_map
```

---

## Voxel-Bin architecture

**Dataflow**: UART RX byte stream (with command bytes `0xFF/0xFE/0xFD/0xFC`) -> assembler/control in `voxel_bin_top` -> `voxel_bin_core` -> `input_fifo` -> `evt2_decoder` -> `voxel_binning` -> `systolic_array` + `weight_ram` -> `gesture_classifier` -> binary UART response + LEDs

Default is **4 temporal bins** (`NUM_BINS=4`) for iCE40 UP5K fit.

### Module map

| Module | File |
|--------|------|
| Top-level UART + command wrapper | `rtl/voxel_bin_architecture/voxel_bin_top.sv` |
| Core pipeline wrapper | `rtl/voxel_bin_architecture/voxel_bin_core.sv` |
| Event FIFO (32-bit EVT2 words) | `rtl/voxel_bin_architecture/input_fifo.sv` |
| EVT2 decoder + downsample | `rtl/voxel_bin_architecture/evt2_decoder.sv` |
| Temporal voxel binning | `rtl/voxel_bin_architecture/voxel_binning.sv` |
| Systolic MAC classifier front-end | `rtl/voxel_bin_architecture/systolic_array.sv` |
| Per-class weight RAM/ROM | `rtl/voxel_bin_architecture/weight_ram.sv` |
| Persistence + confidence logic | `rtl/voxel_bin_architecture/gesture_classifier.sv` |

### How to test, synthesize, and flash

```bash
cd tb/voxel_bin_architecture && make test_unit
python3 setup.py synth voxel_bin
python3 setup.py flash voxel_bin
```

---

## Testbench setup

Each architecture testbench directory uses the same pattern:

- `make test_unit`: run all module unit testbenches in that architecture.
- `make sim TEST=<name>`: run one module testbench.
- `make view_wave TEST=<name>`: open `sim_build_<name>/<toplevel>.fst` in GTKWave.
- `make clean_all`: remove `sim_build`, `results.xml`, and `__pycache__`.

Available `TEST=` targets:

| Directory | TEST values |
|-----------|-------------|
| `tb/gradient_map_architecture` | `input_fifo`, `evt2_decoder`, `gradient_mapping`, `systolic_array`, `gesture_classifier`, `weight_ram` |
| `tb/voxel_bin_architecture` | `input_fifo`, `evt2_decoder`, `voxel_binning`, `systolic_array`, `gesture_classifier`, `weight_ram` |
| `tb/uart` | `uart_rx`, `uart_tx`, `uart_debug` |

Examples:

```bash
cd tb/gradient_map_architecture && make sim TEST=evt2_decoder
cd tb/voxel_bin_architecture && make sim TEST=voxel_binning
cd tb/uart && make sim TEST=uart_debug
```

---

## UART protocol

**Configuration**: 115200 baud, 8N1

### Send events to FPGA (5 bytes per event)

| Byte | Contents |
|------|----------|
| 0 | X_HI: `X[8]` (bit 0) |
| 1 | X_LO: `X[7:0]` |
| 2 | Y_HI: `Y[8]` (bit 0) |
| 3 | Y_LO: `Y[7:0]` |
| 4 | POL: polarity (0=OFF, 1=ON) |

X, Y coordinates range from 0–319.

### Receive from FPGA — gradient_map_top (ASCII)

| Gesture | Output |
|---------|--------|
| UP | `UP\r\n` |
| DOWN | `DOWN\r\n` |
| LEFT | `LEFT\r\n` |
| RIGHT | `RIGHT\r\n` |

### Receive from FPGA — voxel_bin_top (binary)

**Gesture response** (2 bytes):

| Byte | Contents |
|------|----------|
| 0 | `0xA0 \| gesture` (0=UP, 1=DOWN, 2=LEFT, 3=RIGHT) |
| 1 | `confidence[3:0] \| event_count_hi[3:0]` |

**Status response** (1 byte): `0xBx`

- Bits [4:2]: FSM state
- Bit 1: FIFO full
- Bit 0: FIFO empty

### Special commands (voxel_bin only)

| Send | Response | Description |
|------|----------|-------------|
| `0xFF` | `0x55` | Echo test |
| `0xFE` | `0xBx` | Status query |
| `0xFD` | 2 bytes | Config query |
| `0xFC` | (none) | Soft reset |

---

## Hardware validation pipeline (GenX320 + STM32 + iCEBreaker)

End-to-end validation using a live DVS sensor.

**Setup**: Prophesee GenX320 → STM32 board → USB CDC → host machine → UART → iCEBreaker FPGA

### Step 1: Build and flash

```bash
python3 setup.py synth gradient_map
python3 setup.py flash gradient_map
```

### Step 2: Capture a raw EVT stream

```bash
python3 tools/capture_evt_stream.py /dev/ttyACM0 gesture_trial.bin --duration 10
# Windows: replace /dev/ttyACM0 with the STM32 COM port (e.g. COM5)
```

### Step 3a: Replay captured file to FPGA

```bash
python3 tools/replay_evt_to_fpga.py --file gesture_trial.bin --fpga /dev/ttyUSB0
# Windows: --fpga COM3
```

### Step 3b: Live relay (sensor → FPGA)

```bash
python3 tools/replay_evt_to_fpga.py --dvs /dev/ttyACM0 --fpga /dev/ttyUSB0
# Optionally save while relaying:
python3 tools/replay_evt_to_fpga.py --dvs /dev/ttyACM0 --fpga /dev/ttyUSB0 --save capture.bin --duration 30
```

### EVT2 bit layout override

If the GenX320 firmware uses non-standard packing:

```bash
python3 tools/replay_evt_to_fpga.py --file gesture_trial.bin --fpga /dev/ttyUSB0 \
    --x-shift 17 --y-shift 6 --swap-xy
```

### FPGA outputs

- ASCII gesture labels over UART TX: `UP\r\n`, `DOWN\r\n`, `LEFT\r\n`, `RIGHT\r\n`
- Direction LEDs: `led_up` (pin 26), `led_down` (pin 27), `led_left` (pin 25), `led_right` (pin 23)
- `led_heartbeat` (pin 39): ~1.5 Hz blink
- `led_activity` (pin 40): pulses on event receive

### Bandwidth note

At 115200 baud the UART delivers ~2300 events/sec (5 bytes each). The GenX320 may produce more; the relay throttles naturally. Classification still works at reduced rates because the time-surface accumulates over the `FRAME_PERIOD_MS` window.

---

## Tools

### DVS camera emulator (webcam → synthetic DVS events)

```bash
python3 tools/dvs_camera_emulator.py --simulate --preview
python3 tools/dvs_camera_emulator.py --port /dev/ttyUSB0 --preview --max-events 5000 --no-noise --contrast 0.25
```

### FPGA hardware validator (synthetic gestures over UART)

```bash
python3 tools/fpga_gesture_validator.py --list-ports
python3 tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --arch gradient_map --test all
python3 tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --arch voxel_bin --test all
python3 tools/fpga_gesture_validator.py --port /dev/ttyUSB0 --interactive
```

### Raw event capture

```bash
python3 tools/capture_evt_stream.py /dev/ttyACM0 trial_run.bin --duration 10
```

### EVT2 replay / live relay

```bash
python3 tools/replay_evt_to_fpga.py --file aligned.bin --fpga /dev/ttyUSB0
python3 tools/replay_evt_to_fpga.py --dvs /dev/ttyACM0 --fpga /dev/ttyUSB0
python3 tools/replay_evt_to_fpga.py --list-ports
```

---

## iCE40 UP5K resource fit

Both architectures are tuned to fit the Lattice iCE40 UP5K (≈5280 LUT4s, 30×4 Kbit BRAM):

- **Gradient-map**: 16×16 grid (256 cells), 128-entry input FIFO, time-surface and weight ROMs sized for 256 cells; `flatten_buffer` removed from build (classifier streams directly from BRAM).
- **Voxel-bin**: 4 temporal bins, 16×16 grid, 1024-cell feature vector and 4x1024 weight ROMs.
