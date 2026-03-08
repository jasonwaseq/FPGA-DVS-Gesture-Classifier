# FPGA DVS Gesture Classifier

Real-time  gesture classification running on a Lattice iCE40UP5K FPGA. A Prophesee GenX320 event camera streams pixel-change events over UART; the FPGA classifies them into one of four gestures (Down, Left, Right, Up) and reports the result back. 

## Architecture and Dataflow

```
Camera (EVT2.0 over USB)
  └─► evt_stream_to_fpga.py  ← host-side relay script
        └─► UART (115200 baud)
              └─► voxel_bin_top  ← synthesis top-level
                    ├─ uart_rx + 4-byte assembler
                    └─► input_fifo
                          └─► evt2_decoder       (decode + 320×320 → 8×8)
                                └─► voxel_binning (4 bins × 8×8 counters → 256 features)
                                      └─► systolic_array  (4×4 tiled GEMV × 64 tiles)
                                            ├─ weight RAMs (4 classes × 256 weights)
                                            └─► gesture_classifier  (argmax + persistence)
                                                  └─► uart_tx + LEDs
```

**`input_fifo`** — Buffers decoded words between the UART assembler and the decoder with a valid/ready handshake.

**`evt2_decoder`** — Parses 32-bit EVT2.0 words from the camera. Pixel events (`CD_ON`/`CD_OFF`) extract `(x, y, polarity)`; `TIME_HIGH` words update the timestamp. Compresses the 320×320 sensor down to an 8×8 grid.

**`voxel_binning`** — Accumulates events into a ring of 4 time bins, each 250 ms wide (1 s total window). Each bin is an 8×8 grid of 4-bit saturating counters. When a bin expires, all 4 bins are read out as a flat 256-element feature vector.

**`systolic_array`** — A 4×4 array of multiply-accumulate processing elements. Computes a dot product of 4 features against 4 weights per invocation in 10 clock cycles.

**`gesture_classifier`** — Takes the 4 scores, picks the highest (argmax), and requires 2 consecutive windows with the same winner before declaring a gesture valid.

**`voxel_bin_core`** — Wires the pipeline together and runs a tiled GEMV loop: multiplies the 256-feature vector against a 4×256 weight matrix (one row per gesture class) using the systolic array in 64 tiles of 4, accumulating 4 class scores.

**`voxel_bin_top`** — Synthesis top-level. Wraps the core with UART RX/TX, a power-on reset, LEDs, and a single-byte command parser (ping, status, config, diagnostics, soft reset).

---

Cross-platform workflow for setup, verification, synthesis, and flash using a top-level `Makefile`.

## Top-level workflow

From repo root:

```bash
make setup
make doctor
make test
make synth
make flash
```

Run `make help` to see full usage and variables.

## What `make setup` does

`make setup` calls the project bootstrap flow and:

1. Creates a platform-specific venv:
   - PowerShell/Windows: `.venv-win`
   - WSL: `.venv-wsl`
   - Native Linux: `.venv-linux`
   - macOS: `.venv-macos`
2. Installs required Python packages:
   - `cocotb`
   - `cocotb-test`
   - `pytest`
   - `gitpython`
   - `numpy`
   - `opencv-python`
   - `pyserial`
3. Detects or installs OSS CAD Suite tools:
   - `iverilog`, `vvp`, `yosys`, `nextpnr-ice40`, `icepack`, `iceprog`

If you only want Python dependencies:

```bash
make setup SKIP_FPGA=1
```

## Doctor

```bash
make doctor
```

Checks:
- current platform venv exists
- required Python modules import
- OSS CAD tools are discoverable
- `iverilog/vvp` smoke test
- `synth/icebreaker.pcf` exists

Prerequisites:
- GNU Make must be installed and available as `make`.
- Python with `venv` support must be available (`python -m venv`).
- On WSL/Linux, install `python3-venv` and `python3-dev` if venv creation or cocotb/libpython loading fails.

## Test

All benches:

```bash
make test
```

Single bench:

```bash
make test input_fifo
make test evt2_decoder
make test voxel_bin_core
make test voxel_bin_top
```

Supported module targets for `make test <module_name>`:
- `all` (default)
- `evt2_decoder`
- `gesture_classifier`
- `input_fifo`
- `ram_1r1w_sync`
- `systolic_array`
- `uart_rx`
- `uart_tx`
- `uart_debug`
- `voxel_binning`
- `voxel_bin_core`
- `voxel_bin_top`

Aliases:
- `make test unit`
- `make test core`
- `make test top`

## Synthesis

```bash
make synth
```

Output:
- `synth/voxel_bin_top.bit`

## Flash

```bash
make flash
```

Optional flash arguments:

```bash
make flash PORT=COM3 SERIAL=ABC123 VID=0x0403 PID=0x6010
```

Notes:
- Flash uses `iceprog`.
- On Windows, use elevated PowerShell if USB permission errors occur.
- In devcontainers/codespaces, flashing usually fails due no USB passthrough.

## Clean

```bash
make clean
```
