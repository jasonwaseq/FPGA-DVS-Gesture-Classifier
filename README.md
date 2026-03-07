# FPGA DVS Gesture Classifier

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
- `synth/voxel_bin/voxel_bin_top.bit`

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

## Notes

- The top-level workflow is intentionally `make`-first for consistency across setup, verification, synthesis, and flashing.
