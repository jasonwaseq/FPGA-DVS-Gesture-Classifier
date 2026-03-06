# FPGA DVS Gesture Classifier

Cross-platform workflow for simulation, synthesis, and flashing with one top-level CLI.

## Top-level commands

From repo root:

```bash
python setup.py setup
python setup.py doctor
python setup.py test
python setup.py synth voxel_bin
python setup.py flash voxel_bin
```

`setup.py` is the canonical entrypoint on Windows, Linux, and macOS.

## What `setup` installs/configures

`python setup.py setup`:

1. Creates `.venv`
2. Installs Python packages from `requirements.txt`:
   - `cocotb`
   - `pytest`
   - `numpy`
   - `opencv-python`
   - `pyserial`
3. Detects or installs OSS CAD Suite tools (`iverilog`, `vvp`, `yosys`, `nextpnr-ice40`, `icepack`, `iceprog`)

Use `python setup.py setup --skip-fpga` if you only want the Python environment.

## Doctor / health check

Run:

```bash
python setup.py doctor
```

It verifies:

- `.venv` exists
- required Python modules import correctly
- OSS CAD tools are discoverable
- `synth/icebreaker.pcf` is present

## Test workflow

Run all cocotb benches:

```bash
python setup.py test
```

Run one bench:

```bash
python setup.py test evt2_decoder
python setup.py test input_fifo
python setup.py test voxel_bin_core
python setup.py test voxel_bin_top
```

Supported test targets:

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

- `python setup.py test all`
- `python setup.py test unit`
- `python setup.py test core`
- `python setup.py test top`

## Synthesis and flash

Synthesize:

```bash
python setup.py synth voxel_bin
```

Output bitstream:

- `synth/voxel_bin/voxel_bin_top.bit`

Flash:

```bash
python setup.py flash voxel_bin
```

Optional flash flags:

```bash
python setup.py flash voxel_bin --port COM3 --serial ABC123 --vid 0x0403 --pid 0x6010
```

Notes:

- Current flashing path uses `iceprog`.
- On Windows, run elevated PowerShell if USB permission errors occur.
- In devcontainers/codespaces, flashing usually fails due no USB passthrough.

## Cleanup

Remove generated test/synthesis artifacts:

```bash
python setup.py clean
```

## Command reference

```bash
python setup.py setup [--skip-fpga]
python setup.py doctor
python setup.py test [all|<test_target>]
python setup.py synth [voxel_bin]
python setup.py flash [voxel_bin] [--port ... --serial ... --vid ... --pid ...]
python setup.py clean
```

