#!/usr/bin/env python3
"""
DVS Gesture Accelerator - Universal Setup Script
Works on Windows, Linux, and macOS

Usage:
    python setup.py                       # Full setup (venv + dependencies + FPGA tools)
    python setup.py --skip-fpga            # Skip OSS CAD Suite download
    python setup.py test [arch]            # Run cocotb tests (arch: voxel_bin | gradient_map)
    python setup.py synth <arch>           # Synthesize for iCE40 (arch: voxel_bin | gradient_map)
    python setup.py flash <arch>           # Flash bitstream (arch: voxel_bin | gradient_map)
    python setup.py clean                 # Clean build artifacts
"""

import os
import sys
import platform
import subprocess
import shutil
import urllib.request
import tarfile
import tempfile
from pathlib import Path

# =============================================================================
# Configuration
# =============================================================================

PROJECT_ROOT = Path(__file__).parent.resolve()
VENV_PATH = PROJECT_ROOT / ".venv"
OSS_CAD_PATH = PROJECT_ROOT / "oss-cad-suite"
RTL_DIR = PROJECT_ROOT / "rtl"
TB_DIR = PROJECT_ROOT / "tb"
SYNTH_DIR = PROJECT_ROOT / "synth"

# Python packages needed
PYTHON_PACKAGES = [
    "cocotb",
    "pytest", 
    "numpy",
    "opencv-python",
    "pyserial",
]

# RTL source files in dependency order (voxel-bin architecture)
RTL_FILES = [
    "uart_rx.sv",
    "uart_tx.sv",
    "voxel_bin_architecture/InputFIFO.sv",
    "voxel_bin_architecture/SpatialCompressor.sv",
    "voxel_bin_architecture/TemporalAccumulator.sv",
    "voxel_bin_architecture/MotionComputer.sv",
    "voxel_bin_architecture/GestureClassifier.sv",
    "voxel_bin_architecture/OutputRegister.sv",
    "voxel_bin_architecture/TimeSurfaceBinning.sv",
    "voxel_bin_architecture/WeightROM.sv",
    "voxel_bin_architecture/SystolicMatrixMultiply.sv",
    "voxel_bin_architecture/dvs_gesture_accel.sv",
    "voxel_bin_architecture/voxel_bin_top.sv",
]

# RTL files for Gradient-Map gesture architecture (gesture_top)
RTL_FILES_GESTURE = [
    "uart_tx.sv",
    "uart_rx.sv",
    "uart_debug.sv",
    "gradient_map_architecture/input_fifo.sv",
    "gradient_map_architecture/evt2_decoder.sv",
    "gradient_map_architecture/time_surface_memory.sv",
    "gradient_map_architecture/flatten_buffer.sv",
    "gradient_map_architecture/weight_rom.sv",
    "gradient_map_architecture/systolic_array.sv",
    "gradient_map_architecture/spatio_temporal_classifier.sv",
    "gradient_map_architecture/time_surface_encoder.sv",
    "gradient_map_architecture/gesture_top.sv",
]

# RTL files for UART validation mode (gradient_map_top wraps gesture_top)
RTL_FILES_UART = RTL_FILES_GESTURE + [
    "gradient_map_architecture/gradient_map_top.sv",
]

# OSS CAD Suite download URLs (2024-11-21 release)
OSS_CAD_URLS = {
    ("Windows", "AMD64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-windows-x64-20241121.exe",
    ("Linux", "x86_64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz",
    ("Linux", "aarch64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-arm64-20241121.tgz",
    ("Darwin", "x86_64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-darwin-x64-20241121.tgz",
    ("Darwin", "arm64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-darwin-arm64-20241121.tgz",
}

# =============================================================================
# Utility Functions
# =============================================================================

def print_header(msg):
    print(f"\n{'='*60}")
    print(f"  {msg}")
    print(f"{'='*60}")

def print_step(step, msg):
    print(f"\n[{step}] {msg}")

def print_success(msg):
    print(f"  [+] {msg}")

def print_warning(msg):
    print(f"  [!] {msg}")

def print_error(msg):
    print(f"  [x] {msg}")

def get_platform_info():
    """Get OS and architecture."""
    system = platform.system()
    machine = platform.machine()
    return system, machine

def get_python_cmd():
    """Get the python command for this platform."""
    if sys.platform == "win32":
        return VENV_PATH / "Scripts" / "python.exe"
    return VENV_PATH / "bin" / "python"

def get_pip_cmd():
    """Get the pip command for this platform."""
    if sys.platform == "win32":
        return VENV_PATH / "Scripts" / "pip.exe"
    return VENV_PATH / "bin" / "pip"

def get_oss_cad_bin():
    """Get path to OSS CAD Suite binaries."""
    # Check common user locations first (prefer system-wide installations)
    common_paths = [
        Path.home() / "Documents" / "oss-cad-suite",
        Path.home() / "Documents" / "oss-cad-suite-windows",
        Path.home() / "oss-cad-suite",
        Path("C:/oss-cad-suite"),
        Path.home() / "Documents" / "oss-cad-suite" / "oss-cad-suite",
    ]
    
    for p in common_paths:
        if p.exists() and (p / "bin").exists():
            # Verify it has the key tools
            if (p / "bin" / "iverilog.exe" if sys.platform == "win32" else p / "bin" / "iverilog").exists():
                return p
    
    # Check project-local installation last (nested structure on Windows)
    if OSS_CAD_PATH.exists():
        nested = OSS_CAD_PATH / "oss-cad-suite"
        if nested.exists() and (nested / "bin").exists():
            if (nested / "bin" / "iverilog.exe" if sys.platform == "win32" else nested / "bin" / "iverilog").exists():
                return nested
        if (OSS_CAD_PATH / "bin").exists():
            if (OSS_CAD_PATH / "bin" / "iverilog.exe" if sys.platform == "win32" else OSS_CAD_PATH / "bin" / "iverilog").exists():
                return OSS_CAD_PATH
    
    return None

def get_oss_cad_env():
    """Get environment with OSS CAD Suite properly initialized."""
    oss_root = get_oss_cad_bin()
    if oss_root is None:
        return None
    
    env = os.environ.copy()
    oss_bin = oss_root / "bin"
    oss_lib = oss_root / "lib"
    
    # Set PATH with bin and lib (for Windows DLLs)
    env["PATH"] = f"{oss_bin}{os.pathsep}{oss_lib}{os.pathsep}{env.get('PATH', '')}"
    
    # Set OSS CAD Suite environment variables (from environment.bat)
    env["YOSYSHQ_ROOT"] = str(oss_root) + os.sep
    if (oss_root / "etc" / "cacert.pem").exists():
        env["SSL_CERT_FILE"] = str(oss_root / "etc" / "cacert.pem")
    env["PYTHON_EXECUTABLE"] = str(oss_lib / "python3.exe") if sys.platform == "win32" else str(oss_lib / "python3")
    env["QT_PLUGIN_PATH"] = str(oss_lib / "qt5" / "plugins")
    env["QT_LOGGING_RULES"] = "*=false"
    env["GTK_EXE_PREFIX"] = str(oss_root)
    env["GTK_DATA_PREFIX"] = str(oss_root)
    env["GDK_PIXBUF_MODULEDIR"] = str(oss_lib / "gdk-pixbuf-2.0" / "2.10.0" / "loaders")
    env["GDK_PIXBUF_MODULE_FILE"] = str(oss_lib / "gdk-pixbuf-2.0" / "2.10.0" / "loaders.cache")
    env["OPENFPGALOADER_SOJ_DIR"] = str(oss_root / "share" / "openFPGALoader")
    
    return env

def run_cmd(cmd, env=None, cwd=None, check=True):
    """Run a command and return the result."""
    if isinstance(cmd, str):
        cmd = cmd.split()
    result = subprocess.run(cmd, env=env, cwd=cwd, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(f"Command failed: {' '.join(str(c) for c in cmd)}")
        print(result.stderr)
        return None
    return result

def download_with_progress(url, dest):
    """Download a file with progress indication."""
    print(f"  Downloading from: {url}")
    print(f"  This may take several minutes...")
    
    try:
        urllib.request.urlretrieve(url, dest)
        return True
    except Exception as e:
        print_error(f"Download failed: {e}")
        return False

# =============================================================================
# Setup Functions
# =============================================================================

def setup_venv():
    """Create Python virtual environment if it doesn't exist."""
    print_step("1/3", "Setting up Python virtual environment...")
    
    if VENV_PATH.exists():
        print_success("Virtual environment already exists")
        return True
    
    result = run_cmd([sys.executable, "-m", "venv", str(VENV_PATH)])
    if result is None:
        print_error("Failed to create virtual environment")
        return False
    
    print_success(f"Created virtual environment at {VENV_PATH.name}")
    return True

def install_packages():
    """Install Python packages in the virtual environment."""
    print_step("2/3", "Installing Python packages...")
    
    pip = get_pip_cmd()
    if not pip.exists():
        print_error("pip not found in virtual environment")
        return False
    
    # Upgrade pip
    run_cmd([str(pip), "install", "--upgrade", "pip"], check=False)
    
    # Install packages
    result = run_cmd([str(pip), "install"] + PYTHON_PACKAGES)
    if result is None:
        print_error("Failed to install packages")
        return False
    
    print_success(f"Installed: {', '.join(PYTHON_PACKAGES)}")
    return True

def setup_oss_cad_suite():
    """Download and install OSS CAD Suite."""
    print_step("3/3", "Setting up OSS CAD Suite (FPGA tools)...")
    
    # Check if already available (local or system-wide)
    existing = get_oss_cad_bin()
    if existing:
        print_success(f"OSS CAD Suite found at {existing}")
        return True
    
    system, machine = get_platform_info()
    url_key = (system, machine)
    url_key = (system, machine)
    
    if url_key not in OSS_CAD_URLS:
        print_warning(f"No pre-built OSS CAD Suite for {system}/{machine}")
        print("  Install manually from: https://github.com/YosysHQ/oss-cad-suite-build/releases")
        if system == "Darwin":
            print("  Or use Homebrew: brew install yosys nextpnr icarus-verilog")
        return False
    
    url = OSS_CAD_URLS[url_key]
    
    with tempfile.TemporaryDirectory() as tmpdir:
        if system == "Windows":
            # Windows uses self-extracting exe
            installer = Path(tmpdir) / "oss-cad-suite.exe"
            if not download_with_progress(url, installer):
                return False
            
            print("  Extracting (this takes a while)...")
            result = subprocess.run(
                [str(installer), f"-o{OSS_CAD_PATH}", "-y"],
                capture_output=True
            )
            if result.returncode != 0:
                print_error("Extraction failed")
                return False
        else:
            # Linux/macOS uses tar.gz
            archive = Path(tmpdir) / "oss-cad-suite.tgz"
            if not download_with_progress(url, archive):
                return False
            
            print("  Extracting...")
            with tarfile.open(archive, "r:gz") as tar:
                tar.extractall(PROJECT_ROOT)
    
    print_success("OSS CAD Suite installed successfully")
    return True

# =============================================================================
# Test Functions
# =============================================================================

# Architecture config for tests: (rtl_files, toplevel, tb_dir, defines, param_overrides)
ARCH_TEST_CONFIG = {
    "voxel_bin": (RTL_FILES, "voxel_bin_top", TB_DIR / "voxel_bin_architecture", ["-DCLKS_PER_BIT=4", "-DMIN_EVENT_THRESH=4", "-DMOTION_THRESH=2"], ["-Pvoxel_bin_top.CYCLES_PER_BIN=600"]),
    "gradient_map": (RTL_FILES_GESTURE, "gesture_top", TB_DIR / "gradient_map_architecture", [], ["-Pgesture_top.FRAME_PERIOD_MS=1", "-Pgesture_top.MIN_MASS_THRESH=20", "-Pgesture_top.UART_RX_ENABLE=1"]),
}


def run_tests(test_module=None):
    """Run cocotb verification tests. test_module: test_voxel_bin | test_gradient_map | <other>."""
    print_header("Running cocotb Verification Tests")

    # Map test module to architecture
    if test_module in ("test_voxel_bin", "voxel_bin"):
        arch = "voxel_bin"
        test_module = "test_voxel_bin"
    elif test_module in ("test_gradient_map", "gradient_map"):
        arch = "gradient_map"
        test_module = "test_gradient_map"
    else:
        # Default to voxel_bin for backward compatibility
        arch = "voxel_bin"
        test_module = test_module or "test_voxel_bin"

    rtl_files, toplevel, tb_dir, defines, param_overrides = ARCH_TEST_CONFIG[arch]
    
    # Prefer system Icarus to avoid OSS CAD Suite GLIBC conflicts
    system_iverilog = Path("/usr/bin/iverilog")
    system_vvp = Path("/usr/bin/vvp")
    use_system_iverilog = system_iverilog.exists() and system_vvp.exists()
    if use_system_iverilog:
        env = os.environ.copy()
        iverilog_cmd = str(system_iverilog)
        vvp_cmd = str(system_vvp)
    else:
        oss_root = get_oss_cad_bin()
        if oss_root is None:
            print_error("OSS CAD Suite not found. Run 'python setup.py' first.")
            return 1
        env = get_oss_cad_env()
        if env is None:
            print_error("Failed to set up OSS CAD Suite environment")
            return 1
        iverilog_cmd = "iverilog"
        vvp_cmd = "vvp"
    
    python = get_python_cmd()
    if not python.exists():
        print_error("Python venv not found. Run 'python setup.py' first.")
        return 1
    env["PYGPI_PYTHON_BIN"] = str(python)
    
    # Get cocotb libs path
    result = run_cmd(
        [str(python), "-c", "import cocotb, os; print(os.path.dirname(cocotb.__file__))"],
        env=env
    )
    if result is None:
        print_error("cocotb not installed. Run 'python setup.py' first.")
        return 1
    cocotb_path = Path(result.stdout.strip())
    cocotb_libs = cocotb_path / "libs"
    vpi_module = cocotb_libs / "cocotbvpi_icarus.vpi"
    vpi_fallback = cocotb_libs / "libcocotbvpi_icarus.vpl"
    if not vpi_module.exists() and vpi_fallback.exists():
        # Some cocotb wheels ship the Icarus VPI as .vpl; vvp expects .vpi
        try:
            vpi_module.symlink_to(vpi_fallback.name)
        except OSError:
            shutil.copyfile(vpi_fallback, vpi_module)
    
    # Set up cocotb environment
    env["COCOTB_TEST_MODULES"] = test_module
    env["TOPLEVEL"] = toplevel
    env["TOPLEVEL_LANG"] = "verilog"
    env["PYTHONPATH"] = str(tb_dir)
    if sys.platform.startswith("linux") and not use_system_iverilog:
        # Keep system glibc ahead when forced to use OSS CAD Suite Icarus
        env["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
    
    # Python lib for cocotb
    if sys.platform == "win32":
        # Find python DLL
        python_version = f"{sys.version_info.major}{sys.version_info.minor}"
        dll_name = f"python{python_version}.dll"
        env["LIBPYTHON_LOC"] = str(Path(sys.base_prefix) / dll_name)
        env["PYGPI_PYTHON_BIN"] = str(python)
    
    # Create sim_build directory (per-architecture)
    sim_build = tb_dir / "sim_build"
    sim_build.mkdir(parents=True, exist_ok=True)
    
    # Compile RTL with iverilog
    print("\nCompiling RTL with Icarus Verilog...")
    print(f"  Toplevel: {toplevel}")
    print(f"  Test module: {test_module}")
    vvp_file = sim_build / "sim.vvp"
    sources = [str(RTL_DIR / f) for f in rtl_files]
    
    compile_cmd = [
        iverilog_cmd, "-g2012",
        "-s", toplevel,
        "-o", str(vvp_file),
    ] + defines + param_overrides + sources
    
    result = subprocess.run(compile_cmd, env=env, cwd=sim_build, capture_output=True, text=True)
    if result.returncode != 0:
        print_error("Compilation failed")
        if result.stderr:
            print(result.stderr)
        if result.stdout:
            print(result.stdout)
        return 1
    print_success("Compilation successful")
    
    # Run simulation
    print("\nRunning simulation...")
    sim_cmd = [
        vvp_cmd,
        "-M", str(cocotb_libs),
        "-m", "cocotbvpi_icarus",
        str(vvp_file)
    ]
    
    result = subprocess.run(sim_cmd, env=env, cwd=tb_dir)
    
    print_header("Tests Complete" if result.returncode == 0 else "Tests Failed")
    return result.returncode

# =============================================================================
# Synthesis Functions
# =============================================================================

def _synthesize(top_module, rtl_files, pcf_name, label="", arch_dir=None):
    """Generic synthesis for iCE40 UP5K.

    Args:
        top_module: Top-level module name (e.g. 'gradient_map_top')
        rtl_files: List of RTL filenames relative to RTL_DIR
        pcf_name: PCF filename inside SYNTH_DIR
        label: Human-readable label for log messages
    """
    display = label or top_module
    print_header(f"Synthesizing {display} for iCE40 UP5K")

    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found. Run 'python setup.py' first.")
        return 1

    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1

    sources = [str(RTL_DIR / f) for f in rtl_files]

    # Organize synthesis artifacts under per-architecture subdirectories.
    output_root = SYNTH_DIR / arch_dir if arch_dir else SYNTH_DIR
    output_root.mkdir(parents=True, exist_ok=True)

    output_json = output_root / f"{top_module}.json"
    output_asc = output_root / f"{top_module}.asc"
    output_bit = output_root / f"{top_module}.bit"

    # PCF files remain at the top level of SYNTH_DIR for reuse.
    pcf_file = SYNTH_DIR / pcf_name

    if not pcf_file.exists():
        print_error(f"PCF file not found: {pcf_file}")
        return 1

    # Synthesis with Yosys
    print(f"\nRunning Yosys synthesis (top={top_module})...")
    yosys_cmd = (
        f"read_verilog -sv {' '.join(sources)}; "
        f"synth_ice40 -top {top_module} -json {output_json}"
    )
    result = subprocess.run(["yosys", "-p", yosys_cmd], env=env, cwd=SYNTH_DIR)
    if result.returncode != 0:
        print_error("Synthesis failed")
        return 1
    print_success("Synthesis complete")

    # Place and Route with nextpnr
    print("\nRunning nextpnr place and route...")
    result = subprocess.run([
        "nextpnr-ice40",
        "--up5k", "--package", "sg48",
        "--json", str(output_json),
        "--pcf", str(pcf_file),
        "--asc", str(output_asc),
        "--freq", "12"
    ], env=env, cwd=SYNTH_DIR)
    if result.returncode != 0:
        print_error("Place and route failed")
        return 1
    print_success("Place and route complete")

    # Generate bitstream
    print("\nGenerating bitstream...")
    result = subprocess.run(
        ["icepack", str(output_asc), str(output_bit)], 
        env=env, 
        capture_output=True, 
        text=True
    )
    if result.returncode != 0:
        print_error("Bitstream generation failed")
        if result.stderr:
            print(f"  Error: {result.stderr.strip()}")
        if result.stdout:
            print(f"  Output: {result.stdout.strip()}")
        # Check if design is over capacity (common cause of icepack failures)
        print_warning("Note: If design exceeds FPGA capacity, bitstream generation may fail")
        print_warning("      even if place and route completes. Check resource utilization.")
        return 1
    print_success(f"Bitstream created: {output_bit}")

    print_header("Synthesis Complete")
    return 0


# Architecture config for synthesis:
# (top_module, rtl_files, pcf_name, label, subdir)
ARCH_SYNTH_CONFIG = {
    "voxel_bin": (
        "voxel_bin_top",
        RTL_FILES,
        "icebreaker.pcf",
        "Voxel-bin (voxel_bin_top)",
        "voxel_bin",
    ),
    "gradient_map": (
        "gradient_map_top",
        RTL_FILES_UART,
        "icebreaker.pcf",
        "Gradient-map UART (gradient_map_top)",
        "gradient_map",
    ),
}


def run_synthesis(arch):
    """Synthesize design for iCE40 UP5K. arch: voxel_bin | gradient_map."""
    if arch not in ARCH_SYNTH_CONFIG:
        print_error(f"Unknown architecture: {arch}. Use voxel_bin or gradient_map.")
        return 1

    top_module, rtl_files, pcf_name, label, subdir = ARCH_SYNTH_CONFIG[arch]
    return _synthesize(
        top_module=top_module,
        rtl_files=rtl_files,
        pcf_name=pcf_name,
        label=label,
        arch_dir=subdir,
    )

def list_ftdi_devices():
    """List FTDI devices suitable for FPGA programming."""
    devices = []
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Look for FTDI devices (0x0403) commonly used in iCE40 boards
            if port.vid == 0x0403 and port.pid in [0x6010, 0x6014]:
                devices.append({
                    'port': port.device,
                    'vid': port.vid,
                    'pid': port.pid,
                    'serial': port.serial_number,
                    'description': port.description
                })
    except:
        pass
    return devices

def find_ftdi_device(preferred_port=None, preferred_serial=None, preferred_vid=None, preferred_pid=None):
    """Find FTDI device info for FPGA programming, honoring user preference."""
    devices = list_ftdi_devices()

    if preferred_port:
        for dev in devices:
            if dev['port'].lower() == preferred_port.lower():
                return dev

    if preferred_serial:
        for dev in devices:
            if dev['serial'] and dev['serial'].lower() == preferred_serial.lower():
                return dev

    if preferred_vid is not None and preferred_pid is not None:
        for dev in devices:
            if dev['vid'] == preferred_vid and dev['pid'] == preferred_pid:
                return dev

    return devices[0] if devices else None

def flash_fpga(port=None, serial=None, vid=None, pid=None, bitfile_name=None, arch=None):
    """Flash bitstream to FPGA. arch: voxel_bin | gradient_map."""
    print_header("Flashing FPGA")

    # Resolve bitfile from architecture
    if bitfile_name is None and arch is not None:
        bitfile_name = "voxel_bin_top.bit" if arch == "voxel_bin" else "gradient_map_top.bit"
    if bitfile_name is None:
        bitfile_name = "voxel_bin_top.bit"

    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found")
        return 1
    
    # Look for the bitfile under the architecture's synthesis subdirectory
    if arch is not None and arch in ARCH_SYNTH_CONFIG:
        _, _, _, _, subdir = ARCH_SYNTH_CONFIG[arch]
        bitfile_dir = SYNTH_DIR / subdir
    else:
        bitfile_dir = SYNTH_DIR

    bitfile = bitfile_dir / bitfile_name
    if not bitfile.exists():
        print_error(f"Bitstream not found: {bitfile}")
        print("  Run 'python setup.py synth voxel_bin' or 'python setup.py synth gradient_map' first")
        return 1
    
    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1
    
    if any([port, serial, vid, pid]):
        print_warning("Port/serial/VID/PID options are ignored when using iceprog.")

    # Find FTDI device (best-effort info only for user feedback)
    device_info = find_ftdi_device(
        preferred_port=port,
        preferred_serial=serial,
        preferred_vid=vid,
        preferred_pid=pid
    )

    if device_info:
        print(f"Found FPGA: {device_info['description']}")
        print(f"  Port: {device_info['port']}")
        print(f"  USB ID: {device_info['vid']:04x}:{device_info['pid']:04x}")
        if device_info['serial']:
            print(f"  Serial: {device_info['serial']}")
    else:
        print_warning("No FTDI device detected")
    
    print(f"\nFlashing {bitfile.name}...")
    
    # Get tool paths
    iceprog_available = shutil.which("iceprog", path=str(oss_root / "bin"))

    # Method 1: Try iceprog default
    if iceprog_available:
        print("Method 1: iceprog (default)...")
        result = subprocess.run(
            ["iceprog", str(bitfile)],
            env=env,
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print_success("FPGA programmed successfully!")
            return 0
        print_warning("Failed")
    
    # All methods failed
    print_error("\nAll programming methods failed")
    print("\nQuick fixes to try:")
    print("  1. Close this terminal and run PowerShell as Administrator, then retry")
    print("  2. Unplug the FPGA, wait 3 seconds, plug it back in, then retry")
    print("  3. Close any serial monitor programs (Arduino IDE, PuTTY, etc.)")
    print("  4. Check Task Manager for any program using COM ports")
    
    if sys.platform == "win32" and device_info:
        print(f"\nWindows-specific solution:")
        print(f"  The FPGA is showing as COM port, but needs FTDI driver access.")
        print(f"  Try this command in Administrator PowerShell:")
        print(f"     iceprog synth\\{bitfile.name}")
    
    return 1

# =============================================================================
# Clean Function
# =============================================================================

def clean():
    """Clean build artifacts."""
    print_header("Cleaning Build Artifacts")
    
    dirs_to_clean = [
        TB_DIR / "sim_build",
        TB_DIR / "__pycache__",
        PROJECT_ROOT / "__pycache__",
        SYNTH_DIR / "voxel_bin",
        SYNTH_DIR / "gradient_map",
    ]
    
    files_to_clean = [
        TB_DIR / "results.xml",
    ]
    
    for d in dirs_to_clean:
        if d.exists():
            shutil.rmtree(d)
            print_success(f"Removed {d.relative_to(PROJECT_ROOT)}")
    
    for f in files_to_clean:
        if f.exists():
            f.unlink()
            print_success(f"Removed {f.relative_to(PROJECT_ROOT)}")
    
    print_success("Clean complete")
    return 0

# =============================================================================
# Main
# =============================================================================

def print_usage():
    print(__doc__)

def parse_args(raw_args):
    options = {
        "skip_fpga": False,
        "port": None,
        "serial": None,
        "vid": None,
        "pid": None,
    }
    positional = []

    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == "--skip-fpga":
            options["skip_fpga"] = True
            i += 1
            continue
        if arg in ["--port", "--serial", "--vid", "--pid"]:
            if i + 1 >= len(raw_args):
                print_error(f"Missing value for {arg}")
                return None, None
            value = raw_args[i + 1]
            if arg == "--port":
                options["port"] = value
            elif arg == "--serial":
                options["serial"] = value
            elif arg == "--vid":
                options["vid"] = int(value, 0)
            elif arg == "--pid":
                options["pid"] = int(value, 0)
            i += 2
            continue

        positional.append(arg)
        i += 1

    return positional, options

def main():
    args = sys.argv[1:]
    
    # Handle help first
    if "--help" in args or "-h" in args or "help" in args:
        print_usage()
        return 0
    
    # Parse flags
    args, options = parse_args(args)
    if args is None:
        return 1
    skip_fpga = options["skip_fpga"]
    
    if not args:
        # Full setup
        print_header("DVS Gesture Accelerator - Setup")
        print(f"Platform: {platform.system()} {platform.machine()}")
        print(f"Python: {sys.version.split()[0]}")
        
        if not setup_venv():
            return 1
        if not install_packages():
            return 1
        if not skip_fpga:
            setup_oss_cad_suite()
        
        print_header("Setup Complete!")
        print("\nNext steps:")
        if sys.platform == "win32":
            print("  Activate:  .venv\\Scripts\\activate")
        else:
            print("  Activate:  source .venv/bin/activate")
        print("  Run tests:       python setup.py test [voxel_bin|gradient_map]")
        print("  Synth:           python setup.py synth [voxel_bin|gradient_map]")
        print("  Flash:           python setup.py flash [voxel_bin|gradient_map]")
        return 0
    
    command = args[0].lower()
    arch_arg = args[1] if len(args) > 1 else None

    if command in ["test", "verify", "sim"]:
        module = arch_arg or "test_voxel_bin"
        return run_tests(module)
    elif command in ["synth", "synthesis", "build"]:
        arch = arch_arg or "voxel_bin"  # default for synth
        if arch not in ("voxel_bin", "gradient_map"):
            print_error(f"Unknown architecture: {arch}. Use voxel_bin or gradient_map.")
            return 1
        return run_synthesis(arch)
    elif command in ["flash", "program", "prog"]:
        arch = arch_arg or "voxel_bin"  # default for flash
        if arch not in ("voxel_bin", "gradient_map"):
            print_error(f"Unknown architecture: {arch}. Use voxel_bin or gradient_map.")
            return 1
        return flash_fpga(
            port=options["port"],
            serial=options["serial"],
            vid=options["vid"],
            pid=options["pid"],
            arch=arch,
        )
    elif command == "clean":
        return clean()
    elif command in ["help", "-h", "--help"]:
        print_usage()
        return 0
    else:
        print(f"Unknown command: {command}")
        print_usage()
        return 1

if __name__ == "__main__":
    sys.exit(main())
