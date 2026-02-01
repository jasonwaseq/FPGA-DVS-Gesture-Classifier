#!/usr/bin/env python3
"""
DVS Gesture Accelerator - Universal Setup Script
Works on Windows, Linux, and macOS

Usage:
    python setup.py              # Full setup (venv + dependencies + FPGA tools)
    python setup.py --skip-fpga  # Skip OSS CAD Suite download
    python setup.py test         # Run cocotb verification tests
    python setup.py synth        # Synthesize for iCE40
    python setup.py flash        # Flash bitstream to FPGA
    python setup.py clean        # Clean build artifacts
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

# RTL source files in dependency order
RTL_FILES = [
    "uart_rx.sv",
    "uart_tx.sv",
    "InputFIFO.sv",
    "SpatialCompressor.sv",
    "TemporalAccumulator.sv",
    "MotionComputer.sv",
    "GestureClassifier.sv",
    "OutputRegister.sv",
    "dvs_gesture_accel.sv",
    "uart_gesture_top.sv",
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
    print(f"  ✓ {msg}")

def print_warning(msg):
    print(f"  ⚠ {msg}")

def print_error(msg):
    print(f"  ✗ {msg}")

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
    # Check project-local installation first
    if OSS_CAD_PATH.exists():
        return OSS_CAD_PATH
    
    # Check common user locations
    common_paths = [
        Path.home() / "Documents" / "oss-cad-suite",
        Path.home() / "Documents" / "oss-cad-suite-windows",
        Path.home() / "oss-cad-suite",
        Path("C:/oss-cad-suite"),
    ]
    
    for p in common_paths:
        if p.exists() and (p / "bin").exists():
            return p
    
    return None

def get_oss_cad_env():
    """Get environment with OSS CAD Suite properly initialized."""
    oss_root = get_oss_cad_bin()
    if oss_root is None:
        return None
    
    env = os.environ.copy()
    oss_bin = oss_root / "bin"
    oss_lib = oss_root / "lib"
    
    # Set PATH with lib first (for DLLs), then bin
    env["PATH"] = f"{oss_lib}{os.pathsep}{oss_bin}{os.pathsep}{env.get('PATH', '')}"
    
    # Set OSS CAD Suite environment variables (from environment.ps1)
    env["YOSYSHQ_ROOT"] = str(oss_root) + os.sep
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

def run_tests(test_module="test_spatiotemporal_classifier"):
    """Run cocotb verification tests."""
    print_header("Running cocotb Verification Tests")
    
    # Check for required tools
    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found. Run 'python setup.py' first.")
        return 1
    
    python = get_python_cmd()
    if not python.exists():
        print_error("Python venv not found. Run 'python setup.py' first.")
        return 1
    
    # Build environment with OSS CAD Suite properly initialized
    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1
    
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
    
    # Set up cocotb environment
    env["COCOTB_TEST_MODULES"] = test_module
    env["TOPLEVEL"] = "uart_gesture_top"
    env["TOPLEVEL_LANG"] = "verilog"
    env["PYTHONPATH"] = str(TB_DIR)
    
    # Python lib for cocotb
    if sys.platform == "win32":
        # Find python DLL
        python_version = f"{sys.version_info.major}{sys.version_info.minor}"
        dll_name = f"python{python_version}.dll"
        env["LIBPYTHON_LOC"] = str(Path(sys.base_prefix) / dll_name)
        env["PYGPI_PYTHON_BIN"] = str(python)
    
    # Create sim_build directory
    sim_build = TB_DIR / "sim_build"
    sim_build.mkdir(exist_ok=True)
    
    # Compile RTL with iverilog
    print("\nCompiling RTL with Icarus Verilog...")
    vvp_file = sim_build / "sim.vvp"
    sources = [str(RTL_DIR / f) for f in RTL_FILES]
    
    compile_cmd = [
        "iverilog", "-g2012",
        "-s", "uart_gesture_top",
        "-o", str(vvp_file),
        "-DCLKS_PER_BIT=4",
        "-DMIN_EVENT_THRESH=4",
        "-DMOTION_THRESH=2",
    ] + sources
    
    result = subprocess.run(compile_cmd, env=env, cwd=sim_build)
    if result.returncode != 0:
        print_error("Compilation failed")
        return 1
    print_success("Compilation successful")
    
    # Run simulation
    print("\nRunning simulation...")
    sim_cmd = [
        "vvp",
        "-M", str(cocotb_libs),
        "-m", "cocotbvpi_icarus",
        str(vvp_file)
    ]
    
    result = subprocess.run(sim_cmd, env=env, cwd=TB_DIR)
    
    print_header("Tests Complete" if result.returncode == 0 else "Tests Failed")
    return result.returncode

# =============================================================================
# Synthesis Functions
# =============================================================================

def run_synthesis():
    """Synthesize design for iCE40 UP5K."""
    print_header("Synthesizing for iCE40 UP5K")
    
    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found. Run 'python setup.py' first.")
        return 1
    
    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1
    
    # Check for sv2v (needed to convert SystemVerilog)
    sv2v = shutil.which("sv2v", path=str(oss_root / "bin"))
    if sv2v is None:
        print_warning("sv2v not found, trying direct Yosys synthesis...")
    
    sources = [str(RTL_DIR / f) for f in RTL_FILES]
    output_json = SYNTH_DIR / "uart_gesture_top.json"
    output_asc = SYNTH_DIR / "uart_gesture_top.asc"
    output_bit = SYNTH_DIR / "uart_gesture_top.bit"
    pcf_file = SYNTH_DIR / "icebreaker.pcf"
    
    # Synthesis with Yosys
    print("\nRunning Yosys synthesis...")
    yosys_cmd = f"read_verilog -sv {' '.join(sources)}; synth_ice40 -top uart_gesture_top -json {output_json}"
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
    result = subprocess.run(["icepack", str(output_asc), str(output_bit)], env=env)
    if result.returncode != 0:
        print_error("Bitstream generation failed")
        return 1
    print_success(f"Bitstream created: {output_bit}")
    
    print_header("Synthesis Complete")
    return 0

def flash_fpga():
    """Flash bitstream to FPGA."""
    print_header("Flashing FPGA")
    
    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found")
        return 1
    
    bitfile = SYNTH_DIR / "uart_gesture_top.bit"
    if not bitfile.exists():
        print_error(f"Bitstream not found: {bitfile}")
        print("  Run 'python setup.py synth' first")
        return 1
    
    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1
    
    print(f"Flashing {bitfile}...")
    result = subprocess.run(["iceprog", str(bitfile)], env=env)
    
    if result.returncode == 0:
        print_success("FPGA programmed successfully")
    else:
        print_error("Flash failed - check FPGA connection")
    
    return result.returncode

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
    ]
    
    files_to_clean = [
        TB_DIR / "results.xml",
        SYNTH_DIR / "uart_gesture_top.json",
        SYNTH_DIR / "uart_gesture_top.asc",
        SYNTH_DIR / "uart_gesture_top.v",
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

def main():
    args = sys.argv[1:]
    
    # Handle help first
    if "--help" in args or "-h" in args or "help" in args:
        print_usage()
        return 0
    
    # Parse flags
    skip_fpga = "--skip-fpga" in args
    args = [a for a in args if not a.startswith("--")]
    
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
        print("  Run tests: python setup.py test")
        print("  Synthesize: python setup.py synth")
        print("  Flash FPGA: python setup.py flash")
        return 0
    
    command = args[0].lower()
    
    if command in ["test", "verify", "sim"]:
        module = args[1] if len(args) > 1 else "test_spatiotemporal_classifier"
        return run_tests(module)
    elif command in ["synth", "synthesis", "build"]:
        return run_synthesis()
    elif command in ["flash", "program", "prog"]:
        return flash_fpga()
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
