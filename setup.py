#!/usr/bin/env python3
"""Setup: venv, Python deps, OSS CAD Suite, cocotb tests, synthesis, and flash for iCE40."""

import os
import sys
import platform
import subprocess
import shutil
import urllib.request
import tarfile
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.resolve()
VENV_PATH = PROJECT_ROOT / ".venv"
OSS_CAD_PATH = PROJECT_ROOT / "oss-cad-suite"
RTL_DIR = PROJECT_ROOT / "rtl"
TB_DIR = PROJECT_ROOT / "tb"
SYNTH_DIR = PROJECT_ROOT / "synth"

PYTHON_PACKAGES = [
    "cocotb",
    "pytest", 
    "numpy",
    "opencv-python",
    "pyserial",
]

VOXEL_BIN_CORE_FILES = [
    "uart_rx.sv",
    "uart_tx.sv",
    "voxel_bin_architecture/input_fifo.sv",
    "voxel_bin_architecture/evt2_decoder.sv",
    "voxel_bin_architecture/voxel_binning.sv",
    "voxel_bin_architecture/systolic_array.sv",
    "voxel_bin_architecture/gesture_classifier.sv",
    "voxel_bin_architecture/weight_ram.sv",
    "voxel_bin_architecture/voxel_bin_core.sv",
]

RTL_FILES_VOXEL_RAW = VOXEL_BIN_CORE_FILES + [
    "voxel_bin_architecture/voxel_bin_raw_top.sv",
]

RTL_FILES_VOXEL_PROCESSED = VOXEL_BIN_CORE_FILES + [
    "voxel_bin_architecture/voxel_bin_processed_top.sv",
]

# Keep legacy alias so any external references don't break
RTL_FILES = RTL_FILES_VOXEL_RAW

GRADIENT_MAP_CORE_FILES = [
    "uart_tx.sv",
    "uart_rx.sv",
    "uart_debug.sv",
    "gradient_map_architecture/input_fifo.sv",
    "gradient_map_architecture/evt2_decoder.sv",
    "gradient_map_architecture/gradient_mapping.sv",
    "gradient_map_architecture/systolic_array.sv",
    "gradient_map_architecture/weight_ram.sv",
    "gradient_map_architecture/gesture_classifier.sv",
    "gradient_map_architecture/gradient_map_core.sv",
]

RTL_FILES_GRADIENT_RAW = GRADIENT_MAP_CORE_FILES + [
    "gradient_map_architecture/gradient_map_top.sv",
]

RTL_FILES_GRADIENT_PROCESSED = RTL_FILES_GRADIENT_RAW

# Keep legacy aliases
RTL_FILES_GESTURE = GRADIENT_MAP_CORE_FILES
RTL_FILES_UART = RTL_FILES_GRADIENT_RAW

OSS_CAD_URLS = {
    ("Windows", "AMD64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-windows-x64-20241121.exe",
    ("Linux", "x86_64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz",
    ("Linux", "aarch64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-arm64-20241121.tgz",
    ("Darwin", "x86_64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-darwin-x64-20241121.tgz",
    ("Darwin", "arm64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-darwin-arm64-20241121.tgz",
}


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
    system = platform.system()
    machine = platform.machine()
    return system, machine

def get_python_cmd():
    if sys.platform == "win32":
        return VENV_PATH / "Scripts" / "python.exe"
    return VENV_PATH / "bin" / "python"

def get_pip_cmd():
    if sys.platform == "win32":
        return VENV_PATH / "Scripts" / "pip.exe"
    return VENV_PATH / "bin" / "pip"

def get_oss_cad_bin():
    iverilog_exe = "iverilog.exe" if sys.platform == "win32" else "iverilog"

    # 1. Project-local install (preferred — placed here by setup.py download)
    candidates = [
        OSS_CAD_PATH / "oss-cad-suite",  # nested layout from Windows self-extractor
        OSS_CAD_PATH,                     # flat layout from tgz extraction
    ]

    # 2. Common user-level install locations (cross-platform, no hardcoded usernames)
    candidates += [
        Path.home() / "oss-cad-suite",
        Path("/opt/oss-cad-suite"),       # devcontainer / Linux system install
    ]

    for p in candidates:
        if p.exists() and (p / "bin" / iverilog_exe).exists():
            return p

    # 3. Fall back to whatever is already on PATH
    if shutil.which("iverilog"):
        # Return a sentinel that signals "use PATH directly"
        return Path("__PATH__")

    return None

def get_oss_cad_env():
    oss_root = get_oss_cad_bin()
    if oss_root is None:
        return None

    env = os.environ.copy()

    # Sentinel: tools are already on PATH (e.g. devcontainer, system install)
    if str(oss_root) == "__PATH__":
        return env

    oss_bin = oss_root / "bin"
    oss_lib = oss_root / "lib"

    env["PATH"] = f"{oss_bin}{os.pathsep}{oss_lib}{os.pathsep}{env.get('PATH', '')}"
    env["YOSYSHQ_ROOT"] = str(oss_root) + os.sep
    if (oss_root / "etc" / "cacert.pem").exists():
        env["SSL_CERT_FILE"] = str(oss_root / "etc" / "cacert.pem")
    env["QT_LOGGING_RULES"] = "*=false"
    env["GTK_EXE_PREFIX"] = str(oss_root)
    env["GTK_DATA_PREFIX"] = str(oss_root)
    env["GDK_PIXBUF_MODULEDIR"] = str(oss_lib / "gdk-pixbuf-2.0" / "2.10.0" / "loaders")
    env["GDK_PIXBUF_MODULE_FILE"] = str(oss_lib / "gdk-pixbuf-2.0" / "2.10.0" / "loaders.cache")
    env["OPENFPGALOADER_SOJ_DIR"] = str(oss_root / "share" / "openFPGALoader")

    # Shared library search path for Linux — detect the multiarch tuple dynamically
    # so this works on x86_64, aarch64, and any other Linux architecture.
    if sys.platform.startswith("linux"):
        import subprocess as _sp
        try:
            multiarch = _sp.check_output(
                ["dpkg-architecture", "-qDEB_HOST_MULTIARCH"],
                stderr=_sp.DEVNULL, text=True
            ).strip()
        except Exception:
            import platform as _plat
            _machine = _plat.machine()
            _arch_map = {
                "x86_64": "x86_64-linux-gnu",
                "aarch64": "aarch64-linux-gnu",
                "armv7l": "arm-linux-gnueabihf",
            }
            multiarch = _arch_map.get(_machine, f"{_machine}-linux-gnu")
        lib_dirs = [
            f"/lib/{multiarch}",
            f"/usr/lib/{multiarch}",
        ]
        existing = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = os.pathsep.join(lib_dirs + ([existing] if existing else []))

    return env

def run_cmd(cmd, env=None, cwd=None, check=True):
    if isinstance(cmd, str):
        cmd = cmd.split()
    result = subprocess.run(cmd, env=env, cwd=cwd, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(f"Command failed: {' '.join(str(c) for c in cmd)}")
        print(result.stderr)
        return None
    return result

def download_with_progress(url, dest):
    print(f"  Downloading from: {url}")
    print(f"  This may take several minutes...")
    
    try:
        urllib.request.urlretrieve(url, dest)
        return True
    except Exception as e:
        print_error(f"Download failed: {e}")
        return False


def setup_venv():
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
    print_step("2/3", "Installing Python packages...")

    pip = get_pip_cmd()
    if not pip.exists():
        print_error("pip not found in virtual environment")
        return False

    run_cmd([str(pip), "install", "--upgrade", "pip"], check=False)

    req_file = PROJECT_ROOT / "requirements.txt"
    if req_file.exists():
        result = run_cmd([str(pip), "install", "-r", str(req_file)])
    else:
        result = run_cmd([str(pip), "install"] + PYTHON_PACKAGES)

    if result is None:
        print_error("Failed to install packages")
        return False

    print_success(f"Installed packages from {'requirements.txt' if req_file.exists() else 'PYTHON_PACKAGES list'}")
    return True

def setup_oss_cad_suite():
    print_step("3/3", "Setting up OSS CAD Suite (FPGA tools)...")
    
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
            archive = Path(tmpdir) / "oss-cad-suite.tgz"
            if not download_with_progress(url, archive):
                return False
            
            print("  Extracting...")
            with tarfile.open(archive, "r:gz") as tar:
                tar.extractall(PROJECT_ROOT)
    
    print_success("OSS CAD Suite installed successfully")
    return True


def _find_libpython():
    """Return the absolute path to the Python shared library for the running interpreter.

    cocotb's GPI layer needs to dlopen() libpython at simulation time.  The
    plain `python3` apt package on Debian/Ubuntu does not ship the .so; it is
    provided by `python3-dev` / `libpython3.x`.  We try several strategies so
    this works across distros, venvs, and conda environments.
    """
    import sysconfig

    ver = f"{sys.version_info.major}.{sys.version_info.minor}"

    # Candidate library names (most-specific first)
    names = [
        f"libpython{ver}.so.1.0",
        f"libpython{ver}.so",
        f"libpython{ver}m.so.1.0",
        f"libpython{ver}m.so",
        f"libpython3.so",
    ]

    # Directories to search
    search_dirs = []

    # 1. The venv's lib directory (works when Python was built with --enable-shared)
    venv_lib = VENV_PATH / "lib"
    if venv_lib.exists():
        search_dirs += [str(d) for d in venv_lib.iterdir() if d.is_dir()]

    # 2. The base interpreter's LIBDIR and MULTIARCH lib paths
    libdir = sysconfig.get_config_var("LIBDIR")
    if libdir:
        search_dirs.append(libdir)

    # 3. Standard system library paths
    import platform as _plat
    machine = _plat.machine()
    arch_map = {
        "x86_64": "x86_64-linux-gnu",
        "aarch64": "aarch64-linux-gnu",
        "armv7l": "arm-linux-gnueabihf",
    }
    multiarch = arch_map.get(machine, f"{machine}-linux-gnu")
    search_dirs += [
        f"/usr/lib/{multiarch}",
        f"/lib/{multiarch}",
        "/usr/lib",
        "/lib",
        f"/usr/lib/python{ver}",
        f"/usr/local/lib",
    ]

    for d in search_dirs:
        for name in names:
            candidate = Path(d) / name
            if candidate.exists():
                return str(candidate)

    # 4. Last resort: ask ldconfig
    try:
        out = subprocess.check_output(
            ["ldconfig", "-p"], stderr=subprocess.DEVNULL, text=True
        )
        for line in out.splitlines():
            for name in names[:-1]:  # skip the generic libpython3.so
                if name in line and "=>" in line:
                    path = line.split("=>")[-1].strip()
                    if Path(path).exists():
                        return path
    except Exception:
        pass

    return None


ARCH_TEST_CONFIG = {
    "voxel_bin_raw": (RTL_FILES_VOXEL_RAW, "voxel_bin_raw_top", TB_DIR / "voxel_bin_architecture", [], [
        "-Pvoxel_bin_raw_top.CYCLES_PER_BIN=2000",
        "-Pvoxel_bin_raw_top.BAUD_RATE=3000000",
        "-Pvoxel_bin_raw_top.WINDOW_MS=40",
    ]),
    "voxel_bin_processed": (RTL_FILES_VOXEL_PROCESSED, "voxel_bin_processed_top", TB_DIR / "voxel_bin_architecture", [], [
        "-Pvoxel_bin_processed_top.CYCLES_PER_BIN=2000",
        "-Pvoxel_bin_processed_top.WINDOW_MS=40",
    ]),
    "gradient_map_raw": (RTL_FILES_GRADIENT_RAW, "gradient_map_raw_top", TB_DIR / "gradient_map_architecture", [], [
        "-Pgradient_map_raw_top.FRAME_PERIOD_MS=1",
        "-Pgradient_map_raw_top.MIN_MASS_THRESH=20",
        "-Pgradient_map_raw_top.DECAY_SHIFT=12",
    ]),
    "gradient_map_processed": (RTL_FILES_GRADIENT_PROCESSED, "gradient_map_processed_top", TB_DIR / "gradient_map_architecture", [], [
        "-Pgradient_map_processed_top.FRAME_PERIOD_MS=1",
        "-Pgradient_map_processed_top.MIN_MASS_THRESH=20",
        "-Pgradient_map_processed_top.DECAY_SHIFT=12",
    ]),
    # Legacy aliases
    "voxel_bin": (RTL_FILES_VOXEL_RAW, "voxel_bin_raw_top", TB_DIR / "voxel_bin_architecture", [], [
        "-Pvoxel_bin_raw_top.CYCLES_PER_BIN=2000",
        "-Pvoxel_bin_raw_top.BAUD_RATE=3000000",
        "-Pvoxel_bin_raw_top.WINDOW_MS=40",
    ]),
    "gradient_map": (RTL_FILES_GRADIENT_RAW, "gradient_map_raw_top", TB_DIR / "gradient_map_architecture", [], [
        "-Pgradient_map_raw_top.FRAME_PERIOD_MS=1",
        "-Pgradient_map_raw_top.MIN_MASS_THRESH=20",
        "-Pgradient_map_raw_top.DECAY_SHIFT=12",
    ]),
}


def run_tests(test_module=None):
    print_header("Running cocotb Verification Tests")

    TEST_MODULE_MAP = {
        "voxel_bin_raw":          ("voxel_bin_raw",       "test_voxel_bin_raw"),
        "test_voxel_bin_raw":     ("voxel_bin_raw",       "test_voxel_bin_raw"),
        "voxel_bin_processed":    ("voxel_bin_processed", "test_voxel_bin_processed"),
        "test_voxel_bin_processed": ("voxel_bin_processed", "test_voxel_bin_processed"),
        "gradient_map_raw":       ("gradient_map_raw",    "test_gradient_map_raw"),
        "test_gradient_map_raw":  ("gradient_map_raw",    "test_gradient_map_raw"),
        "gradient_map_processed": ("gradient_map_processed", "test_gradient_map_processed"),
        "test_gradient_map_processed": ("gradient_map_processed", "test_gradient_map_processed"),
        # Legacy aliases
        "voxel_bin":              ("voxel_bin_raw",       "test_voxel_bin_raw"),
        "test_voxel_bin":         ("voxel_bin_raw",       "test_voxel_bin_raw"),
        "gradient_map":           ("gradient_map_raw",    "test_gradient_map_raw"),
        "test_gradient_map":      ("gradient_map_raw",    "test_gradient_map_raw"),
    }
    key = test_module or "voxel_bin_raw"
    if key in TEST_MODULE_MAP:
        arch, test_module = TEST_MODULE_MAP[key]
    else:
        arch = "voxel_bin_raw"
        test_module = "test_voxel_bin_raw"

    rtl_files, toplevel, tb_dir, defines, param_overrides = ARCH_TEST_CONFIG[arch]
    
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
        try:
            vpi_module.symlink_to(vpi_fallback.name)
        except OSError:
            shutil.copyfile(vpi_fallback, vpi_module)
    
    env["COCOTB_TEST_MODULES"] = test_module
    env["TOPLEVEL"] = toplevel
    env["TOPLEVEL_LANG"] = "verilog"
    env["PYTHONPATH"] = str(tb_dir)
    env.setdefault("WAVES", "0")
    env.setdefault("COCOTB_LOG_LEVEL", "WARNING")
    env.setdefault("COCOTB_REDUCED_LOG_FMT", "1")
    
    # cocotb's GPI layer dlopen()s the Python shared library at runtime.
    # We must tell it exactly where to find it, otherwise it fails with
    # "cannot open shared object file" even when Python itself runs fine.
    if sys.platform == "win32":
        python_version = f"{sys.version_info.major}{sys.version_info.minor}"
        dll_name = f"python{python_version}.dll"
        env["LIBPYTHON_LOC"] = str(Path(sys.base_prefix) / dll_name)
        env["PYGPI_PYTHON_BIN"] = str(python)
    elif sys.platform.startswith("linux"):
        libpython = _find_libpython()
        if libpython:
            env["LIBPYTHON_LOC"] = libpython
    
    sim_build = tb_dir / "sim_build"
    sim_build.mkdir(parents=True, exist_ok=True)
    
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


def _synthesize(top_module, rtl_files, pcf_name, label="", arch_dir=None, allow_unconstrained=False):
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

    output_root = SYNTH_DIR / arch_dir if arch_dir else SYNTH_DIR
    output_root.mkdir(parents=True, exist_ok=True)

    output_json = output_root / f"{top_module}.json"
    output_asc = output_root / f"{top_module}.asc"
    output_bit = output_root / f"{top_module}.bit"

    pcf_file = SYNTH_DIR / pcf_name

    if not pcf_file.exists():
        print_error(f"PCF file not found: {pcf_file}")
        return 1

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

    print("\nRunning nextpnr place and route...")
    nextpnr_cmd = [
        "nextpnr-ice40",
        "--up5k", "--package", "sg48",
        "--json", str(output_json),
        "--pcf", str(pcf_file),
        "--asc", str(output_asc),
        "--freq", "12"
    ]
    if allow_unconstrained:
        nextpnr_cmd.append("--pcf-allow-unconstrained")
    result = subprocess.run(nextpnr_cmd, env=env, cwd=SYNTH_DIR)
    if result.returncode != 0:
        print_error("Place and route failed")
        return 1
    print_success("Place and route complete")

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
        return 1
    print_success(f"Bitstream created: {output_bit}")

    print_header("Synthesis Complete")
    return 0


ARCH_SYNTH_CONFIG = {
    # (top_module, rtl_files, pcf_name, label, subdir, allow_unconstrained)
    "voxel_bin_raw": (
        "voxel_bin_raw_top",
        RTL_FILES_VOXEL_RAW,
        "icebreaker.pcf",
        "Voxel-bin raw/UART (voxel_bin_raw_top)",
        "voxel_bin",
        False,
    ),
    "voxel_bin_processed": (
        "voxel_bin_processed_top",
        RTL_FILES_VOXEL_PROCESSED,
        "icebreaker_processed_voxel_bin.pcf",
        "Voxel-bin processed/bus (voxel_bin_processed_top)",
        "voxel_bin",
        True,
    ),
    "gradient_map_raw": (
        "gradient_map_raw_top",
        RTL_FILES_GRADIENT_RAW,
        "icebreaker.pcf",
        "Gradient-map raw/UART (gradient_map_raw_top)",
        "gradient_map",
        False,
    ),
    "gradient_map_processed": (
        "gradient_map_processed_top",
        RTL_FILES_GRADIENT_PROCESSED,
        "icebreaker_processed_gradient_map.pcf",
        "Gradient-map processed/bus (gradient_map_processed_top)",
        "gradient_map",
        False,
    ),
    # Legacy aliases
    "voxel_bin": (
        "voxel_bin_raw_top",
        RTL_FILES_VOXEL_RAW,
        "icebreaker.pcf",
        "Voxel-bin raw/UART (voxel_bin_raw_top)",
        "voxel_bin",
        False,
    ),
    "gradient_map": (
        "gradient_map_raw_top",
        RTL_FILES_GRADIENT_RAW,
        "icebreaker.pcf",
        "Gradient-map raw/UART (gradient_map_raw_top)",
        "gradient_map",
        False,
    ),
}


def run_synthesis(arch):
    if arch not in ARCH_SYNTH_CONFIG:
        print_error(
            f"Unknown architecture: {arch}. "
            "Use voxel_bin_raw, voxel_bin_processed, gradient_map_raw, or gradient_map_processed."
        )
        return 1

    top_module, rtl_files, pcf_name, label, subdir, allow_unconstrained = ARCH_SYNTH_CONFIG[arch]
    return _synthesize(
        top_module=top_module,
        rtl_files=rtl_files,
        pcf_name=pcf_name,
        label=label,
        arch_dir=subdir,
        allow_unconstrained=allow_unconstrained,
    )

def list_ftdi_devices():
    devices = []
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
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
    print_header("Flashing FPGA")

    if bitfile_name is None and arch is not None:
        if arch in ARCH_SYNTH_CONFIG:
            top_module = ARCH_SYNTH_CONFIG[arch][0]
            bitfile_name = f"{top_module}.bit"
        else:
            bitfile_name = f"{arch}.bit"
    if bitfile_name is None:
        bitfile_name = "voxel_bin_raw_top.bit"

    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found")
        return 1
    
    if arch is not None and arch in ARCH_SYNTH_CONFIG:
        _, _, _, _, subdir, _ = ARCH_SYNTH_CONFIG[arch]
        bitfile_dir = SYNTH_DIR / subdir
    else:
        bitfile_dir = SYNTH_DIR

    bitfile = bitfile_dir / bitfile_name
    if not bitfile.exists():
        print_error(f"Bitstream not found: {bitfile}")
        print("  Run 'python setup.py synth <arch>' first (e.g. synth voxel_bin_raw)")
        return 1
    
    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1
    
    if any([port, serial, vid, pid]):
        print_warning("Port/serial/VID/PID options are ignored when using iceprog.")

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
    
    iceprog_available = shutil.which("iceprog", path=str(oss_root / "bin"))

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
    
    print_error("\nAll programming methods failed")
    print("\nQuick fixes to try:")
    print("  1. Close this terminal and run PowerShell as Administrator, then retry")
    print("  2. Unplug the FPGA, wait 3 seconds, plug it back in, then retry")
    print("  3. Close any serial monitor programs (Arduino IDE, PuTTY, etc.)")
    print("  4. Check Task Manager for any program using COM ports")
    
    if sys.platform == "win32" and device_info:
        print(f"\nWindows-specific solution:")
        print(f"  Try this command in Administrator PowerShell:")
        print(f"     iceprog synth\\{bitfile.name}")
    
    return 1


def clean():
    print_header("Cleaning Build Artifacts")
    
    dirs_to_clean = [
        TB_DIR / "voxel_bin_architecture" / "sim_build",
        TB_DIR / "voxel_bin_architecture" / "__pycache__",
        TB_DIR / "gradient_map_architecture" / "sim_build",
        TB_DIR / "gradient_map_architecture" / "__pycache__",
        TB_DIR / "__pycache__",
        PROJECT_ROOT / "__pycache__",
        SYNTH_DIR / "voxel_bin",
        SYNTH_DIR / "gradient_map",
    ]
    
    files_to_clean = [
        TB_DIR / "voxel_bin_architecture" / "results.xml",
        TB_DIR / "gradient_map_architecture" / "results.xml",
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
    
    if "--help" in args or "-h" in args or "help" in args:
        print_usage()
        return 0
    
    args, options = parse_args(args)
    if args is None:
        return 1
    skip_fpga = options["skip_fpga"]
    
    if not args:
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
        print("  Run tests:       python setup.py test [voxel_bin_raw|voxel_bin_processed|gradient_map_raw|gradient_map_processed]")
        print("  Synth:           python setup.py synth [voxel_bin_raw|voxel_bin_processed|gradient_map_raw|gradient_map_processed]")
        print("  Flash:           python setup.py flash [voxel_bin_raw|voxel_bin_processed|gradient_map_raw|gradient_map_processed]")
        return 0
    
    command = args[0].lower()
    arch_arg = args[1] if len(args) > 1 else None

    if command in ["test", "verify", "sim"]:
        module = arch_arg or "test_voxel_bin"
        return run_tests(module)
    elif command in ["synth", "synthesis", "build"]:
        arch = arch_arg or "voxel_bin_raw"
        if arch not in ARCH_SYNTH_CONFIG:
            valid = ", ".join(k for k in ARCH_SYNTH_CONFIG if not k in ("voxel_bin", "gradient_map"))
            print_error(f"Unknown architecture: {arch}. Use one of: {valid}")
            return 1
        return run_synthesis(arch)
    elif command in ["flash", "program", "prog"]:
        arch = arch_arg or "voxel_bin_raw"
        if arch not in ARCH_SYNTH_CONFIG:
            valid = ", ".join(k for k in ARCH_SYNTH_CONFIG if not k in ("voxel_bin", "gradient_map"))
            print_error(f"Unknown architecture: {arch}. Use one of: {valid}")
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
