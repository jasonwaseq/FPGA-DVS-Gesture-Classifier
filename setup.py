#!/usr/bin/env python3
"""Setup: venv, Python deps, OSS CAD Suite, cocotb tests, synthesis, and flash for iCE40."""

import os
import sys
import platform
import subprocess
import signal
import shutil
import urllib.request
import tarfile
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.resolve()
OSS_CAD_PATH = PROJECT_ROOT / "oss-cad-suite"
RTL_DIR = PROJECT_ROOT / "rtl"
TB_DIR = PROJECT_ROOT / "tb"
SYNTH_DIR = PROJECT_ROOT / "synth"


def _is_wsl():
    if not sys.platform.startswith("linux"):
        return False
    try:
        return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text(encoding="ascii", errors="ignore").lower()
    except Exception:
        return False


def _primary_venv_name():
    if sys.platform == "win32":
        return ".venv-win"
    if sys.platform == "darwin":
        return ".venv-macos"
    if _is_wsl():
        return ".venv-wsl"
    if sys.platform.startswith("linux"):
        return ".venv-linux"
    return ".venv"


def _venv_has_expected_layout(path):
    if sys.platform == "win32":
        return (path / "Scripts" / "python.exe").exists() and (path / "Scripts" / "pip.exe").exists()
    return (path / "bin" / "python").exists() and (path / "bin" / "pip").exists()


PRIMARY_VENV_PATH = PROJECT_ROOT / _primary_venv_name()
LEGACY_VENV_PATH = PROJECT_ROOT / ".venv"
VENV_PATH = PRIMARY_VENV_PATH

PYTHON_PACKAGES = [
    "cocotb",
    "cocotb-test",
    "pytest",
    "gitpython",
    "numpy",
    "opencv-python",
    "pyserial",
]

RTL_FILES = [
    "ram_1r1w_sync.sv",
    "input_fifo.sv",
    "evt2_decoder.sv",
    "voxel_binning.sv",
    "systolic_array.sv",
    "gesture_classifier.sv",
    "uart_rx.sv",
    "uart_tx.sv",
    "uart_debug.sv",
    "voxel_bin_core.sv",
    "voxel_bin_top.sv",
]

TEST_TARGETS = {
    "evt2_decoder": {"toplevel": "evt2_decoder", "test_module": "test_evt2_decoder"},
    "gesture_classifier": {"toplevel": "gesture_classifier", "test_module": "test_gesture_classifier"},
    "input_fifo": {"toplevel": "input_fifo", "test_module": "test_input_fifo"},
    "ram_1r1w_sync": {"toplevel": "ram_1r1w_sync", "test_module": "test_ram_1r1w_sync"},
    "systolic_array": {"toplevel": "systolic_array", "test_module": "test_systolic_array"},
    "uart_rx": {"toplevel": "uart_rx", "test_module": "test_uart_rx"},
    "uart_tx": {"toplevel": "uart_tx", "test_module": "test_uart_tx"},
    "uart_debug": {"toplevel": "uart_debug", "test_module": "test_uart_debug"},
    "voxel_binning": {"toplevel": "voxel_binning", "test_module": "test_voxel_binning"},
    "voxel_bin_core": {"toplevel": "voxel_bin_core", "test_module": "test_voxel_bin_core"},
    "voxel_bin_top": {"toplevel": "voxel_bin_top", "test_module": "test_voxel_bin_top"},
}

TEST_TARGET_ALIASES = {
    "all": "all",
    "unit": "all",
    "top": "voxel_bin_top",
    "core": "voxel_bin_core",
}

OSS_CAD_URLS = {
    ("Windows", "AMD64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-windows-x64-20241121.exe",
    ("Linux", "x86_64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz",
    ("Linux", "aarch64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-arm64-20241121.tgz",
    ("Darwin", "x86_64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-darwin-x64-20241121.tgz",
    ("Darwin", "arm64"): "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-darwin-arm64-20241121.tgz",
}

ANSI_RESET = "\033[0m"
ANSI_RED = "\033[31m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"


def colorize(msg, color):
    if sys.stdout.isatty():
        return f"{color}{msg}{ANSI_RESET}"
    return msg


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

def _venv_expected_layout_exists():
    return get_python_cmd().exists() and get_pip_cmd().exists()


def _venv_wrong_platform_layout_exists():
    if sys.platform == "win32":
        return (VENV_PATH / "bin" / "python").exists() or (VENV_PATH / "bin" / "pip").exists()
    return (VENV_PATH / "Scripts" / "python.exe").exists() or (VENV_PATH / "Scripts" / "pip.exe").exists()


def _recreate_venv():
    if VENV_PATH.exists():
        shutil.rmtree(VENV_PATH)
    result = run_cmd([sys.executable, "-m", "venv", str(VENV_PATH)])
    return result is not None

def get_oss_cad_bin():
    iverilog_exe = "iverilog.exe" if sys.platform == "win32" else "iverilog"

    # 1. Project-local install (preferred — placed here by workflow download)
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
    which_iverilog = shutil.which("iverilog")
    if which_iverilog:
        tool_path = Path(which_iverilog).resolve()
        oss_candidate = tool_path.parent.parent
        ivl_exe = "ivl.exe" if sys.platform == "win32" else "ivl"
        if (oss_candidate / "lib" / "ivl" / ivl_exe).exists():
            return oss_candidate

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


def _terminate_process(proc):
    if proc.poll() is not None:
        return

    try:
        if sys.platform == "win32":
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            proc.terminate()
        proc.wait(timeout=2)
    except Exception:
        pass

    if proc.poll() is None:
        proc.kill()
        try:
            proc.wait(timeout=2)
        except Exception:
            pass


def _run_streaming_process(cmd, env=None, cwd=None):
    popen_kwargs = {
        "env": env,
        "cwd": cwd,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.STDOUT,
        "text": True,
        "bufsize": 1,
    }

    if sys.platform == "win32":
        popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

    proc = subprocess.Popen(cmd, **popen_kwargs)

    try:
        assert proc.stdout is not None
        for line in proc.stdout:
            print(line, end="")
    except KeyboardInterrupt:
        _terminate_process(proc)
        # Do not print child traceback/noise after Ctrl+C; exit cleanly via main().
        raise

    return proc.wait()

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

    if LEGACY_VENV_PATH.exists() and (LEGACY_VENV_PATH != VENV_PATH):
        print_warning(
            f"Legacy venv detected at {LEGACY_VENV_PATH.name}. "
            f"Using {VENV_PATH.name} for this platform."
        )
    
    if VENV_PATH.exists():
        if _venv_expected_layout_exists():
            print_success("Virtual environment already exists")
            return True

        if _venv_wrong_platform_layout_exists():
            print_warning(
                f"Existing {VENV_PATH.name} was created for another OS/shell layout. "
                "Recreating it for this environment."
            )
        else:
            print_warning(f"Existing {VENV_PATH.name} is incomplete/corrupt. Recreating it.")

        try:
            if not _recreate_venv():
                print_error("Failed to recreate virtual environment")
                return False
        except Exception as e:
            print_error(f"Failed to recreate virtual environment: {e}")
            return False

        print_success(f"Recreated virtual environment at {VENV_PATH.name}")
        return True

    if not _recreate_venv():
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


def _resolve_test_target(name):
    key = (name or "all").strip().lower()
    if key in TEST_TARGET_ALIASES:
        key = TEST_TARGET_ALIASES[key]
    if key.startswith("test_"):
        key = key[5:]
    return key


def _get_sim_tools(env, oss_root):
    if str(oss_root) == "__PATH__":
        return "iverilog", "vvp", []

    iverilog_name = "iverilog.exe" if sys.platform == "win32" else "iverilog"
    vvp_name = "vvp.exe" if sys.platform == "win32" else "vvp"
    iverilog_cmd = str(oss_root / "bin" / iverilog_name)
    vvp_cmd = str(oss_root / "bin" / vvp_name)

    iverilog_args = []
    ivl_dir = oss_root / "lib" / "ivl"
    if ivl_dir.exists():
        iverilog_args = ["-B", str(ivl_dir)]

    return iverilog_cmd, vvp_cmd, iverilog_args


def _get_linux_system_sim_tools():
    if not sys.platform.startswith("linux"):
        return None, None

    candidates = [
        (Path("/usr/bin/iverilog"), Path("/usr/bin/vvp")),
        (Path("/usr/local/bin/iverilog"), Path("/usr/local/bin/vvp")),
    ]
    for ivl, vvp in candidates:
        if ivl.exists() and vvp.exists():
            return str(ivl), str(vvp)
    return None, None


def _get_cocotb_libs(python_cmd, env):
    result = run_cmd(
        [str(python_cmd), "-c", "import cocotb, os; print(os.path.dirname(cocotb.__file__))"],
        env=env
    )
    if result is None:
        return None

    cocotb_path = Path(result.stdout.strip())
    cocotb_libs = cocotb_path / "libs"
    module_names = ["cocotbvpi_icarus", "libcocotbvpi_icarus"]
    chosen_module = None
    for module_name in module_names:
        vpi_module = cocotb_libs / f"{module_name}.vpi"
        vpi_vpl = cocotb_libs / f"{module_name}.vpl"
        vpi_so = cocotb_libs / f"{module_name}.so"

        if not vpi_module.exists() and vpi_vpl.exists():
            try:
                vpi_module.symlink_to(vpi_vpl.name)
            except OSError:
                shutil.copyfile(vpi_vpl, vpi_module)

        if vpi_module.exists() or vpi_vpl.exists() or vpi_so.exists():
            chosen_module = module_name
            break

    if chosen_module is None:
        return None, None

    return cocotb_libs, chosen_module


def _collect_testcase_results(results_file):
    passed = []
    failed = []

    if not results_file.exists():
        return passed, failed

    try:
        root = ET.parse(results_file).getroot()
    except ET.ParseError as exc:
        print_warning(f"Unable to parse cocotb results file {results_file}: {exc}")
        return passed, failed

    for testcase in root.findall(".//testcase"):
        name = testcase.attrib.get("name", "<unnamed>")
        has_failure = testcase.find("failure") is not None or testcase.find("error") is not None
        if has_failure:
            failed.append(name)
        else:
            passed.append(name)

    return passed, failed


def _print_testbench_summary(target_name, results_file):
    passed, failed = _collect_testcase_results(results_file)
    total = len(passed) + len(failed)

    if total == 0:
        print_warning(f"[SUMMARY] {target_name}: no testcase results found")
        return passed, failed

    headline = f"[SUMMARY] {target_name}: {len(passed)}/{total} passed"
    if failed:
        print(colorize(headline, ANSI_YELLOW))
    else:
        print(colorize(headline, ANSI_GREEN))

    for name in passed:
        print(f"  {colorize('PASS', ANSI_GREEN)} {name}")
    for name in failed:
        print(f"  {colorize('FAIL', ANSI_RED)} {name}")

    return passed, failed


def _run_single_test(target_name):
    cfg = TEST_TARGETS[target_name]
    toplevel = cfg["toplevel"]
    test_module = cfg["test_module"]

    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found. Run 'make setup' first.")
        return 1

    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1

    python = get_python_cmd()
    if not python.exists():
        print_error("Python venv not found. Run 'make setup' first.")
        return 1

    cocotb_libs, cocotb_vpi_module = _get_cocotb_libs(python, env)
    if cocotb_libs is None or cocotb_vpi_module is None:
        print_error("cocotb not installed in the project venv. Run 'make setup' first.")
        return 1

    iverilog_cmd, vvp_cmd, iverilog_args = _get_sim_tools(env, oss_root)

    # In WSL/Linux, bundled OSS-CAD runtime libs can conflict with host libpython
    # (GLIBC/libm mismatch). Prefer distro-native Icarus for cocotb tests when present.
    sys_iverilog, sys_vvp = _get_linux_system_sim_tools()
    if sys_iverilog and sys_vvp:
        iverilog_cmd, vvp_cmd, iverilog_args = sys_iverilog, sys_vvp, []
        env = os.environ.copy()

    env["PYGPI_PYTHON_BIN"] = str(python)
    env["COCOTB_TEST_MODULES"] = test_module
    env["TOPLEVEL"] = toplevel
    env["COCOTB_TOPLEVEL"] = toplevel
    env["TOPLEVEL_LANG"] = "verilog"
    env["PYTHONPATH"] = str(TB_DIR)
    env["WAVES"] = "1"
    # Keep cocotb/GPI framework noise quiet; test progress is printed by tb/util/test_logging.py.
    env["COCOTB_LOG_LEVEL"] = "ERROR"
    env["COCOTB_REDUCED_LOG_FMT"] = "1"
    env["GPI_LOG_LEVEL"] = "ERROR"
    env["PYGPI_LOG_LEVEL"] = "ERROR"

    if sys.platform == "win32":
        python_version = f"{sys.version_info.major}{sys.version_info.minor}"
        env["LIBPYTHON_LOC"] = str(Path(sys.base_prefix) / f"python{python_version}.dll")
    elif sys.platform.startswith("linux"):
        libpython = _find_libpython()
        if libpython:
            env["LIBPYTHON_LOC"] = libpython

    sim_build = TB_DIR / f"sim_build_{target_name}"
    sim_build.mkdir(parents=True, exist_ok=True)
    vvp_file = sim_build / "sim.vvp"
    wave_file = sim_build / f"{toplevel}.fst"
    dump_stub_file = sim_build / "cocotb_iverilog_dump.v"
    sources = [str(RTL_DIR / f) for f in RTL_FILES]

    # Always emit a waveform per testbench run.
    dump_stub_file.write_text(
        "\n".join(
            [
                "module cocotb_iverilog_dump();",
                "initial begin",
                f'    $dumpfile("{wave_file.as_posix()}");',
                f"    $dumpvars(0, {toplevel});",
                "end",
                "endmodule",
                "",
            ]
        ),
        encoding="utf-8",
    )
    sources.append(str(dump_stub_file))

    print(f"\n[TEST] {target_name}")
    compile_cmd = [
        iverilog_cmd,
    ] + iverilog_args + [
        "-g2012",
        "-s", "cocotb_iverilog_dump",
        "-s", toplevel,
        "-o", str(vvp_file),
    ] + sources

    result = subprocess.run(compile_cmd, env=env, cwd=PROJECT_ROOT, capture_output=True, text=True)
    if result.returncode != 0:
        print_error(f"Compilation failed for {target_name}")
        if result.stderr:
            print(result.stderr)
        if result.stdout:
            print(result.stdout)
        return 1

    sim_cmd = [
        vvp_cmd,
        "-M", str(cocotb_libs),
        "-m", cocotb_vpi_module,
        str(vvp_file),
        "-fst",
    ]
    results_file = sim_build / "results.xml"
    if results_file.exists():
        results_file.unlink()
    env["COCOTB_RESULTS_FILE"] = str(results_file)

    # Run from project root so relative data files used by RTL init blocks
    # (e.g. 8192weights.txt) resolve consistently.
    result_rc = _run_streaming_process(sim_cmd, env=env, cwd=PROJECT_ROOT)

    passed_tests = []
    failed_tests = []
    if results_file.exists():
        passed_tests, failed_tests = _print_testbench_summary(target_name, results_file)

    if result_rc != 0:
        print_error(colorize(f"[TESTBENCH FAIL] {target_name}", ANSI_RED))
        return result_rc
    if not results_file.exists():
        print_error("cocotb did not produce results.xml; simulator likely failed to load VPI module.")
        return 1

    if failed_tests:
        print_error(colorize(f"[TESTBENCH FAIL] {target_name}", ANSI_RED))
        return 1

    if wave_file.exists():
        print_success(f"[WAVEFORM] {wave_file}")
    else:
        print_warning(f"[WAVEFORM] missing expected file: {wave_file}")

    print_success(colorize(f"[TESTBENCH PASS] {target_name}", ANSI_GREEN))
    return 0


def run_tests(target="all"):
    print_header("Running cocotb Verification Tests")
    resolved = _resolve_test_target(target)

    if resolved == "all":
        failed = []
        for name in TEST_TARGETS:
            rc = _run_single_test(name)
            if rc != 0:
                failed.append(name)
        if failed:
            print_error(f"Failed test targets: {', '.join(failed)}")
            print_header("Tests Failed")
            return 1
        print_header("Tests Complete")
        return 0

    if resolved not in TEST_TARGETS:
        valid = ", ".join(["all"] + list(TEST_TARGETS.keys()))
        print_error(f"Unknown test target: {target}. Use one of: {valid}")
        return 1

    rc = _run_single_test(resolved)
    print_header("Tests Complete" if rc == 0 else "Tests Failed")
    return rc


def _synthesize(top_module, rtl_files, pcf_name, label="", allow_unconstrained=False):
    display = label or top_module
    print_header(f"Synthesizing {display} for iCE40 UP5K")

    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found. Run 'make setup' first.")
        return 1

    env = get_oss_cad_env()
    if env is None:
        print_error("Failed to set up OSS CAD Suite environment")
        return 1

    sources = [str(RTL_DIR / f) for f in rtl_files]

    SYNTH_DIR.mkdir(parents=True, exist_ok=True)

    output_json = SYNTH_DIR / f"{top_module}.json"
    output_asc = SYNTH_DIR / f"{top_module}.asc"
    output_bit = SYNTH_DIR / f"{top_module}.bit"

    pcf_file = SYNTH_DIR / pcf_name

    if not pcf_file.exists():
        print_error(f"PCF file not found: {pcf_file}")
        return 1

    print(f"\nRunning Yosys synthesis (top={top_module})...")
    yosys_cmd = (
        f"read_verilog -sv -D SYNTHESIS {' '.join(sources)}; "
        f"synth_ice40 -dsp -top {top_module} -json {output_json}"
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


def run_synthesis():
    return _synthesize(
        top_module="voxel_bin_top",
        rtl_files=RTL_FILES,
        pcf_name="icebreaker.pcf",
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

def flash_fpga(port=None, serial=None, vid=None, pid=None):
    print_header("Flashing FPGA")

    bitfile = SYNTH_DIR / "voxel_bin_top.bit"

    oss_root = get_oss_cad_bin()
    if oss_root is None:
        print_error("OSS CAD Suite not found")
        return 1
    if not bitfile.exists():
        print_error(f"Bitstream not found: {bitfile}")
        print("  Run 'make synth' first")
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
        TB_DIR / "__pycache__",
        TB_DIR / "util" / "__pycache__",
        PROJECT_ROOT / "__pycache__",
    ]
    
    files_to_clean = [
        TB_DIR / "results.xml",
    ]

    for sim_build in TB_DIR.glob("sim_build*"):
        if sim_build.is_dir():
            dirs_to_clean.append(sim_build)
        elif sim_build.is_file():
            files_to_clean.append(sim_build)
    
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
    test_targets = ", ".join(TEST_TARGETS.keys())
    print(
        "Usage:\n"
        "  make setup [SKIP_FPGA=1]\n"
        "  make doctor\n"
        f"  make test [all|{test_targets}]\n"
        "  make synth\n"
        "  make flash [PORT=COM3 SERIAL=<sn> VID=0x0403 PID=0x6010]\n"
        "  make clean\n"
        "\n"
        "Defaults:\n"
        "  make test               -> test all\n"
        "  make synth              -> synth voxel_bin\n"
        "  make flash              -> flash voxel_bin\n"
        "\n"
        "Internal script (if needed):\n"
        "  python setup.py <command>\n"
    )


def run_setup(skip_fpga=False):
    print_header("DVS Gesture Accelerator - Setup")
    print(f"Platform: {platform.system()} {platform.machine()}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Venv path: {VENV_PATH.name}")

    if not setup_venv():
        return 1
    if not install_packages():
        return 1
    if not skip_fpga and not setup_oss_cad_suite():
        return 1

    print_header("Setup Complete")
    print("\nNext steps:")
    if sys.platform == "win32":
        print(f"  Activate venv: {VENV_PATH.name}\\Scripts\\activate")
    else:
        print(f"  Activate venv: source {VENV_PATH.name}/bin/activate")
    print("  Check setup:   make doctor")
    print("  Run tests:     make test")
    print("  Synthesize:    make synth")
    print("  Flash:         make flash")
    return 0


def _check_python_packages():
    python = get_python_cmd()
    if not python.exists():
        return None, ["venv-missing"]

    check_script = (
        "import importlib.util, json;"
        "mods=['cocotb','cocotb_test','pytest','git','numpy','cv2','serial'];"
        "missing=[m for m in mods if importlib.util.find_spec(m) is None];"
        "print(json.dumps(missing))"
    )
    result = run_cmd([str(python), "-c", check_script], check=False)
    if result is None or result.returncode != 0:
        return None, ["import-check-failed"]

    try:
        import json as _json
        missing = _json.loads(result.stdout.strip() or "[]")
    except Exception:
        missing = ["import-check-parse-failed"]
    return python, missing


def _sim_tool_smoke_test(env, oss_root):
    iverilog_cmd, vvp_cmd, iverilog_args = _get_sim_tools(env, oss_root)
    tmp_dir = PROJECT_ROOT / ".doctor_smoke"
    src = tmp_dir / "smoke.sv"
    out = tmp_dir / "smoke.vvp"
    try:
        tmp_dir.mkdir(parents=True, exist_ok=True)
        src.write_text(
            "module smoke; initial begin $display(\"SMOKE_OK\"); $finish; end endmodule\n",
            encoding="ascii",
        )

        compile_cmd = [iverilog_cmd] + iverilog_args + ["-g2012", "-s", "smoke", "-o", str(out), str(src)]
        c = subprocess.run(compile_cmd, env=env, capture_output=True, text=True)
        if c.returncode != 0:
            return False, f"compile failed: {c.stderr.strip() or c.stdout.strip()}"

        r = subprocess.run([vvp_cmd, str(out)], env=env, capture_output=True, text=True)
        if r.returncode != 0:
            return False, f"runtime failed: {r.stderr.strip() or r.stdout.strip()}"
    except Exception as e:
        return False, f"smoke test exception: {e}"
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    return True, "ok"


def doctor():
    print_header("Environment Doctor")
    failures = 0
    warnings = 0

    python, missing_pkgs = _check_python_packages()
    if python is None:
        print_error("Python venv not found. Run: make setup")
        failures += 1
    else:
        print_success(f"Python venv: {python}")
        if missing_pkgs:
            print_error(f"Missing Python packages in venv: {', '.join(missing_pkgs)}")
            print("  Fix: make setup")
            failures += 1
        else:
            print_success("Python packages: cocotb, cocotb-test, pytest, gitpython, numpy, opencv-python, pyserial")

    oss_root = get_oss_cad_bin()
    env = get_oss_cad_env()
    if oss_root is None or env is None:
        print_error("OSS CAD Suite tools not found. Run: make setup")
        failures += 1
    else:
        print_success(f"OSS CAD Suite root: {oss_root}")
        required_tools = ["iverilog", "vvp", "yosys", "nextpnr-ice40", "icepack"]
        optional_tools = ["iceprog"]

        for tool in required_tools:
            path = shutil.which(tool, path=env.get("PATH"))
            if path:
                print_success(f"{tool}: {path}")
            else:
                print_error(f"{tool}: not found on PATH")
                failures += 1

        for tool in optional_tools:
            path = shutil.which(tool, path=env.get("PATH"))
            if path:
                print_success(f"{tool}: {path}")
            else:
                print_warning(f"{tool}: not found (flash will fail until installed/accessible)")
                warnings += 1

        smoke_ok, smoke_msg = _sim_tool_smoke_test(env, oss_root)
        if smoke_ok:
            print_success("iverilog/vvp smoke test passed")
        else:
            print_error(f"iverilog/vvp smoke test failed: {smoke_msg}")
            failures += 1

    pcf = SYNTH_DIR / "icebreaker.pcf"
    if pcf.exists():
        print_success(f"Constraints file: {pcf}")
    else:
        print_error(f"Missing constraints file: {pcf}")
        failures += 1

    if failures == 0:
        if warnings:
            print_header("Doctor Complete (Warnings)")
        else:
            print_header("Doctor Complete")
        return 0

    print_header("Doctor Found Issues")
    return 1

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
    try:
        args = sys.argv[1:]

        if "--help" in args or "-h" in args or "help" in args:
            print_usage()
            return 0

        args, options = parse_args(args)
        if args is None:
            return 1

        command = args[0].lower() if args else "setup"
        target = args[1] if len(args) > 1 else None

        if command == "setup":
            return run_setup(skip_fpga=options["skip_fpga"])
        if command == "doctor":
            return doctor()
        if command in ["test", "verify", "sim"]:
            return run_tests(target or "all")
        if command in ["synth", "synthesis", "build"]:
            return run_synthesis()
        if command in ["flash", "program", "prog"]:
            return flash_fpga(
                port=options["port"],
                serial=options["serial"],
                vid=options["vid"],
                pid=options["pid"],
            )
        if command == "clean":
            return clean()

        print(f"Unknown command: {command}")
        print_usage()
        return 1
    except KeyboardInterrupt:
        print_warning("Interrupted by user (Ctrl+C).")
        return 130

if __name__ == "__main__":
    sys.exit(main())
