"""
Manual runner for Time-Surface Gesture Classifier cocotb testbench
Bypasses cocotb_tools.runner for better Windows compatibility
"""

import os
import sys
import subprocess

def main():
    # Project paths
    proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rtl_dir = os.path.join(proj_root, "rtl")
    tb_dir = os.path.join(proj_root, "tb")
    build_dir = os.path.join(tb_dir, "sim_build_gesture")
    
    # OSS CAD Suite paths - check multiple locations
    oss_cad_candidates = [
        os.path.join(proj_root, "oss-cad-suite", "oss-cad-suite"),
        r"C:\Users\jason\Documents\oss-cad-suite",
        os.path.expanduser("~/Documents/oss-cad-suite"),
    ]
    
    oss_cad_root = None
    for candidate in oss_cad_candidates:
        if os.path.exists(os.path.join(candidate, "bin", "iverilog.exe")):
            oss_cad_root = candidate
            break
    
    if oss_cad_root is None:
        print("ERROR: Could not find oss-cad-suite installation!")
        print(f"Searched: {oss_cad_candidates}")
        return 1
    
    oss_cad_bin = os.path.join(oss_cad_root, "bin")
    oss_cad_lib = os.path.join(oss_cad_root, "lib")
    print(f"Using oss-cad-suite from: {oss_cad_root}")
    
    # Set up environment
    env = os.environ.copy()
    env["PATH"] = oss_cad_bin + os.pathsep + oss_cad_lib + os.pathsep + env.get("PATH", "")
    
    # Cocotb environment variables
    venv_dir = os.path.join(proj_root, ".venv")
    cocotb_lib_dir = os.path.join(venv_dir, "Lib", "site-packages", "cocotb", "libs")
    python_exe = os.path.join(venv_dir, "Scripts", "python.exe")
    
    env["MODULE"] = "test_gradient_map"
    env["COCOTB_TEST_MODULES"] = "test_gradient_map"  # Cocotb 2.0+
    env["TOPLEVEL"] = "gesture_top"
    env["TOPLEVEL_LANG"] = "verilog"
    env["COCOTB_RESULTS_FILE"] = os.path.join(build_dir, "results.xml")
    env["PYGPI_PYTHON_BIN"] = python_exe
    env["LIBPYTHON_LOC"] = r"C:\Users\jason\AppData\Local\Python\pythoncore-3.14-64\python314.dll"
    
    # Python path for test module
    env["PYTHONPATH"] = tb_dir + os.pathsep + env.get("PYTHONPATH", "")
    
    # Create build directory
    os.makedirs(build_dir, exist_ok=True)
    
    # RTL source files
    sources = [
        os.path.join(rtl_dir, "bram_256x16.sv"),
        os.path.join(rtl_dir, "input_fifo.sv"),
        os.path.join(rtl_dir, "evt2_decoder.sv"),
        os.path.join(rtl_dir, "time_surface_memory.sv"),
        os.path.join(rtl_dir, "feature_extractor.sv"),
        os.path.join(rtl_dir, "uart_tx.sv"),
        os.path.join(rtl_dir, "uart_rx.sv"),
        os.path.join(rtl_dir, "uart_debug.sv"),
        os.path.join(rtl_dir, "gesture_top.sv"),
    ]
    
    vvp_file = os.path.join(build_dir, "sim.vvp")
    
    # Step 1: Compile with iverilog
    print("=" * 60)
    print("STEP 1: Compiling RTL with iverilog")
    print("=" * 60)
    
    iverilog_cmd = [
        os.path.join(oss_cad_bin, "iverilog.exe"),
        "-o", vvp_file,
        "-s", "gesture_top",
        "-g2012",
    ] + sources
    
    print(f"Running: {' '.join(iverilog_cmd)}")
    result = subprocess.run(iverilog_cmd, env=env, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"COMPILATION FAILED!")
        print(f"stdout: {result.stdout}")
        print(f"stderr: {result.stderr}")
        return 1
    
    print("Compilation successful!")
    
    # Step 2: Run simulation with vvp and cocotb
    print("\n" + "=" * 60)
    print("STEP 2: Running simulation with cocotb")
    print("=" * 60)
    
    # Find cocotb VPI library - for icarus it's .vpl not .dll
    cocotb_vpi_name = "cocotbvpi_icarus"
    cocotb_vpi = os.path.join(cocotb_lib_dir, cocotb_vpi_name + ".vpl")
    
    if not os.path.exists(cocotb_vpi):
        print(f"ERROR: Could not find VPI library at {cocotb_vpi}")
        # List what's available
        print("Available files in cocotb libs:")
        for f in os.listdir(cocotb_lib_dir):
            print(f"  {f}")
        return 1
    
    print(f"Using VPI library: {cocotb_vpi}")
    
    vvp_cmd = [
        os.path.join(oss_cad_bin, "vvp.exe"),
        "-M", cocotb_lib_dir,
        "-m", cocotb_vpi_name,
        vvp_file,
    ]
    
    print(f"Running: {' '.join(vvp_cmd)}")
    print(f"MODULE={env['MODULE']}")
    print(f"TOPLEVEL={env['TOPLEVEL']}")
    
    result = subprocess.run(vvp_cmd, env=env, cwd=tb_dir)
    
    return result.returncode

if __name__ == "__main__":
    sys.exit(main())
