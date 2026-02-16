"""
Python runner for Time-Surface Gesture Classifier cocotb testbench
Uses cocotb_tools.runner for Windows compatibility
"""

import os
import sys
import subprocess

# Add oss-cad-suite to PATH for iverilog BEFORE importing runner
proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
oss_cad_bin = os.path.join(proj_root, "oss-cad-suite", "oss-cad-suite", "bin")
oss_cad_lib = os.path.join(proj_root, "oss-cad-suite", "oss-cad-suite", "lib")

# Set PATH with both bin and lib directories
os.environ["PATH"] = oss_cad_bin + os.pathsep + oss_cad_lib + os.pathsep + os.environ.get("PATH", "")

# For Windows: ensure iverilog can find its DLLs
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(oss_cad_bin)
    os.add_dll_directory(oss_cad_lib)

from cocotb_tools.runner import get_runner

def run_tests():
    """Run all gesture_top tests"""
    
    # Get the project root directory
    proj_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rtl_dir = os.path.join(proj_dir, "rtl")
    tb_dir = os.path.join(proj_dir, "tb")
    
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
    
    # Get the runner for Icarus Verilog
    runner = get_runner("icarus")
    
    # Build directory
    build_dir = os.path.join(tb_dir, "sim_build_gesture")
    os.makedirs(build_dir, exist_ok=True)
    
    # Build arguments for faster simulation
    # Note: Parameter overrides (-P) don't work on Windows iverilog
    # Use defines instead for test configuration
    build_args = [
        "-g2012",  # SystemVerilog
        "-DSIM_FAST=1",  # Enable fast simulation mode if RTL supports it
    ]
    
    # Build the simulation
    runner.build(
        sources=sources,
        hdl_toplevel="gesture_top",
        build_args=build_args,
        build_dir=build_dir,
    )
    
    # Run the tests
    runner.test(
        hdl_toplevel="gesture_top",
        test_module="test_gesture_top",
        build_dir=build_dir,
    )

if __name__ == "__main__":
    run_tests()
