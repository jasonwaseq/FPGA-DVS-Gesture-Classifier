"""
Python runner for Gradient-Map Gesture Classifier cocotb testbench
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
    """Run all gesture_top tests for the gradient_map_architecture"""
    
    proj_dir = proj_root
    rtl_dir = os.path.join(proj_dir, "rtl")
    tb_dir = os.path.join(proj_dir, "tb", "gradient_map_architecture")
    
    # RTL source files for gradient_map_architecture
    sources = [
        os.path.join(rtl_dir, "uart_tx.sv"),
        os.path.join(rtl_dir, "uart_rx.sv"),
        os.path.join(rtl_dir, "uart_debug.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "input_fifo.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "evt2_decoder.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "time_surface_memory.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "time_surface_encoder.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "flatten_buffer.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "weight_rom.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "systolic_array.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "spatio_temporal_classifier.sv"),
        os.path.join(rtl_dir, "gradient_map_architecture", "gesture_top.sv"),
    ]
    
    runner = get_runner("icarus")
    
    build_dir = os.path.join(tb_dir, "sim_build_gesture")
    os.makedirs(build_dir, exist_ok=True)
    
    build_args = [
        "-g2012",  # SystemVerilog
    ]
    
    runner.build(
        sources=sources,
        hdl_toplevel="gesture_top",
        build_args=build_args,
        build_dir=build_dir,
    )
    
    runner.test(
        hdl_toplevel="gesture_top",
        test_module="test_gradient_map",
        build_dir=build_dir,
    )

if __name__ == "__main__":
    run_tests()

