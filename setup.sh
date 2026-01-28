#!/bin/bash
# DVS Gesture Accelerator - Linux/macOS Setup Script
# Run with: ./setup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKIP_FPGA_TOOLS=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-fpga-tools)
            SKIP_FPGA_TOOLS=true
            shift
            ;;
        --help|-h)
            echo "DVS Gesture Accelerator Setup Script"
            echo ""
            echo "Usage: ./setup.sh [options]"
            echo ""
            echo "Options:"
            echo "    --skip-fpga-tools    Skip downloading OSS CAD Suite"
            echo "    --help, -h           Show this help message"
            echo ""
            echo "This script will:"
            echo "    1. Create a Python virtual environment"
            echo "    2. Install Python dependencies"
            echo "    3. Download and extract OSS CAD Suite (Linux only)"
            echo "    4. Create an activation script"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "============================================"
echo "  DVS Gesture Accelerator - Setup Script"
echo "============================================"
echo ""

# Detect OS
OS="$(uname -s)"
ARCH="$(uname -m)"

# Check Python
echo "[1/4] Checking Python installation..."
if command -v python3 &> /dev/null; then
    PYTHON=python3
elif command -v python &> /dev/null; then
    PYTHON=python
else
    echo "ERROR: Python not found. Please install Python 3.10+"
    exit 1
fi
echo "  Found: $($PYTHON --version)"

# Create virtual environment
echo ""
echo "[2/4] Setting up Python virtual environment..."
VENV_PATH="$SCRIPT_DIR/.venv"
if [ ! -d "$VENV_PATH" ]; then
    $PYTHON -m venv "$VENV_PATH"
    echo "  Created virtual environment at .venv"
else
    echo "  Virtual environment already exists"
fi

# Install Python packages
echo ""
echo "[3/4] Installing Python dependencies..."
source "$VENV_PATH/bin/activate"
pip install --upgrade pip > /dev/null
pip install opencv-python numpy pyserial cocotb pytest
echo "  Installed: opencv-python, numpy, pyserial, cocotb, pytest"

# Download OSS CAD Suite (Linux only)
if [ "$SKIP_FPGA_TOOLS" = false ]; then
    echo ""
    echo "[4/4] Setting up FPGA tools (OSS CAD Suite)..."
    
    OSS_PATH="$SCRIPT_DIR/oss-cad-suite"
    
    if [ -d "$OSS_PATH" ]; then
        echo "  OSS CAD Suite already exists"
    elif [ "$OS" = "Linux" ]; then
        if [ "$ARCH" = "x86_64" ]; then
            OSS_URL="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz"
        elif [ "$ARCH" = "aarch64" ]; then
            OSS_URL="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-arm64-20241121.tgz"
        else
            echo "  WARNING: Unsupported architecture: $ARCH"
            OSS_URL=""
        fi
        
        if [ -n "$OSS_URL" ]; then
            echo "  Downloading OSS CAD Suite (~1GB)..."
            echo "  This may take several minutes..."
            
            TMP_FILE="/tmp/oss-cad-suite.tgz"
            if curl -L -o "$TMP_FILE" "$OSS_URL"; then
                echo "  Extracting..."
                tar -xzf "$TMP_FILE" -C "$SCRIPT_DIR"
                rm -f "$TMP_FILE"
                echo "  OSS CAD Suite installed successfully"
            else
                echo "  WARNING: Failed to download OSS CAD Suite"
                echo "  You can manually download from: https://github.com/YosysHQ/oss-cad-suite-build/releases"
            fi
        fi
    elif [ "$OS" = "Darwin" ]; then
        echo "  macOS detected. Install FPGA tools via Homebrew:"
        echo "    brew install yosys nextpnr icarus-verilog"
        echo "  Or download OSS CAD Suite manually from:"
        echo "    https://github.com/YosysHQ/oss-cad-suite-build/releases"
    fi
else
    echo ""
    echo "[4/4] Skipping FPGA tools (--skip-fpga-tools specified)"
fi

# Create activation script
echo ""
echo "Creating activation script..."

cat > "$SCRIPT_DIR/activate.sh" << 'ACTIVATE_EOF'
#!/bin/bash
# DVS Gesture Accelerator Environment Activation
# Run with: source ./activate.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Activate Python venv
if [ -f "$SCRIPT_DIR/.venv/bin/activate" ]; then
    source "$SCRIPT_DIR/.venv/bin/activate"
    echo "Python virtual environment activated"
fi

# Add OSS CAD Suite to PATH
if [ -d "$SCRIPT_DIR/oss-cad-suite" ]; then
    export PATH="$SCRIPT_DIR/oss-cad-suite/bin:$PATH"
    echo "OSS CAD Suite added to PATH"
fi

echo ""
echo "Environment ready! Available commands:"
echo "  yosys, nextpnr-ice40, icepack, iceprog, iverilog"
echo ""
echo "Quick start:"
echo "  cd synth && yosys -p 'read_verilog -sv ../rtl/*.sv; synth_ice40 -top uart_gesture_top -json out.json'"
echo "  cd tools && python dvs_camera_emulator.py --preview"
ACTIVATE_EOF

chmod +x "$SCRIPT_DIR/activate.sh"

echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "To activate the environment, run:"
echo "  source ./activate.sh"
echo ""
echo "Then you can:"
echo "  - Run camera emulator:  cd tools && python dvs_camera_emulator.py --preview"
echo "  - Synthesize design:    cd synth && make"
echo "  - Run tests:            cd tb && make test"
echo ""
