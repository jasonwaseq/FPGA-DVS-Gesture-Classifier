# DVS Gesture Accelerator - Windows Setup Script
# Run with: .\setup.ps1

param(
    [switch]$SkipFPGATools,
    [switch]$Help
)

if ($Help) {
    Write-Host @"
DVS Gesture Accelerator Setup Script

Usage: .\setup.ps1 [options]

Options:
    -SkipFPGATools    Skip downloading OSS CAD Suite (FPGA tools)
    -Help             Show this help message

This script will:
    1. Create a Python virtual environment
    2. Install Python dependencies (opencv, pyserial, cocotb)
    3. Download and extract OSS CAD Suite (yosys, nextpnr, icarus, etc.)
    4. Create an activation script for easy environment setup
"@
    exit 0
}

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  DVS Gesture Accelerator - Setup Script" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# Check Python
Write-Host "[1/4] Checking Python installation..." -ForegroundColor Yellow
$python = Get-Command python -ErrorAction SilentlyContinue
if (-not $python) {
    Write-Host "ERROR: Python not found. Please install Python 3.10+ from https://python.org" -ForegroundColor Red
    exit 1
}
$pythonVersion = python --version
Write-Host "  Found: $pythonVersion" -ForegroundColor Green

# Create virtual environment
Write-Host ""
Write-Host "[2/4] Setting up Python virtual environment..." -ForegroundColor Yellow
$venvPath = Join-Path $ProjectRoot ".venv"
if (-not (Test-Path $venvPath)) {
    python -m venv $venvPath
    Write-Host "  Created virtual environment at .venv" -ForegroundColor Green
} else {
    Write-Host "  Virtual environment already exists" -ForegroundColor Green
}

# Install Python packages
Write-Host ""
Write-Host "[3/4] Installing Python dependencies..." -ForegroundColor Yellow
$pipPath = Join-Path $venvPath "Scripts\pip.exe"
& $pipPath install --upgrade pip | Out-Null
& $pipPath install opencv-python numpy pyserial cocotb pytest
Write-Host "  Installed: opencv-python, numpy, pyserial, cocotb, pytest" -ForegroundColor Green

# Download OSS CAD Suite
if (-not $SkipFPGATools) {
    Write-Host ""
    Write-Host "[4/4] Setting up FPGA tools (OSS CAD Suite)..." -ForegroundColor Yellow
    
    $ossPath = Join-Path $ProjectRoot "oss-cad-suite"
    
    if (Test-Path $ossPath) {
        Write-Host "  OSS CAD Suite already exists at $ossPath" -ForegroundColor Green
    } else {
        $ossUrl = "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-windows-x64-20241121.exe"
        $ossInstaller = Join-Path $env:TEMP "oss-cad-suite.exe"
        
        Write-Host "  Downloading OSS CAD Suite (~300MB)..." -ForegroundColor Cyan
        Write-Host "  This may take a few minutes..."
        
        try {
            Invoke-WebRequest -Uri $ossUrl -OutFile $ossInstaller -UseBasicParsing
            Write-Host "  Extracting to $ossPath..." -ForegroundColor Cyan
            Start-Process -FilePath $ossInstaller -ArgumentList "-o`"$ossPath`"", "-y" -Wait -NoNewWindow
            Remove-Item $ossInstaller -ErrorAction SilentlyContinue
            Write-Host "  OSS CAD Suite installed successfully" -ForegroundColor Green
        } catch {
            Write-Host "  WARNING: Failed to download OSS CAD Suite" -ForegroundColor Yellow
            Write-Host "  You can manually download from: https://github.com/YosysHQ/oss-cad-suite-build/releases" -ForegroundColor Yellow
        }
    }
} else {
    Write-Host ""
    Write-Host "[4/4] Skipping FPGA tools (use -SkipFPGATools was specified)" -ForegroundColor Yellow
}

# Create activation script
Write-Host ""
Write-Host "Creating activation script..." -ForegroundColor Yellow

$activateScript = @'
# DVS Gesture Accelerator Environment Activation
# Run with: . .\activate.ps1

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

# Activate Python venv
$venvActivate = Join-Path $ProjectRoot ".venv\Scripts\Activate.ps1"
if (Test-Path $venvActivate) {
    . $venvActivate
    Write-Host "Python virtual environment activated" -ForegroundColor Green
}

# Add OSS CAD Suite to PATH
$ossPath = Join-Path $ProjectRoot "oss-cad-suite"
if (Test-Path $ossPath) {
    $env:PATH = "$ossPath\bin;$ossPath\lib;$env:PATH"
    Write-Host "OSS CAD Suite added to PATH" -ForegroundColor Green
}

# Check for user's existing OSS CAD Suite
$userOss = "$env:USERPROFILE\Documents\oss-cad-suite-windows"
if ((Test-Path $userOss) -and -not (Test-Path $ossPath)) {
    $env:PATH = "$userOss\bin;$userOss\lib;$env:PATH"
    Write-Host "Using OSS CAD Suite from $userOss" -ForegroundColor Green
}

Write-Host ""
Write-Host "Environment ready! Available commands:" -ForegroundColor Cyan
Write-Host "  yosys, nextpnr-ice40, icepack, iceprog, iverilog" -ForegroundColor White
Write-Host ""
Write-Host "Quick start:" -ForegroundColor Cyan
Write-Host "  cd synth; yosys -p 'read_verilog -sv ../rtl/*.sv; synth_ice40 -top uart_gesture_top -json out.json'" -ForegroundColor White
Write-Host "  cd tools; python dvs_camera_emulator.py --preview" -ForegroundColor White
'@

$activateScript | Out-File -FilePath (Join-Path $ProjectRoot "activate.ps1") -Encoding UTF8

Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  Setup Complete!" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Write-Host "To activate the environment, run:" -ForegroundColor Cyan
Write-Host "  . .\activate.ps1" -ForegroundColor White
Write-Host ""
Write-Host "Then you can:" -ForegroundColor Cyan
Write-Host "  - Run camera emulator:  cd tools; python dvs_camera_emulator.py --preview" -ForegroundColor White
Write-Host "  - Synthesize design:    cd synth; yosys -p 'read_verilog -sv ../rtl/*.sv; synth_ice40 -top uart_gesture_top'" -ForegroundColor White
Write-Host "  - Run tests:            cd tb; make test" -ForegroundColor White
Write-Host ""
