FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

ENV LANG=C.UTF-8
ARG INSTALL_ZSH="true"
ARG TARGETARCH

# System packages: build tools, Python, GTKWave, Icarus Verilog (fallback simulator)
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    unzip \
    curl \
    git \
    make \
    build-essential \
    libgmp-dev \
    python3 \
    python3-pip \
    python3-venv \
    gtkwave \
    iverilog \
    && rm -rf /var/lib/apt/lists/*

# Install OSS CAD Suite (Yosys, nextpnr, iceprog, iverilog, verilator, …)
# Pinned to a stable release; update the date tag to upgrade.
ARG OSS_CAD_VERSION=20241121
RUN cd /opt && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        wget -q "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-arm64-${OSS_CAD_VERSION}.tgz" -O oss-cad-suite.tgz; \
    else \
        wget -q "https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-${OSS_CAD_VERSION}.tgz" -O oss-cad-suite.tgz; \
    fi && \
    tar -xf oss-cad-suite.tgz && \
    rm oss-cad-suite.tgz

ENV PATH="/opt/oss-cad-suite/bin:${PATH}"

# Install sv2v (SystemVerilog → Verilog converter, useful for linting)
RUN if [ "$TARGETARCH" = "arm64" ]; then \
        echo "Building sv2v from source (arm64)" && \
        curl -sSL https://get.haskellstack.org/ | sh && \
        cd /opt && git clone --depth 1 https://github.com/zachjs/sv2v.git && \
        cd sv2v && make && \
        ln -sf /opt/sv2v/bin/sv2v /usr/local/bin/sv2v; \
    else \
        echo "Installing prebuilt sv2v (amd64)" && \
        cd /tmp && \
        wget -q https://github.com/zachjs/sv2v/releases/download/v0.0.13/sv2v-Linux.zip && \
        unzip -q sv2v-Linux.zip && \
        mv sv2v-Linux/sv2v /usr/local/bin/sv2v && \
        chmod +x /usr/local/bin/sv2v && \
        rm -rf sv2v-Linux sv2v-Linux.zip; \
    fi

# Install Python dependencies system-wide so they are available without
# activating a venv (PEP 668 override required on Ubuntu 23+).
RUN python3 -m pip install --break-system-packages --no-cache-dir \
    cocotb \
    pytest \
    numpy \
    opencv-python-headless \
    pyserial

USER vscode
