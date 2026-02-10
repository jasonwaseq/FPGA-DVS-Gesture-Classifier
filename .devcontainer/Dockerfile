FROM mcr.microsoft.com/devcontainers/python:1-3.11

ENV LANG C.UTF-8
ARG INSTALL_ZSH="true" 
ARG TARGETARCH

# Remove Yarn APT repo to avoid GPG issues in base image
RUN rm -f /etc/apt/sources.list.d/yarn.list

RUN apt-get update
RUN apt-get -y install \
    wget \
    unzip \
    curl \
    git \
    build-essential \
    libgmp-dev \
    libusb-1.0-0 \
    udev \
    gtkwave \
    iverilog

#install oss-cad-suite
RUN cd /opt && \
    wget https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz && \
    tar -xf oss-cad-suite-linux-x64-20241121.tgz && \
    rm oss-cad-suite-linux-x64-20241121.tgz
    
ENV PATH="/opt/oss-cad-suite/bin:${PATH}"

# Make OSS CAD Suite discoverable by setup.py
RUN mkdir -p /home/vscode/Documents && \
    ln -s /opt/oss-cad-suite /home/vscode/Documents/oss-cad-suite && \
    chown -h vscode:vscode /home/vscode/Documents/oss-cad-suite

# Allow access to serial/USB devices for flashing and UART tools
RUN groupadd -f dialout && usermod -a -G dialout vscode

# install sv2v
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        echo "Installing prebuilt sv2v (amd64)"; \
        cd /opt && \
        wget https://github.com/zachjs/sv2v/releases/download/v0.0.13/sv2v-Linux.zip && \
        unzip sv2v-Linux.zip && \
        rm sv2v-Linux.zip; \
    else \
        echo "Building sv2v from source (arm64)"; \
        curl -sSL https://get.haskellstack.org/ | sh && \
        cd /opt && \
        git clone https://github.com/zachjs/sv2v.git && \
        cd sv2v && \
        make; \
    fi

ENV PATH="/opt/sv2v-Linux:${PATH}"
ENV PATH="/opt/sv2v/bin:${PATH}"

# install cocotb for simulation (PEP 668: allow system install)
RUN python3 -m pip install --break-system-packages cocotb pytest

USER vscode

