FROM ubuntu:20.04

WORKDIR /app

RUN apt update && apt install -y wget build-essential

# Install recent CMake
RUN mkdir -p /opt/cmake && \
    cd /opt/cmake && \
    wget https://github.com/Kitware/CMake/releases/download/v3.19.0-rc3/cmake-3.19.0-rc3-Linux-x86_64.sh && \
    sh cmake-3.19.0-rc3-Linux-x86_64.sh --prefix=/opt/cmake --skip-license && \
    ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake && \
    rm /opt/cmake/cmake-3.19.0-rc3-Linux-x86_64.sh

# Grab toolchain
RUN mkdir -p /opt/arc_gnu && \
    cd /opt/arc_gnu && \
    wget https://cdn.edgeimpulse.com/build-system/himax.arc_gnu_ei_prebuilt_minimal_elf32_le_linux_install.2020.09.tar.gz && \
    tar xf himax.arc_gnu_ei_prebuilt_minimal_elf32_le_linux_install.2020.09.tar.gz && \
    rm -rf himax.arc_gnu_ei_prebuilt_minimal_elf32_le_linux_install.2020.09.tar.gz

# Add to PATH
ENV PATH="${PATH}:/opt/arc_gnu/arc_gnu_ei_prebuilt_minimal_elf32_le_linux_install/bin/"
