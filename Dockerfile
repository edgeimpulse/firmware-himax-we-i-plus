FROM ubuntu:20.04

WORKDIR /app

RUN apt update && apt install -y wget build-essential

# Install recent CMake
RUN mkdir -p /opt/cmake && \
    cd /opt/cmake && \
    wget https://github.com/Kitware/CMake/releases/download/v3.19.0-rc3/cmake-3.19.0-rc3-Linux-x86_64.sh && \
    sh cmake-3.19.0-rc3-Linux-x86_64.sh --prefix=/opt/cmake --skip-license && \
    ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake && \
    rm cmake-3.19.0-rc3-Linux-x86_64.sh

# Grab Metaware toolkit and
# Install Metaware
RUN mkdir -p /tmp/metaware && \
    cd /tmp/metaware && \
    wget https://cdn.edgeimpulse.com/build-system/mw_devkit.bin && \
    cd /tmp/metaware && \
    chmod +x mw_devkit.bin && \
    ./mw_devkit.bin -i silent; \
    ([ $? -eq 1 ] && true) && \
    rm -rf /tmp/metaware

RUN apt install -y libtinfo5

# Add to PATH
ENV PATH="${PATH}:/root/ARC/MetaWare/arc/bin/"
