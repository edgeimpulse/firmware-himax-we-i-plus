# Edge Impulse firmware for Himax WE-I Plus

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the ST B-L475E-IOT01A development board. This device supports all Edge Impulse device features, including ingestion, remote management and inferencing.

## Requirements

**Hardware**

* [Himax WE-I Plus](https://www.sparkfun.com/products/17256) development board.

**Software**

You'll need:

* A valid ARC MetaWare license to build the firmware. You cannot build this firmware with GCC at the moment. You can compile either by:
    * Having a local install of [DesignWare ARC MetaWare Toolkit](https://www.synopsys.com/dw/ipdir.php?ds=sw_metaware). Make sure `ccac` is in your PATH, and that the licenses are in place.
    * Or, building with [Docker desktop](https://www.docker.com/products/docker-desktop).
* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation) - to flash the firmware.

## How to build (locally)

1. Clone this repository.
2. Create a build directory and initialize CMake:

    ```
    $ mkdir build
    $ cd build
    $ cmake ..
    ```

3. Build and link the application:

    ```
    $ make -j
    $ sh ../make-image.sh
    ```

## How to build (Docker)

1. Clone this repository.
1. Build the container:

    ```
    $ docker build -t himax-build .
    ```

1. Then set up your build environment:

    ```
    $ mkdir -p build
    $ docker run --rm -it -v $PWD:/app himax-build /bin/bash -c "cd build && cmake .."
    ```

1. And build and link the application:

    ```
    $ docker run --rm -it -v $PWD:/app:delegated -e SNPSLMD_LICENSE_FILE=27020@synopsys.edgeimpulse.com himax-build /bin/bash -c "cd build && make -j && sh ../make-image.sh"
    ```

    Where you'll have to replace `27020@synopsys.edgeimpulse.com` with your license server or license file.

## Flashing

You'll need the Edge Impulse CLI v1.12 or higher. Then flash the binary with:

```
$ himax-flash-tool --firmware-path image_gen_linux_v3/out.img
```
