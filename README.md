# Edge Impulse firmware for Himax WE-I Plus

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the ST B-L475E-IOT01A development board. This device supports all Edge Impulse device features, including ingestion, remote management and inferencing.

## Requirements

**Hardware**

* [Himax WE-I Plus](https://www.sparkfun.com/products/17256) development board.

**Software**

* You'll need a build toolchain, either:
    * [ARC GNU Toolchain](https://github.com/foss-for-synopsys-dwc-arc-processors/toolchain/releases/download/arc-2020.09-release/arc_gnu_2020.09_prebuilt_elf32_le_linux_install.tar.gz) (free).
    * [DesignWare ARC MetaWare Toolkit](https://www.synopsys.com/dw/ipdir.php?ds=sw_metaware) (paid):
        * A valid ARC MetaWare license is required to build the firmware.
        * Having a local install of [DesignWare ARC MetaWare Toolkit](https://www.synopsys.com/dw/ipdir.php?ds=sw_metaware). Make sure `ccac` is in your `PATH`, and that the licenses are in place.
    * Or, building with [Docker desktop](https://www.docker.com/products/docker-desktop).
* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation) - to flash the firmware.

## Building with the GNU Toolchain

### How to build (locally)

1. Clone this repository.
2. Create a build directory and initialize CMake:

    ```
    $ mkdir build-gnu
    $ cd build-gnu
    $ cmake .. -DCMAKE_TOOLCHAIN_FILE=toolchain.gnu.cmake
    ```

3. Build and link the application:

    ```
    $ make -j
    $ sh ../make-image.sh GNU
    ```

### How to build (Docker)

1. Clone this repository.
1. Build the container:

    ```
    $ docker build -t himax-build-gnu -f Dockerfile.gnu .
    ```

1. Then set up your build environment:

    ```
    $ mkdir -p build-gnu
    $ docker run --rm -it -v $PWD:/app himax-build-gnu /bin/bash -c "cd build-gnu && cmake .. -DCMAKE_TOOLCHAIN_FILE=toolchain.gnu.cmake"
    ```

1. And build and link the application:

    ```
    $ docker run --rm -it -v $PWD:/app:delegated himax-build-gnu /bin/bash -c "cd build-gnu && make -j && sh ../make-image.sh GNU"
    ```

## Building with ARC MetaWare

### How to build (locally)

1. Clone this repository.
2. Create a build directory and initialize CMake:

    ```
    $ mkdir build-mw
    $ cd build-mw
    $ cmake .. -DCMAKE_TOOLCHAIN_FILE=toolchain.metaware.cmake
    ```

3. Build and link the application:

    ```
    $ make -j
    $ sh ../make-image.sh MW
    ```

### How to build (Docker)

1. Clone this repository.
1. Build the container:

    ```
    $ docker build -t himax-build-mw -f Dockerfile .
    ```

1. Then set up your build environment:

    ```
    $ mkdir -p build-mw
    $ docker run --rm -it -v $PWD:/app himax-build-mw /bin/bash -c "cd build-mw && cmake .. -DCMAKE_TOOLCHAIN_FILE=toolchain.metaware.cmake"
    ```

1. And build and link the application:

    ```
    $ docker run --rm -it -v $PWD:/app:delegated -e SNPSLMD_LICENSE_FILE=27020@synopsys.edgeimpulse.com himax-build-mw /bin/bash -c "cd build-mw && make -j && sh ../make-image.sh MW"
    ```

    Where you'll have to replace `27020@synopsys.edgeimpulse.com` with your license server or license file.


## Flashing

You'll need the Edge Impulse CLI v1.12 or higher. Then flash the binary with:

```
$ himax-flash-tool --firmware-path image_gen_linux_v3/out.img
```

### Images larger than 1MB

Images larger than 1MB will be automatically split into 1MB images (`out_0.img` and `out_1.img`). Note however that it is required that Himax's bootloader is v.1.4.4 or greater as older bootloader versions do not suppport flashing multiple images. Follow the [instructions to update the bootloader](https://github.com/HimaxWiseEyePlus/bsp_tflu/tree/master/HIMAX_WE1_EVB_user_guide#update-bootloader-version-at-linux-environment).

Then to flash the images:
```
$ himax-flash-tool --firmware-path image_gen_linux_v3/out_0.img
$ # follow instructions
$ himax-flash-tool --firmware-path image_gen_linux_v3/out_1.img
$ # follow instructions
```

`
