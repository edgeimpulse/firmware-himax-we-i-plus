# firmware-himax-we1

Ingestion & inferencing firmware for the HiMax WE 1 target.

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
    $ docker run --rm -it -v $PWD:/app:delegated -e SNPSLMD_LICENSE_FILE=27020@10.0.58.32 himax-build /bin/bash -c "cd build && make -j && sh ../make-image.sh"
    ```

## Flashing

You'll need the Edge Impulse CLI v1.9.2 or higher. Then flash the binary with:

```
$ himax-flash-tool --firmware-path image_gen_linux_v3/out.img
```
