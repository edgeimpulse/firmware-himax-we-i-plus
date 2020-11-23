#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $SCRIPTPATH/build
cp app.elf ../image_gen_linux_v3
cp app.map ../image_gen_linux_v3

cd $SCRIPTPATH/image_gen_linux_v3

PATH=$PATH:$PWD

./image_gen -e app.elf -m app.map -o out.img
rm app.elf
rm app.map
