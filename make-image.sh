#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
PROJECT=app

cd $SCRIPTPATH/build
cp *.elf ../image_gen_linux_v3
cp *.map ../image_gen_linux_v3

cd $SCRIPTPATH/image_gen_linux_v3

PATH=$PATH:$PWD

./image_gen -e *.elf -m *.map -o out.img
rm *.elf
rm *.map
