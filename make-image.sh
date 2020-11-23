#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
PROJECT=app

cd $SCRIPTPATH/build
cp ${PROJECT}.elf ../image_gen_linux_v3
cp ${PROJECT}.map ../image_gen_linux_v3

cd $SCRIPTPATH/image_gen_linux_v3

PATH=$PATH:$PWD

./image_gen -e ${PROJECT}.elf -m ${PROJECT}.map -o out.img
rm ${PROJECT}.elf
rm ${PROJECT}.map
