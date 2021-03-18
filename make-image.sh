#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

get_build () {
    cd $1
    rm -f ../image_gen_linux/*.elf
    rm -f ../image_gen_linux/*.map
    cp *.elf ../image_gen_linux
    cp *.map ../image_gen_linux
    cd $SCRIPTPATH/image_gen_linux
    PATH=$PATH:$PWD
}

if [ "$2" = "--no-esc" ];
then
    FORMAT=""
    END_FORMAT="\n"
else
    FORMAT="\033[1m"
    END_FORMAT="\033[0m\n"
fi

if [ "$1" = "GNU" ];
then
    get_build $SCRIPTPATH/build-gnu
    printf $FORMAT"GNU Image Gen Tool"$END_FORMAT
    ./image_gen_gnu -e *.elf -o out.img -s 1024
elif [ "$1" = "MW" ];
then
    get_build $SCRIPTPATH/build-mw
    printf $FORMAT"Metaware Image Gen Tool"$END_FORMAT
    ./image_gen -e *.elf -m *.map -o out.img -s 1024
else
    echo "Invalid arguments. Usage: ./make-image.sh [ GNU | MW ]"
fi

rm *.elf
rm *.map
