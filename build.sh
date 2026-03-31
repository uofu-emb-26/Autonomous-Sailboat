#!/bin/bash
set -e

TOOLCHAIN="$(pwd)/gcc-arm-none-eabi.cmake"

case "$1" in
    cm7)
        printf "\n===========================================================================\n"
        printf "Building CM7...\n"
        cmake --build CM7/build
        ;;
    cm4)
        printf "\n===========================================================================\n"
        printf "Building CM4...\n"
        cmake --build CM4/build
        ;;
    startup)
        printf "\n===========================================================================\n"
        printf "Installing dependencies...\n"
        sudo apt-get install -y gcc-arm-none-eabi ninja-build cmake openocd

        printf "\n===========================================================================\n"
        printf "Configuring CM7...\n"
        cmake --no-warn-unused-cli -B CM7/build \
              -DCMAKE_BUILD_TYPE=Debug \
              -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              CM7

        printf "\n===========================================================================\n"
        printf "Configuring CM4...\n"
        cmake --no-warn-unused-cli -B CM4/build \
              -DCMAKE_BUILD_TYPE=Debug \
              -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              CM4

        printf "\n===========================================================================\n"
        printf "Done! IntelliSense databases are at CM7/build/compile_commands.json and CM4/build/compile_commands.json\n\n"
        ;;
    *)
        echo "Usage: $0 [cm7|cm4|startup]"
        exit 1
        ;;
esac
