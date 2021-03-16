#!/bin/bash

NAME=7-2018-q2
GCC_URL=https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2?revision=bc2c96c0-14b5-4bb4-9f18-bceb4050fee7?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2018-q2-update

pushd .
cd $WORKSPACE
mkdir arm-gcc-toolchain
wget -O $WORKSPACE/arm-gcc-toolchain/gcc.tar.bz2 $GCC_URL
cd arm-gcc-toolchain
tar -jxf gcc.tar.bz2 --strip=1
popd
export PATH=$WORKSPACE/arm-gcc-toolchain/bin:$PATH

echo -e "ARM GCC Compiler Version:\n"
arm-none-eabi-gcc --version
