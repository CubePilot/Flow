#!/bin/bash

export PATH=~/gcc-arm-none-eabi-6-2017-q2-update/bin/:$PATH

python2 -m pip uninstall -y pandas uavcan 
python2 -m pip install setuptools wheel virtualenv
python2 -m pip install pandas==0.21.0 tabulate natsort uavcan==1.0.0.dev32 crcmod empy
python2 -m pip install tokenize

rm -rf deploy
mkdir deploy

BOARDS="com.hex.flow_2.0 com.hex.flow_2.1"

for board in $BOARDS
do
    make clean && make -j12 BOARD_DIR=boards/$board
    cp build/$board/Flow_$board-crc.bin build/$board/Flow_$board-combined.bin build/$board/Flow_$board.elf build/$board/Flow_$board-bootloader.bin build/$board/Flow_$board-bootloader.elf deploy
done
