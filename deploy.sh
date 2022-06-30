#!/bin/bash

export PATH=~/gcc-arm-none-eabi-6-2017-q2-update/bin/:$PATH

pip2 install pandas==0.21.0 tabulate natsort uavcan crcmod

rm -rf deploy
mkdir deploy

BOARDS="com.hex.flow_2.0 com.hex.flow_2.1"
for board in $BOARDS
do
    make clean && make -j12 BOARD_DIR=boards/$board
    cp build/$board/Flow_$board-crc.bin build/$board/Flow_$board-combined.bin build/$board/Flow_$board.elf build/$board/Flow_$board-bootloader.bin build/$board/Flow_$board-bootloader.elf deploy
done
