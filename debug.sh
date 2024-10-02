#!/bin/bash

port=/dev/ttyACM0
esptool="./components/esptool_py/esptool/esptool.py"
bootloader="./Projects/ESP32_E-Puck_2/build/bootloader/bootloader.bin"
partitions="./Projects/ESP32_E-Puck_2/build/partitions_singleapp.bin"
main="./Projects/ESP32_E-Puck_2/build/ESP32_E-Puck_2.bin"
elf="./Projects/ESP32_E-Puck_2/build/ESP32_E-Puck_2.elf"

export PATH="${HOME}/.xtensa/xtensa-esp-elf/venv/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp-elf/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp-elf-gdb/bin:${PATH}"
export IDF_PATH=$(pwd)
. ${IDF_PATH}/add_path.sh

xtensa-esp32-elf-gdb -x ./gdbinit ${elf}

# target remote :2331
# monitor reset
# load
# break main
# continue
