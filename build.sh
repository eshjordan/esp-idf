#!/bin/bash

mkdir -p ${HOME}/.xtensa

# Check if xtensa-esp32-elf is already installed
if [ ! -f ${HOME}/.xtensa/xtensa-esp32-elf.tar.gz ]; then
    wget https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz -O ${HOME}/.xtensa/xtensa-esp32-elf.tar.gz
fi

tar -xzf ${HOME}/.xtensa/xtensa-esp32-elf.tar.gz -C ${HOME}/.xtensa

# Ensure python3.7 is used for compatibility
which python3.7
if [ $? -ne 0 ]; then
    sudo apt update && sudo apt install -y python3.7
fi

ln -sf /usr/bin/python3.7 ${HOME}/.xtensa/xtensa-esp32-elf/bin/python

export PATH="${HOME}/.xtensa/xtensa-esp32-elf/bin:${PATH}"
export IDF_PATH=$(pwd)
. ${IDF_PATH}/add_path.sh
cd Projects/ESP32_E-Puck_2
make
