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
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt install -y python3.7 python3.7-venv
fi

which bear
if [ $? -ne 0 ]; then
    sudo apt update && sudo apt install -y bear
fi

which flex
if [ $? -ne 0 ]; then
    sudo apt update && sudo apt install -y flex
fi

which bison
if [ $? -ne 0 ]; then
    sudo apt update && sudo apt install -y bison
fi

which gperf
if [ $? -ne 0 ]; then
    sudo apt update && sudo apt install -y gperf
fi

python3.7 -m venv ${HOME}/.xtensa/xtensa-esp32-elf/venv
. ${HOME}/.xtensa/xtensa-esp32-elf/venv/bin/activate
# ln -sf $(which python) ${HOME}/.xtensa/xtensa-esp32-elf/bin/python

python -m pip install -r ${PWD}/requirements.txt

export PATH="${HOME}/.xtensa/xtensa-esp32-elf/venv/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp32-elf/bin:${PATH}"
export IDF_PATH=$(pwd)
. ${IDF_PATH}/add_path.sh
cd Projects/ESP32_E-Puck_2
bear -- make
