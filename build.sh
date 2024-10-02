#!/bin/bash

mkdir -p ${HOME}/.xtensa

# Check if xtensa-esp-elf is already installed
if [ ! -f ${HOME}/.xtensa/xtensa-esp-elf.tar.xz ]; then
    wget https://github.com/espressif/crosstool-NG/releases/download/esp-13.2.0_20230928/xtensa-esp-elf-13.2.0_20230928-x86_64-linux-gnu.tar.xz -O ${HOME}/.xtensa/xtensa-esp-elf.tar.xz
fi

# Check if xtensa-esp-elf-gdb is already installed
if [ ! -f ${HOME}/.xtensa/xtensa-esp-elf-gdb.tar.xz ]; then
    wget https://github.com/espressif/binutils-gdb/releases/download/esp-gdb-v14.2_20240403/xtensa-esp-elf-gdb-14.2_20240403-x86_64-linux-gnu.tar.gz -O ${HOME}/.xtensa/xtensa-esp-elf-gdb.tar.xz
fi

tar -xaf ${HOME}/.xtensa/xtensa-esp-elf.tar.xz -C ${HOME}/.xtensa
tar -xaf ${HOME}/.xtensa/xtensa-esp-elf-gdb.tar.xz -C ${HOME}/.xtensa

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

which ls
if [ $? -ne 0 ]; then
    sudo apt update && sudo apt install -y libncurses-dev ncurses-bin
fi

python3.7 -m venv ${HOME}/.xtensa/xtensa-esp-elf/venv
. ${HOME}/.xtensa/xtensa-esp-elf/venv/bin/activate
# ln -sf $(which python) ${HOME}/.xtensa/xtensa-esp-elf/bin/python

python -m pip install -r ${PWD}/requirements.txt

export PATH="${HOME}/.xtensa/xtensa-esp-elf/venv/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp-elf/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp-elf-gdb/bin:${PATH}"
export IDF_PATH=$(pwd)
. ${IDF_PATH}/add_path.sh

# Find all patches in the current directory and apply them
for patch in `ls *.patch`; do
    patch -N -r /dev/null -p0 < $patch
done

cd Projects/ESP32_E-Puck_2
bear -- make -j12
# cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -B build .
# cmake --build build
