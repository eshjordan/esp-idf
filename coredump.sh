#!/bin/bash
export PATH="${HOME}/.xtensa/xtensa-esp-elf/venv/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp-elf/bin:${PATH}"
export PATH="${HOME}/.xtensa/xtensa-esp-elf-gdb/bin:${PATH}"
export IDF_PATH=$(pwd)
. ${IDF_PATH}/add_path.sh
# python3.7 ./components/espcoredump/espcoredump.py info_corefile -t b64 -c ./Projects/ESP32_E-Puck_2/core.dump ./Projects/ESP32_E-Puck_2/build/ESP32_E-Puck_2.elf
python3.7 ./components/espcoredump/espcoredump.py dbg_corefile -t b64 -c ./Projects/ESP32_E-Puck_2/core.dump ./Projects/ESP32_E-Puck_2/build/ESP32_E-Puck_2.elf
