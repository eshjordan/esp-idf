#/bin/bash

port=/dev/ttyACM1
if [ "$port" = "" ]; then
    echo "Robot not detected, be sure the robot is connected to the computer"
    exit
fi
echo "Robot detected on port $port";

esptool="./components/esptool_py/esptool/esptool.py"
bootloader="./Projects/ESP32_E-Puck_2/build/bootloader/bootloader.bin"
partitions="./Projects/ESP32_E-Puck_2/build/partitions_singleapp.bin"
main="./Projects/ESP32_E-Puck_2/build/ESP32_E-Puck_2.bin"

python3.7 $esptool --chip esp32 --port $port --baud 230400 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 $bootloader 0x10000 $main 0x8000 $partitions
