target extended-remote /dev/ttyACM0
mon select_mode 2
set serial baud 230400
set remote hardware-watchpoint-limit 2
monitor reset halt
flushregs
thb app_main
