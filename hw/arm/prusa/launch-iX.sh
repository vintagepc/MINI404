#!/bin/sh
# This script is an EXAMPLE and specific to my development setup. 
# I intend it only as a basic reference on how to bring up all the XL components together
# in the correct sequence.
# You will want to copy it to your binary output folder and adapt it to your needs. 
# Note - closing the bed will cause all other instances to be killed too.
MODBED="070"
echo "Launching iX w/ bed v ${MODBED}"
MAIN_FW="firmware.bin"
MODBED_BL="newbl/bootloader-v296-prusa_modular_bed-1.0.elf"
USBARGS="-drive id=usbstick,file=fat:rw:sd2 -device usb-storage,drive=usbstick"
rm bed.log
clear && ./qemu-system-buddy -machine prusa-iX-027c -kernel ${MAIN_FW} ${USBARGS} -chardev stdio,id=stm32_itm  -icount 1 -S -s & sleep 1 &&
screen -L -Logfile bed.log -SDm bed ./qemu-system-buddy -machine prusa-ix-bed-${MODBED} -kernel ${MODBED_BL} -icount 5 &\
read -p "Press any key to quit..." -n1 -e
killall qemu-system-buddy
