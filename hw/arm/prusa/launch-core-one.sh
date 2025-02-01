#!/bin/sh
# This script is an EXAMPLE and specific to my development setup. 
# I intend it only as a basic reference on how to bring up all the XL components together
# in the correct sequence.
# You will want to copy it to your binary output folder and adapt it to your needs. 
# Note - closing the extension will cause all other instances to be killed too.
MAIN_FW="private.bin"
EXT_BL="bootloader-v1-prusa_xbuddy_extension-1.0.bin"
clear && ./qemu-system-buddy -machine prusa-core-one -kernel ${MAIN_FW} -chardev stdio,id=stm32_itm -drive id=usbstick,file=fat:rw:sd2 -device usb-storage,drive=usbstick -icount 1 -S -s & sleep 1 &&
sleep 2 && screen -SDm extension ./qemu-system-buddy -machine prusa-xbuddy-extension-05 -kernel ${EXT_BL} -icount 2
killall qemu-system-buddy
