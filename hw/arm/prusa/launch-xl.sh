#!/bin/sh
# This script is an EXAMPLE and specific to my development setup. 
# I intend it only as a basic reference on how to bring up all the XL components together
# in the correct sequence.
# You will want to copy it to your binary output folder and adapt it to your needs. 
# Note - closing the bed will cause all other instances to be killed too.
case $1 in
    v040)
        echo "Launching XL v0.4.0"
        XL="040"
        DWARF="040"
        MODBED="060"
        ;;
    v050)
        echo "Launching XL v0.5.0"
        XL="050"
        DWARF="060"
        MODBED="070"
        ;;
    *)
        echo "Unrecognized HW version $1."
        exit 1;
esac
MAIN_FW="public.bin"
MODBED_BL="newbl/bootloader-v296-prusa_modular_bed-1.0.elf"
DWARF_BL="newbl/bootloader-v296-prusa_dwarf-1.0.elf"
#ESP_SERIAL="-chardev serial,id=stm32uart7,path=/dev/ttyUSB1"
#ESP_SERIAL="-chardev socket,id=stm32uart7,path=/tmp/esp"
#ESP_SERIAL="-chardev pty,id=stm32uart7"
rm bed.log
rm e0.log
rm esp32.log
clear && ./qemu-system-buddy -machine prusa-xl-${XL} -kernel ${MAIN_FW} ${ESP_SERIAL} -chardev stdio,id=stm32_itm -drive id=usbstick,file=fat:rw:sd2 -device usb-storage,drive=usbstick -icount 1 -S -s & sleep 1 &&
# screen -SDm dwarf5 ./qemu-system-buddy -machine prusa-xl-extruder-g0-4 -kernel newbl/bootloader-v296-prusa_dwarf-1.0.elf -icount 5 &
# screen -SDm dwarf4 ./qemu-system-buddy -machine prusa-xl-extruder-g0-3 -kernel newbl/bootloader-v296-prusa_dwarf-1.0.elf -icount 5 &
# screen -SDm dwarf3 ./qemu-system-buddy -machine prusa-xl-extruder-g0-2 -kernel newbl/bootloader-v296-prusa_dwarf-1.0.elf -icount 5 &
# screen -SDm dwarf2 ./qemu-system-buddy -machine prusa-xl-extruder-g0-1 -kernel newbl/bootloader-v296-prusa_dwarf-1.0.elf -icount 5 &
screen -L -Logfile e0.log -SDm dwarf  ./qemu-system-buddy -machine prusa-xl-extruder-${DWARF}-0 -kernel ${DWARF_BL} -icount 5 &
sleep 2 && screen -L -Logfile bed.log -SDm bed ./qemu-system-buddy -machine prusa-xl-bed-${MODBED} -kernel ${MODBED_BL} -icount 4 \
& sleep 1 && screen -L -Logfile esp32.log -SDm esp32 ./qemu-system-xtensa -machine prusa-xl-esp32
killall qemu-system-buddy
killall qemu-system-xtensa
