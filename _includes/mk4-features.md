
This is an overview of the current status of the Mk4 implementation.

Name|Description|Status 
----|-----------|------
Bootloader/.bbf| Used to load stock images | ✔️ Works
CDC (USB) | Serial USB control | ❌ ⚠ Not working (#29). Use a workaround to re-route serial to UART 1 and then use the serial VC. There is no ETA on this as it requires integrating USBIP and "device mode" USB which is not particularly straightforward 
EEPROM/flash | persistent storage|  ✔️ Works (see [Flash Storage Persistence](https://github.com/vintagepc/MINI404/wiki/Flash-Storage-Persistence))
Ethernet | LAN connectivity | ✔️ Appears to work, see [Ethernet](https://github.com/vintagepc/MINI404/wiki/Ethernet)
Fan| Simulated fan component |✔️ Works - with tachometer feedback
GT911| Touchscreen | ✔️ Works - Single point touch via mouse inupt (not yet enabled in firmware).
Hall Sensor| Magnetic Filament Sensor | ✔️ Works.
Heater| Simulated heater cartridge | ✔️ Works, feeds back temperature to thermistors.
ili9488 | SPI Display driver | ✔️ Works 
LoadCell| Z-endstop sensor |  ✔️ Works.
Print[er] Visualization |Viewing motion/print| ❌ Not yet implemented, "lite" visuals may work but are untested.
RotaryEncoder| Input knob | ✔️ Works
st25dv64k| I2C EEPROM chip | ✔️ Works 
Thermistor| Assorted temperature inputs | ✔️ Works 
TMC2130| Stepper driver | ✔️ Works ⚠ Not all registers/features implemented, many are transparent to the simulator.
w25q64jv| SPI flash chip | ✔️ Works 
ws281x| LED drivers | ✔️ Works
USB | Flash drive for gcode input | ✔️ Works 
