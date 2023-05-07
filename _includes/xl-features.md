
This is an overview of the current status of the XL implementation.

**Please note the XL is very much a work-in-progress at this time**

Name|Description|Status 
----|-----------|------
Bootloader/.bbf| Used to load stock images | ✔️ Works
CDC (USB) | Serial USB control | ❌ ⚠ Untested at this time.
EEPROM/flash | persistent storage|  ✔️ Works (see [Flash Storage Persistence](https://github.com/vintagepc/MINI404/wiki/Flash-Storage-Persistence))
Ethernet | LAN connectivity | ✔️ Should work, no different than Mk4/Mini
Fan| Simulated fan component |✔️ Works - ⚠ Readback isn't always correct in the firmware though.
GT911| Touchscreen | ✔️ Works - Single point touch via mouse inupt (not yet enabled in firmware).
Hall Sensor| Magnetic Filament Sensor | ⚠ Partial - side sensors work, extruder is not implemented yet.
Heater| Simulated heater cartridge | ✔️ Works, feeds back temperature to thermistors.
ili9488 | SPI Display driver | ✔️ Works 
LoadCell| Z-endstop sensor |  ✔️ Works for basic z-sensing, ⚠ more advanced uses not implemented.
Multi-tool | Multiple Dwarfs |  ✔️ ⚠ Works - tools are detected but calibration is not supported
Print[er] Visualization |Viewing motion/print| ❌ Not yet implemented, "lite" visuals may work but are untested.
RotaryEncoder| Input knob | ✔️ Works
st25dv64k| I2C EEPROM chip | ✔️ Works 
Selftest | General functionality | ❌ Not passing yet - some work is needed to support heater current measurement. 
Thermistor| Assorted temperature inputs | ✔️ Works 
TMC2130| Stepper driver | ✔️ Works ⚠ Not all registers/features implemented, many are transparent to the simulator.
w25q64jv| SPI flash chip | ✔️ Works 
ws281x| LED drivers | ✔️ Works (display LEDs), side RGB strips
USB | Flash drive for gcode input | ✔️ Works 
