
Name|Description|Status 
----|-----------|------
Bootloader/.bbf| Used to load stock images | ✔️ Works
CDC (USB) | Serial USB control | ❌ ⚠ Not working (#29). Use a workaround to re-route serial to UART 1 and then use the serial VC. There is no ETA on this as it requires integrating USBIP and "device mode" USB which is not particularly straightforward 
EEPROM/flash | persistent storage|  ✔️ Works (`-pflash`, `-mtdblock`, see [Flash Storage Persistence](https://github.com/vintagepc/MINI404/wiki/Flash-Storage-Persistence))
Ethernet | LAN connectivity | ✔️ Appears to work, see [Ethernet](https://github.com/vintagepc/MINI404/wiki/Ethernet)
Fan| Simulated fan component |✔️ Works - with tachometer feedback
Heater| Simulated heater cartridge | ✔️ Works, feeds back temperature to thermistors.
IRSensor| IR-based MKxS sensor | ✔️ Works.
MINDA| Z-endstop sensor |  ✔️ Works.
Print[er] Visualization |Viewing motion/print| ✔️ Works - see [Advanced Visuals](https://github.com/vintagepc/MINI404/wiki/Advanced-Visuals)
RotaryEncoder| Input knob | ✔️ Works
st25dv64k| I2C EEPROM chip | ✔️ Works 
st7789v | SPI Display driver | ✔️ Works 
Thermistor| Assorted temperature inputs | ✔️ Works 
TMC2209| Stepper driver | ✔️ Works ⚠ Not all registers/features implemented, many are transparent to the simulator.
w25q64jv| SPI flash chip | ✔️ Works 
USB | Flash drive for gcode input | ✔️ Works 
