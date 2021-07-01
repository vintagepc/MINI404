# Introduction

Mini404 is the counterpart to MK404 for Prusa's STM32-based Mini printer! It's still relatively young in its development and the ARM architecture and peripherals are much more complex to simulate. It is a spare time project but I am still actively developing features and capabilities where time permits. It's built on QEMU this time as that was the best ARM emulation platform I could find.

We feature many overlapping features with MK404, including the scripting engine, advanced arm-gdb debugging capabilities, and 3D printer visuals!

![](https://user-images.githubusercontent.com/53943260/99891868-8242ae00-2c3c-11eb-91fd-7bab7657e3ee.png)

![](https://raw.githubusercontent.com/wiki/vintagepc/MINI404/images/3D_model.png)

In addition, the use of QEMU gives us even more features to leverage:

- Simulated ethernet to the host

- Simulated or physical USB drive on the host computer

- Emulation save states so you can snapshot the machine state and resume it later

- Many more capabilities we don't explicitly use but are provided by QEMU, such as virtual consoles, host serial devices, pipes, and file-backed storage for EEPROMs and data. 

## Prusa Mini Features 

This is an overview of the current status of the Mini implementation. 

Name|Description|Status 
----|-----------|------
Bootloader/.bbf| Used to load stock images | ✔️ Works
CDC (USB) | Serial USB control | ❌ ⚠ Not working/in progress (#29). Use a workaround to re-route serial to UART 1 and then use the serial VC. 
EEPROM/flash | persistent storage|  ✔️ Works (`-pflash`, `-mtdblock`, see [Flash Storage Persistence](/vintagepc/MINI404/wiki/Flash-Storage-Persistence))
Ethernet | LAN connectivity | ✔️ Appears to work, see [Ethernet](/vintagepc/MINI404/wiki/Ethernet)
Fan| Simulated fan component |✔️ Works - with tachometer feedback
Heater| Simulated heater cartridge | ✔️ Works, feeds back temperature to thermistors.
IRSensor| IR-based MKxS sensor | ✔️ Works.
MINDA| Z-endstop sensor |  ⚠ Not implemented - currently using Z min "stallguard" as MINDA input.
Print[er] Visualization |Viewing motion/print| ✔️ Works - see [Advanced Visuals](/vintagepc/MINI404/wiki/Advanced-Visuals)
RotaryEncoder| Input knob | ✔️ Works
st25dv64k| I2C EEPROM chip | ✔️ Works 
st7789v | SPI Display driver | ✔️ Works 
Thermistor| Assorted temperature inputs | ✔️ Works 
TMC2209| Stepper driver | ✔️ Works ⚠ Not all registers/features implemented, many are transparent to the simulator.
w25q64jv| SPI flash chip | ✔️ Works 
USB | Flash drive for gcode input | ✔️ Works 

# Supported platforms

At current we only actively support Linux. While it should be possible to compile QEMU and probably the Mini404 componenets for Windows (MinGW64) or OSX, I do not currently have the time nor focus to handle this. Hopefully a native MinGW build will be available in the near future, however as I do not have OSX hardware or a development environment it is not something I can support nor maintain. 

See [Getting Started](https://github.com/vintagepc/MINI404/wiki/Getting-Started) for a primer on what you need to compile and how to get started. 

# More Reading

Head on over to the [Main Repo](https://github.com/vintagepc/MINI404/tree/MINI404) to get started, or...
... browse the [Wiki](https://github.com/vintagepc/MINI404/wiki) for soem more in-depth reading on features and how to use them. 

