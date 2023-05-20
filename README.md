# Serial Mouse Blaster Firmware

This is the firmware for the [Serial Mouse Blaster](https://github.com/scrapcomputing/SerialMouseBlasterPCB) project's Raspberry Pi Pico.

The project converts a modern USB optical mouse into a serial mouse.

# Download

Binaries are available in the [releases](https://github.com/scrapcomputing/SerialMouseBlasterFirmware/releases). The file to download is the one with the `.uf2` extension.

Currently there are two images:
- `SerialMouseBlasterFirmware_revX.Y_0_90v.uf2` : 0.90v core voltage (best)
- `SerialMouseBlasterFirmware_revX.Y_1_00v.uf2` : 1.00v core voltage (in case 0.90v does not work for you)

# Serial mouse protocol
We are using just two of the serial port signals:
- Rx Data (Pin 2) : This sends the mouse data to the mouse driver
- RTS (Pin 7) : The mouse driver uses this for mouse detection

Currently we are emulating 2-button Microsoft serial mouse.
The protocol is very simple. For each mouse state update the mouse sends 3 bytes.
These 3 bytes encode the status of the Left/Right mouse buttons and the X/Y movement deltas.
The following awesome ascii art is a verbatim copy from [CuteMouse](https://cutemouse.sourceforge.net/) `docs/protocol.txt`:
```
Serial Microsoft mode: 1200 bps, 7 data bits, 1 stop bit, no parity

              1st byte          2nd byte          3rd byte
         +---------------+ +---------------+ +---------------+
         |0|1|L|R|Y|Y|X|X| |0|0|X|X|X|X|X|X| |0|0|Y|Y|Y|Y|Y|Y|
         +---------------+ +---------------+ +---------------+
              | | \ / \ /       \----+----/       \----+----/
              | |  |   |             |                 |
              | |  +---|-------------|---------+       |
              | |      +-----+       |         |       |
              | |           / \ /----+----\   / \ /----+----\
              | |          +---------------+ +---------------+
 Left Button -+ |          | | | | | | | | | | | | | | | | | |
Right Button ---+          +---------------+ +---------------+
(1 if pressed)                 X movement        Y movement


Movement values are 8-bit signed twos complement integers.
Positive movement value indicates motion to the right/downward.
```

The firmware receives data from the mouse with the help of [TinyUSB](https://docs.tinyusb.org).
Once the data is received, it encodes it using the microsoft protocol and sends the 3-byte packet.
This is done in a loop using a polling frequency of 40Hz.


# Mouse Detection / Reset

The mouse driver briefly lowers RTS and expects to receive character 'M' (0x4D) after about 14ms on the Rx pin.
This is for the two button Microsoft-compatible mouse. For more detailed information please refer to the mouse man page mouse(4).

The SerialMouseBlaster PCB connects the level-shifted RTS to the Pico's GPIO 6, and Rx to GPIO 4.

# Build dependencies
- [Pico SDK 1.5.0](https://github.com/raspberrypi/pico-sdk). Please adjust the `PICO_SDK_PATH` in `src/CMakeLists.txt`
- C++ 17 or later compiler
- CMake 3.13 or later
- Recent version of `make` or other build system supported by CMake

# Build instructions
- Create a build directory and `cd` into it: `mkdir build && cd build`
- `cmake ../src/`
- `make`. The firmware should be in `build/SerialMouseBlasterFirmware.uf2`

# CMake configuration options
- `-DPICO_FREQ` to force a frequency in KHz. The default is `0` which uses a special routine that runs everything off the USB PLL at 48MHz.
- `-DPICO_VOLT` to set the core voltage. Accepted values can be found in `<pico-sdk>/src/rp2_common/hardware_vreg/include/hardware/vreg.h`. For example: `VREG_VOLTAGE_0_85` for 0.85V, `VREG_VOLTAGE_1_00` for 1.00v.
- `-DPICO_LED` `0` or `1`. If set to 0 disables the LEDs completely. The default consumes very little power, so overriding it is not very useful.
- `-DUSB_POLL_PERIOD` controls the poll period of the USB mouse. The default is 25ms which is 40Hz.
- `-DENABLE_SERIAL_DBG` `1` or `0`. Setting it to `1` enables serial debug output using the SerialMouseBlaster's debug output (connected to the Pico's pins 1 to 3).


# How to load the firmware

- Unplug the mouse from the SerialMouseBlaster PCB
- Press and hold the small button on the Pico
- While holding the button, connect the Pico to your PC with a micro-USB cable
- The Pico should show up as a mass-storage device
- Copy the `.uf2` firmware to the drive associated with the Pico
- Safely eject the mass-storage device

The Pico should boot and you should see the LED blinking very briefly (and faintly) every second or so.


# Serial port
The direction as relative to the PC. So Output means that the PC is sending data to the RS232 device.

Pin | Name | Direction | Description
----|------|-----------|------------
  1 | CD   | IN        | Carrier Detect
  2 | RXD  | IN        | Receive Data
  3 | TXD  | OUT       | Transmit Data
  4 | DTR  | OUT       | Data Terminal Ready
  5 | GND  |           | Ground
  6 | DSR  | IN        | Data Set Ready
  7 | RTS  | OUT       | Request To Send
  8 | CTS  | IN        | Clear To Send
  9 | IR   | IN        | Ring Indicator

```
Female connector looking at the holes:
 -----------------------
|     -------------     |
|     \ 5 4 3 2 1 /     |
| ( )  \ 9 8 7 6 /  ( ) |
|        -------        |
 -----------------------
```


# Raspberry Pi Pico's current consumption at 5V

We set the core voltage with `vreg_set_voltage()` and the clock with `set_sys_clock_khz()`.
The default core voltage is 1.10V.

Freq (MHz) | Core (V)    | Current (mA)
-----------|-------------|-------------
125        |  1.10       | 19.0
125        |  1.00       | 17.7
 48        |  1.10       | 10.6
 48        |  1.00       |  9.7
 48        |  0.90       |  8.9
 48*       |  1.10       |  8.9
 48*       |  1.00       |  8.1
 48*       |  0.90       |  7.4

(*) set_sys_clock_48mhz() which uses only the USB PLL.

The Default LED consumes about 2mA.

