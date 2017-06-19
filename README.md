# Naze32 for robotics!

This project is based on cheap, small and light STM32F1 based Naze32 
flight controller  board used in radio controlled drones (multicopters). 
It provides a simple to understand basis for creating real time software 
on the Naze32 and similar ARM Cortex based boards. It is intended for
controlling robots with multiple servos or autonomous model cars, but
could serve other purposes as well.

The project provides the following features:
- Cmake based build system from https://github.com/ObKo/stm32-cmake
- implementation of newlibc system calls for bare metal programming
- application of STHAL
- generation of pwm pulses for rc servos or speed controllers using
  timer channels in pwm mode
- decoding pulses from a rc receiver using timer channels 
  in input capture mode and timer interrupts
- communication via the serial port for debugging

In its current state, this project could serve as an example.
To make it really useful, you have to adapt it to your 
application:
- The software is currently configured for 6 servo outputs and 8 rx input 
channels. If you need more servo outputs (e.g. for your multi-arm robot),
your can convert any rx input to a servo output, i.e. you could control 
a maximum of 14 servos.
- In robotics, you would use your Naze32 to for i/o with exact timing.
High-level functions would typically be realized on a more powerful board
like a Raspberry Pi or a smartphone. The serial interface could be used
for communication, e.g. for processing rc inputs and setting servo output.
Other interfaces like I2C and SPI are available as well.


### Setup

#### Prerequisites
I use JetBrains CLion 2017.1 on MacOS for development, other operating systems
or IDEs (vi?) should work as well.
The preparation on my system consisted of the following steps:
- Get STHAL for your controller family. On my system, STM's CubeMX tool 
  automatically downloaded STHAL to ~/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0
- Get a cross toolchain for ARM. You might manually download your 
  version of the toolchain from 
  https://launchpad.net/gcc-arm-embedded .
  On my Mac, I got mine using Homebrew:
  ```bash
  # Install the cross toolchain using homebrew on MacOS
  brew install gcc-arm-embedded
  ```
- Get cmake. Again, I installed my copy using Homebrew:
  ```bash
  # Install cmake using homebrew on MacOS
  brew install cmake
  ```
- Get a tool for flashing the controller. I got stm32flash from 
  https://sourceforge.net/projects/stm32flash/
- Although not strictly necessary, terminal program (I used CoolTerm)
  will be useful for debugging.

#### Application
I assume that you have cloned this repository to a local directory.
1. Adapt the start of CMakeLists.txt to reflect your system settings
1. Build a hex file in a separate build directory:
   ```bash
   rm -rf build
   mkdir build
   cd build
   cmake ..
   make move32.hex
   ```
1. Flash hex hex file to your device:
   ```bash
   stm32flash -b 230400 -w move32.hex -v -g 0 /dev/tty.SLAB_USBtoUART
   ```

Positive feedback, clarifications or improvements are welcome!
