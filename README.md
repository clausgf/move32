# About



The cmake based build system is from https://github.com/ObKo/stm32-cmake
- thanks a lot!


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
