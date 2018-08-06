# Move32 - Naze32 for robotics and autonomous model cars!

## Overview

In a robot or autonomous model car, high level functions are usually controlled by an embedded Linux system like a Raspberry Pi or a Nvidia Jetson. Unfortunately, such systems are not suited for hard realtime processing on 10 µs timescales as needed for controlling servos, ESCs or for decoding signals from a RC remote. This project solves this problem by offloading the hard realtime processing to a cheap, small and light weight STM32F1 based Naze32 flight controller originally developed for radio controlled drones (multicopters).

Move32 provides the following features:
- generation of precise pwm pulses (µs accuracy) for rc servos or speed controllers (ESCs)
- decoding pulses from a rc receiver using either individual wires per channel or a sum signal (CPPM)
- serial communications for host communications
- flexible host, fallback and failsafe modes for controlling servos/ESCs via the host for autonomous functions or via a remote for teaching/learning

TODO: Add a system context diagram.

TODO: Add a wiring diagram for the Naze32.

Technically, Move32 is written in C++, uses a Cmake based build system from [ObKo/stm32-cmake](https://github.com/ObKo/stm32-cmake), implements some newlibc system calls and otherwise relies on STM's STHAL library.

## Functions

Move32's servo output is primarily created from commands received via the serial UART interface from a host (e. g., a Raspberry Pi). When Move32 does not receive timely updates for a servo, it falls back to the remote's signal after a configurable timeout. Fallback mode can also be entered via a dedicated signal from the remote, so that you can take over control of an autonomous vehicle by a switch on your remote.

If Move32 is in fallback mode and the input from the remote is also not timely updated, the servo is driven into a configurable failsave position. E.g., this might happen when the receiver looses its signal because it is out of range. You might also have decided not to install a receiver in the first place.

The LEDs on the Naze32 board show you whether the system is alive and in host, fallback or failsafe mode. LED1 (green) provides a heartbeat signal to show that the system is still alive. LED0 (red) is off when all servo signals are provided from the host via the serial interface, solid when at least one servo signal is in its failsafe state, and flashing otherwise when at least one servo signal has fallen back to the remote.


## Serial interface

- Parameters for the serial interface: 115200 Baud, 8N1
- Commands are not echoed while typing, but confirmed after hitting Enter. The status after entering a command is either _ok_ or _error_.
- Use `help` to show a list of commands, `dump` to display the current settings.
- The data type for servo positions is the underlying pulse
  length in microseconds.
  Values for most remotes: 1000 (left) - 1500 (middle) - 2000 (right). Individual calibration is recommended.


### Examples

Please do not enter the comment lines marked by `#`. Output of the Move32 is not shown.

#### Robot example (no remote or receiver)
```
# *** configuration of fallback and failsafe (optional) ***
# failsafe positions for all servos
failsafe_pulse 0 1500
failsafe_pulse 1 1500
# fallback (and failsafe) after 200 ms
fallback_timeout 200

# *** cyclic communication starts here ***
# set desired output (ignored if in fallback or failsafe mode)
servo 0 2000
servo 1 1500
# ... and again
servo 0 1900
servo 1 1500
# ...

# alternative for setting servos 1-4 to given values, others to failsafe
servos 2000 1500 1000 1100 0 0
```

#### Autonomous model car example

```
# *** configuration (optional) ***
# failsafe positions for servos if rx fails
failsafe_pulse 0 1000
failsafe_pulse 1 1500
# configure default servo output from rx channel (for fallback)
fallback_servo_rx 0 0
fallback_servo_rx 1 1
# fallback after 200 ms
fallback_timeout 200
# fallback if rx channel 2 < 1500 ...
fallback_force 2 0 1500
# ... or fallback if rx channel 2 > 1500
fallback_force 2 1 1500

# auto transmit remote's data on new data (usually every 20 ms)
# e.g. for teaching/learning or special functions ...
rx_auto 1
# ... or auto transmit received data periodically every 100 ms ...
rx_auto 100
# ... or do not transmit from the remote on demand only (default)
rx_auto 0

# *** cyclic communication starts here ***
# set desired output (ignored if in fallback or failsafe mode)
servos 1500 2000 0 0 0 0
# optional: get rx (if not in auto transmit mode)
rx
# ...
```

## Setup, Compilation & Flashing

### Development environment
I use JetBrains CLion 2018.2 or Atom on MacOS for development, other operating systems or IDEs should work as well.

This how I set up my development system:
- ~~Get STHAL for your controller family. On my system, STM's CubeMX tool
  automatically downloaded STHAL to ~/STM32Cube/Repository/STM32Cube_FW_F1_V1.6.1.~~ STHAL is included in the Github repository, no need to download it yourself.
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
- Although not strictly necessary, a terminal program (I used CoolTerm)
  will be useful for debugging.


### Build

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

## Adaptions

- The software is currently configured for 6 servo/ESC outputs and 8 rx input channels. If you need more servo outputs (e.g. for your multi-arm robot), your can convert any rx input to a servo output, i.e. you could control a maximum of 14 servos.

Feedback, clarifications or improvements are welcome!
