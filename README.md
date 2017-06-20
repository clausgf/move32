# Naze32 for robotics!

This project runs on cheap, small and light STM32F1 based Naze32 
flight controller boards originally developed for radio controlled drones (multicopters). 
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

Without any adaptions, this project could serve as a robotics controller
or in an autonomous model car.


## Function

The servo output is primarily created from commands received via the
serial UART interface (see below). When Move32 does not receives not 
timely updates (timeout configurable per servo), the servo falls
back to a channel from the rc receiver. Fallback mode can also be
entered via a signal from rc receiver, so that you can take over
control of an autonomous vehicle by switch on your transmitter.

If input from the rc receiver is also not timely updated, 
e.g. because the receiver is out of range
or because you did not install a receiver in the first place
in a robotics application, the servo
is driven into a failsave position (configurable per servo).

The LEDs show you whether the system is alive and in fall back or 
fail safe mode. LED1 (green) provides a heartbeat signal to show 
that the system is still alive. 
LED0 (red) is on when all servo signals are provided from the host
via the serial interface,
off when at least one servo signal is in its failsafe states, and 
flashing otherwise when at least one servo signal has fallen back 
to the rc receiver.


## Serial protocol

- The data type for servo positions is the underlying pulse length in 
microseconds. Some values: 1000 (left) - 1500 (middle) - 2000 (right).

### Examples

#### Robot example
```
# configuration of fallback and failsafe (optional)
# failsafe positions for all servos
failsafe 0 1500
failsafe 1 1500
# fallback (and failsafe) after 200 ms
fallback_timeout 200

# cyclic communication starts here:
# set desired output (ignored if in fallback or failsafe mode)
servo 0 2000
servo 1 1500
# ... and again
servo 0 1900
servo 1 1500
# ...

# alternative for setting servos 1-4, others untouched
servos 2000 1500 1000 1100 0 0
```

#### Autonomous model car example

##### Autonomous mode controlled by the serial interface with rc fallback
```
# configuration (optional)
# configure default servo output from rx channel (for fallback)
rx_for_servo 0 0
rx_for_servo 1 1
# fallback after 200 ms
fallback_timeout 200
# fallback if rx channel 2 < 1500
fallback_force 2 0 1500
# fallback if rx channel 2 > 1500 or after 200 ms
fallback_force 2 1 1500
# failsafe positions for servos if rx fails
failsafe 0 1500
failsafe 1 1500
# optional: auto transmit received data
autorx 1

# cyclic communication starts here
# set desired output (ignored if in fallback or failsafe mode)
servos 1500 2000 0 0 0 0
# optional: get rx
rx
# ...
```

### Specification

tbd

## Compilation, Flashing & Setup

### Prerequisites
I use JetBrains CLion 2017.1 on MacOS for development, other operating systems
or IDEs or even vi should work as well.
I prepared my system in these steps:
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

Feedback, clarifications or improvements are welcome!
