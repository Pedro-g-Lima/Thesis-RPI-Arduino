# Thesis-RPI-Arduino

## Overview

Code developed during master's Thesis to send actuator commands and receive sensor data via a Simulink environment.

## Files

This repository consists of the following files:

- [Arduino](https://github.com/Pedro-g-Lima/Thesis-RPI-Arduino/tree/main/Arduino), contains the code that runs continually on the slave board [Arduino Nano 33 BLE](https://store.arduino.cc/products/arduino-nano-33-ble). It continually sends [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit) and current measurement's data and receives actuator commands to/from the master [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) board, via the [I2C protocol](https://en.wikipedia.org/wiki/I%C2%B2C).

- [Matlab RPI System Blocks](https://github.com/Pedro-g-Lima/Thesis-RPI-Arduino/tree/main/Matlab%20RPI%20System%20Blocks), contains the [Matlab systems blocks](https://www.mathworks.com/help/simulink/slref/matlabsystem.html) that run the on target [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) board, and can be used to send actuator commands and receive sensor data using a Simulink environment.

  - [src](https://github.com/Pedro-g-Lima/Thesis-RPI-Arduino/tree/main/Matlab%20RPI%20System%20Blocks/src), folder contains the source code that runs on the [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) board and is used to fetch the sensors measurements data and send actuator commands from/to the [Arduino Nano 33 BLE](https://store.arduino.cc/products/arduino-nano-33-ble) board, or from the [Hall effect sensors](https://en.wikipedia.org/wiki/Hall_effect_sensor) directly connected to the master board.
  
  - [include](https://github.com/Pedro-g-Lima/Thesis-RPI-Arduino/tree/main/Matlab%20RPI%20System%20Blocks/include), folder contains the C-functions that are called by the [Matlab system blocks](https://www.mathworks.com/help/simulink/slref/matlabsystem.html).

  - Files with the `.m` extension are the custom [Matlab system blocks](https://www.mathworks.com/help/simulink/slref/matlabsystem.html) that can be used in Simulink to send actuator commands and receive sensor data.
  
  - [Blocos.slx](https://github.com/Pedro-g-Lima/Thesis-RPI-Arduino/blob/main/Matlab%20RPI%20System%20Blocks/Blocos.slx), example ready to use Simulink file containing the all the custom [Matlab system blocks](https://www.mathworks.com/help/simulink/slref/matlabsystem.html).
