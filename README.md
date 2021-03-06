# Simulink Real-Time & Arduino for Sensing and Control #
This repository is created to develop DC motor control using Arduino and to bridge it to Simulink Real-time (SLRT). We plan to use Arduino for motor PID, to use Roboguia for encoder reading, and to use L298N or Sabertooth as amplifiers. We plan to use Ethernet (UDP or TCP/IP) for the communication between the Arduino and SLRT.

More documentation details can be found on the wiki associated with this repo.

This repo is part of an ongoing [development of a wrist](https://docs.google.com/document/d/18pi1abE7RSy7YfeVbhr9RNU76l_90QLvAo3EZrTzdI8/edit?ts=5a4e6b82).

### Arduino Programming ###

#### Ethernet Communication ####
* Doc of [Ethernet shield V1](https://www.arduino.cc/en/Main/ArduinoEthernetShieldV1), Doc of [Ethernet shield V2](https://www.arduino.cc/en/Guide/ArduinoEthernetShield) 
* test the Ethernet communication between Arduino and Matlab using ***streamVectorUDP.ino*** and ***test.m***
* get motor current state (position, running mode, running status)
* set desired position

#### Motor Interface ####
* Encoder Reading ([sample code for Robogaia](https://www.robogaia.com/3-axis-encoder-conter-arduino-shield.html))
* Analog write option 1: (PWM set) ([doc](https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/)). Also, for the Sabertooth amplifier, one needs an [RC filter](http://www.instructables.com/id/Analog-Output-Convert-PWM-to-Voltage/). 
* Analog write option 2: (DAC) [MCP4725](https://learn.adafruit.com/mcp4725-12-bit-dac-tutorial?view=all)
* Amplifier: currently we are using [Sabertooth 2x25 2007](https://www.dimensionengineering.com/datasheets/Sabertooth2x25.pdf). Newer version [Sabertooth 2x25 V2](https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf) should also work.
* PCB integration of 3-axes D/A channels, please refer to Seonghoon Noh's folder named ***Parker\_wrist\_DAC\_board***.

#### Sensor Interface ####

* [Read an array of time-of-flight sensors](https://github.com/wanglong06/slrt-arduino-sensing-control/tree/master/Arduino/Read_Multiple_ToF_Sensors)
* [Read soft potentiometer](https://github.com/wanglong06/slrt-arduino-sensing-control/tree/master/Arduino/Read_Soft_Pot)

#### PID controller ####
We use [Arduino PID Library](https://playground.arduino.cc/Code/PIDLibrary)

### Matlab Programming ###

#### Matlab Scripts ####

* ***UDP\_msgr***, "UDP messenger" is a class written for script line use. It includes functions ***send***, ***receiveDataMsg***, and ***receiveStringMsg***.
* run ***udp\_send\_receive/test.m*** to receive data from Aruino

#### Simulink Real-time Model ####

Under development.

#### SPI Conflicts between RoboGaia & Arduino Ethernet Shield ####

* Both RoboGaia and Ethernet shield need access to the Arduino SPI (Serial Peripheral Interface) library
* RoboGaia uses Arduino Pin ***D10***, ***D9***, ***D8***, for SS (Slave Select) purposes for the three encoder channels, see [RoboGaia spec](https://www.robogaia.com/uploads/6/8/0/9/6809982/robogaia_arduino_encoder_shield_schematics_v3.pdf) for details
* Arduino Ethernet Shield (both V1 and V2) uses Arduino Pin ***D10*** for SS (Slave Select), see [Ethernet spec](https://www.arduino.cc/en/Reference/Ethernet)
* In the case when both RoboGaia and Ethernet Shield attempt to access D10, RoboGaia is not working.
* ***CONCLUSION***: for now, we no longer use RoboGaia channel ***D10***.
* ***Future Fix***: we could change the select pin on the Ethernet shield, see the [GitHub wiki](https://github.com/kiwisincebirth/Arduino/tree/master/Ethernet), which comes from a [forum post](http://forum.arduino.cc/index.php?topic=217423.0)
* ***Update***: The SS pins for the Ethernet and encoder shields are hardwired, so SS pins cannot be reassigned in the code. The solution was to connect D10 pin on the Ethernet shield directly to the Arduino by bypassing the D10 pin on the encoder shield and rerouting the D10 pin on the encoder shield to the D7 pin on the Arduino.
