## Overview
This is a software for controlling a walking, four-legged Galgo robot.

## Description
The whole project consists of:
* Galgo robot with:
  * LattePanda board with installed Ubuntu Linux
  * Board with power supply unit and converters from USB to RS485, made by Dominik Belter
  * Four legs with three XM430-W210-R servomotors each
* Software, written in C++14:
  * BoardGalgo class (implementing Board interface which we got from our supervisors), used to communicate with robot for setting or reading specific options, like position or speed
  * Demos used to show abilities of robot

## Tools
- Operating system: Linux Ubuntu 16.04.02 LTS.
- Editors/IDE: Vim, QTCreator and Visual Studio Code
- Language: C++14
- Compiler: GNU G++
- Build system: CMake and GNU Make
- Controlling the robot: via SSH

## How to run
Run built program with sudo privileges. You can now select in the main menu one of four demos to run on robot.  
Robot can be controlled from any device supporting SSH protocol, for example from mobile phone.

## How to compile
Clone code from GitHub to the Linux and compile it using CMake + Make.

## Future improvements
The computer sometimes isn't able to communicate with servomotors, but it is a problem with Dynamixel SDK on more recent versions of Linux kernel (there are issues about it on the official repository). We tried our best to fix all the bugs dependent on our code, so currently there's no such bugs detected.

## Attributions
We used the following code:
- Dynamixl SDK (https://github.com/ROBOTIS-GIT/DynamixelSDK) - official library for communication with Dynamixel servomotors
- Board class - interface from Dominik Belter
- BoardDynamixel class - Board interface implementation for another robot, from Dominik Belter, on which we took pattern

We also got a lot of help from Przemysław Walkowiak, who told us a lot about using C++ templates for strongly-typed units conversion.

## License
MIT

## Credits
Daniel Staśczak  
Marcin Orczyk

## Additional informations
The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.  
Supervisors: Dominik Belter/Krzysztof Walas/Tomasz Mańkowski
