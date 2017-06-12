## Overwiev
This is a walking, four-legged robot. We made communication with its legs.

## Description
The whole project consists of:
- four-legged robot:
* Latte Panda board with Ubuntu
* Board with power supply unit and converters from USB to RS485, made by Dominik Belter
* every leg has three XM430-W210-R servomotors
- program, written in C++14:
* BoardGalgo class (based on Board class which we got from our supervisors), used to communicate with robot and sending commands like setPosition
* some functions used to show abilities of robot

## Tools
- Operating system: Linux Ubuntu 16.04.
- Editors: vim, QTCreator and Visual Studio Code
- Compiler: g++
- Language: C++14
- SSH protocol to communicate with computer on robot

## How to run
You should compile code using program make, and run it, using sudo command.

## How to compile
You should clone code from GitHub on Linux, run program cmake and compile code using program make.

## Future improvements
In the project, there are some bugs. The computer sometimes isn't able to communicate with servomotors, but there is a problem with Linux kernel (reported by some users on forums) or with badly written Dynamixel SDK. We made every effort to repair all bugs dependent on our code, so actually we havent't information about any bug in our program which we should repair.

## Attributions
We used following code:
- Dynamixl SDK (https://github.com/ROBOTIS-GIT/DynamixelSDK) - used to send and receive data to/from servomotors
- BoardDynamixel class from Dominik Belter, on which we took pattern
- Board class - parent class for our project

## License
MIT

## Credits
Daniel Staśczak<br />
Marcin Orczyk

## Additional informations
The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.<br />
Supervisors: Dominik Belter/Krzysztof Walas/Tomasz Mańkowski
