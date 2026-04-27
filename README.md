# AUTONOMOUS SAILBOAT

For our capstone we are building a semi-autonomous sail boat capable of navigating between a series of way points set by an operator. This will include sensor integration, path planning, wirless communication, and mechanical operation of the boat. For our Embedded class we split into 2 teams in order to handle the sensor integration and wireless communication.

This project runs on the STM32H7 Nucleo-144. This board was chosen for its dual-core, floating point unit, and high clock rate.

### Sensor Team (refs/heads/main-sensor)

Sensor team was in charge of sensor integration. We combined a servo, IMU, magnetometer, GPS, and wind vain. In the final boat, servos will be used as the control mechanisms, the magnetometer, IMU, and GPS provide position and heading for waypoint navigation, the wind vane is used to correctly position the sail for optimal lift when on the water.

* Corbin Barney (u1066089)
* Charbel Salloum (u1446840)
* Derek Tinoco (u1382366)
* Connor Stevens (u1463295)

### Antenna Team

* Harrison LeTourneau (u1460207)
* Adam Welsh (u1456456)

## SCRIPTS

`./build.sh [-cm4|-cm7|-startup]`

The build script can be used to setup the repo when it is first cloned or compile the individual processor .elf files. 

* After first cloning the repo run the build script with the -startup flag, it will install tools, configure projects, and build both processors. 
* If you run with the -cm4 or -cm7 flags it will build that processors but will not perform the startup routine. You can declare both flags at the same time to build both .elf files.

`./flash.sh [-cm4|-cm7|-b]`

The flash script can be used to build and flash the STM32 for a specific processor.

* The -b flag will build the binary for the declared processors and then flash them. If -b is omitted then the script will just flash, no build.
* the -cm4 and -cm7 flags are used to flash to those specific processors. If both are declared the script will always flash the cm4 and then the cm7.

## DEBUG

This STM32 is equiped with a virtual serial port that lets you connect to its serial debug port directly over the USB you use to flash, no extra nonsense needed. To run it you need to use the following commands.

linux
```bash
screen $(ls /dev/ttyACM* | head -1) 115200 # To start the screen
# To stop the screen click "k".
```