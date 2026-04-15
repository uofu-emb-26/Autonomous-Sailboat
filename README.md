# AUTONOMOUS SAILBOAT (WIRELESS BRANCH)

### Sensor Team

* Corbin Barney (u1066089)
* Charbel Salloum (u1446840)
* Derek Tinoco (u1382366)
* Connor Stevens (u1463295)

### Antenna Team

* Harrison LeTourneau (u1460207)
* Adam Welsh (u1456456)
* Jared Pratt (u1237327)
* Aliou Tippett (u1415075)

## SCRIPTS

`./build.sh [-cm4|-cm7|-startup]`

The build script can be used to setup the repo when it is first cloned or compile the individual processor .elf files. 

* After first cloning the repo run the build script with the -startup flag, it will install tools, configure projects, and build both processors. 
* If you run with the -cm4 or -cm7 flags it will build that processors but will not perform the startup routine. You can declare both flags at the same time to build both .elf files.

`./flash.sh [-cm4|-cm7|-b]`

The flash script can be used to build and flash the STM32 for a specific processor.

* The -b flag will build the binary for the declared processors and then flash them. If -b is omitted then the script will just flash, no build.
* the -cm4 and -cm7 flags are used to flash to those specific processors. If both are declared the script will always flash the cm4 and then the cm7.
