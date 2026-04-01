# AUTONOMOUS SAILBOAT

### Sensor Team

* Corbin Barney (u1066089)
* Charbel Salloum (u1446840)
* Derek Tinoco (u1382366)
* Connor Stevens (u1463295)

### Antenna Team

* Harrison LeTourneau (u1460207)

## SCRIPTS

`./build.sh [-cm4|-cm7|-startup]`
* If you run the -startup flag it will configure the repo with all of the generated files and then build both processors.
* If you run with the -cm4 or -cm7 flags it will build both of those processors but will not perform the startup routine. You can declare both flags at the same time to build both binaries.

`./flash.sh [-cm4|-cm7|-b]`
* The -b flag will build the binary for the declared processors and then flash them.
* the -cm4 and -cm7 flags are used to flash to those specific processors. If both are declared the script will always flash the cm4 and then the cm7.