# ALAN — Autonomous Long Range Aquatic Navigation Vessel
## LoRa Communications Subsystem

ALAN is a senior capstone project implementing a long-range wireless communication link between a boat-mounted STM32H755 microcontroller and a shore-based SparkFun SAMD21 ground station, using LoRa radio modulation at 915 MHz.

This subsystem handles all bidirectional telemetry and command communication between the vessel and ground station. The boat transmits state information (LED status, servo angle) to the ground station at 1Hz, and the ground station operator can send real-time commands to control the vessel.

---

## Team

### Sensor Team
- Corbin Barney (u1066089)
- Charbel Salloum (u1446840)
- Derek Tinoco (u1382366)
- Connor Stevens (u1463295)

### Antenna Team
- Harrison LeTourneau (u1460207)
- Adam Welsh (u1456456)
- Jared Pratt (u1237327)
- Aliou Tippett (u1415075)

---

## System Overview

```
[Ground Station Laptop]
        |
   Python dashboard (Flask) or serial terminal
        |
   USB serial
        |
[SparkFun SAMD21 Pro RF]  ←── RFM95W built in
        |
   915 MHz LoRa (bidirectional)
        |
[RFM95W module] ←── wired via SPI1
        |
[STM32H755 Nucleo-144]
   CM4 core: LoRa comms (this repo)
   CM7 core: Sensor integration / path planning (separate subsystem)
        |
   Rudder servo
```

The STM32H755 is a dual-core processor. The CM4 core exclusively handles LoRa communication. The CM7 core runs a separate sensor integration and path planning subsystem. Both cores must be flashed for the board to operate correctly — the CM7 releases a hardware semaphore that allows the CM4 to exit stop mode and begin executing.

## Hardware Requirements

**Boat Side**
- STM32H755ZI-Q Nucleo-144 development board
- RFM95W LoRa transceiver module wired to SPI1
- 915 MHz antenna
- UART serial adapter for monitoring incoming/outgoing messages
- Servo and motor board with appropriate battery

**Ground Station**
- SparkFun SAMD21 Pro RF (has RFM95W onboard)
- 915 MHz antenna
- USB cable

---


## Wiring — STM32 Nucleo to RFM95W
 
All connections are made to the ZIO connectors on the Nucleo-144 board. No soldering required — jumper wires only.
 
| RFM95W Pin | STM32 Pin | ZIO Location | Notes |
|------------|-----------|--------------|-------|
| SCK        | PA5       | CN7 pin 10   | SPI1 clock |
| MISO       | PA6       | CN7 pin 12   | SPI1 MISO |
| MOSI       | PB5       | CN7 pin 14   | SPI1 MOSI |
| NSS (CS)   | PD14      | CN7 pin 16   | Software controlled chip select |
| RESET      | PD15      | CN7 pin 18   | Active low reset |
| DIO0       | PG9       | CN10 pin 20  | TX done / RX done interrupt |
| 3.3V       | 3V3       | CN8 pin 7    | Power |
| GND        | GND       | CN8 pin 11   | Ground |

 
---
 
## LoRa Configuration
 
Both the STM32 and SAMD21 must use identical settings or they cannot hear each other.
 
| Parameter | Value |
|-----------|-------|
| Frequency | 915 MHz (US ISM band) |
| Spreading Factor | SF7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| TX Power | 17 dBm |
| Header Mode | Explicit |
| CRC | Enabled |
 
At SF7/125 kHz each packet has approximately 70 ms airtime. Transmitting at 1 Hz gives roughly 7% duty cycle — well within US ISM band limits, which impose no duty cycle restrictions. Expected range on open water with clear line of sight: **10–20 km**.
 
---
 
## Packet Format
 
### Boat → Ground (1 Hz telemetry)
 
The STM32 sends a framed packet over LoRa:
 
| Byte | Field | Value |
|------|-------|-------|
| 0 | Destination address | `0xAA` (SAMD21) |
| 1 | Source address | `0xBB` (STM32) |
| 2 | Sequence number | 0–255, wraps |
| 3 | Payload length | N bytes |
| 4..N+3 | Payload | ASCII string |
 
The payload is an ASCII string in the format:
```
STATUS,<angle>,<leds>
```
Example: `STATUS,-20,5`
 
- `angle` — rudder angle in degrees, range -45 to +45
- `leds` — bitmask: bit0 = green, bit1 = red, bit2 = yellow
### Ground → Boat (commands)
 
Single ASCII character commands sent from the SAMD21:
 
| Key | Action |
|-----|--------|
| `q` | Rudder +20° |
| `e` | Rudder -20° |
| `a` | Rudder +10° |
| `d` | Rudder -10° |
| `z` | Rudder +5° |
| `c` | Rudder -5° |
| `g` | Toggle green LED |
| `r` | Toggle red LED |
| `y` | Toggle yellow LED |
 
Rudder angle is clamped to ±45°.
 
---
 
## Software Requirements
 
- macOS (tested on Mac only)
- Xcode Command Line Tools:
```bash
xcode-select --install
```
- Homebrew dependencies:
```bash
brew install armmbed/formulae/arm-none-eabi-gcc ninja cmake
brew reinstall open-ocd
```
- PlatformIO (for SAMD21 ground station firmware):
```bash
pip install platformio
```
- Python dashboard dependencies:
```bash
pip install flask pyserial
```
 
---
 
## Scripts
 
### `./build.sh [-cm4 | -cm7 | -startup]`
 
Builds the STM32 firmware.
 
- `-startup` — Run once after first cloning. Installs tools, configures CMake, and builds both processors.
- `-cm4` — Builds the CM4 `.elf` only.
- `-cm7` — Builds the CM7 `.elf` only.
- Both flags can be combined: `./build.sh -cm4 -cm7`
> **Note:** The `-startup` flag attempts to install dependencies via `apt-get` which does not exist on macOS. Use the manual `brew` commands above instead, then run `./build.sh -cm4 -cm7` to build.
 
### `./flash.sh [-cm4 | -cm7 | -b]`
 
Flashes the STM32 over USB via OpenOCD.
 
- `-cm4` — Flashes the CM4 processor (Bank 2, `0x08100000`).
- `-cm7` — Flashes the CM7 processor (Bank 1, `0x08000000`).
- `-b` — Builds before flashing. Omit to flash an already-built binary.
- Both flags can be combined. **CM7 is always flashed before CM4.**
> **Note:** Always flash CM7 before CM4. The dual-core boot sequence requires the CM7 to release a hardware semaphore before the CM4 will exit stop mode and begin executing. Flashing CM4 alone will result in the CM4 hanging at startup.
 
---
 
## How To Operate
 
### First Time Setup
 
```bash
git clone <repo-url>
cd Autonomous-Sailboat
# install brew dependencies manually (see Software Requirements above)
./build.sh -cm4 -cm7
```
 
### Boat Side (STM32)
 
Flash both cores — CM7 first, then CM4:
 
```bash
./flash.sh -cm7
./flash.sh -cm4
```
 
Or build and flash in one step:
 
```bash
./flash.sh -b -cm7
./flash.sh -b -cm4
```
 
Once running, connect a UART serial adapter and open a terminal at **115200 baud, 8N1**. You will see output like:
 
```
[TX] STATUS,0,4
[RX] cmd len=1 byte0=0x71
[TX] STATUS,20,4
```
 
### Ground Station (SAMD21)
 
Switch to the ground station branch and follow its README:
 
```bash
git checkout SAMD21_LoRa
pio run -t upload
```
 
**Option 1 — Serial terminal:**
 
```bash
pio device monitor
```
 
Type single character commands directly into the terminal (`q`, `e`, `a`, `d`, `z`, `c`, `g`, `r`, `y`).
 
**Option 2 — Web dashboard:**
 
```bash
python3 dashboard.py <port> <baud_rate>
```
 
Example:
```bash
python3 dashboard.py /dev/cu.usbmodem1101 115200
```
 
Then open `http://localhost:8080` in a browser. The dashboard displays:
- Live rudder angle gauge
- LED status indicators (green, red, yellow)
- Packet sequence number and last update timestamp
- Clickable command buttons (keyboard shortcuts also work)
- Raw serial output from the SAMD21
