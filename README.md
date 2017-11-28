# Beacons
TelecomRobotics' real time locating system, based on UWB radio transceivers.

## Hardware
This code is designed to run on STM32F302C6T6 Cortex M4-based microcontroller.
The controller drives a Decawave DWM1000 transceiver for radio communications.
Documentation about the board can be found at
[github.com/TelecomParistoc/radioboard](https://github.com/TelecomParistoc/radioboard).

## Usage

### LED behaviour

**Sync LED (green):** On beacon 1, *ON* when at least one robot module is connected. On all others, *ON* when synchronized with beacon 1 (i.e. receives start-of-frame, see [protocol.md](https://github.com/TelecomParistoc/Beacons/blob/master/protocol.md)).  
**RX/TX LED (yellow):** blinks when transmitting or receiving data through the remote serial port.  
**Battery LED (red):** *ON* when battery voltage is low (when is drops bellow 4.5V, calibrated for 4xAA alkaline batteries).

## Radio Protocol
The protocol used in communications between modules is specified in [protocol.md](https://github.com/TelecomParistoc/Beacons/blob/master/protocol.md).
