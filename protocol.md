# Radio protocol
This document describes the radio protocol used in Telecom Robotics' real time
locating system. This protocol is designed to be used with Decawave's DWM1000
modules.

## System components

The purpose of this system is to locate the robots from both teams in real time
during a game. It is composed of the following elements:

* 3 fixed beacons (B1, B2, B3)
* a module on each of our robots : BB (big bot) and SB (small bot)
* a module on each of the other team robots : BF (big foe) and SF (small foe)

## General principles

* Time is divided in **frames** of constant and pre-defined length.
* Each frame is divided into **time slots** (currently 2ms long), each containing
a message from a single sender and a response from a beacon in case of ranging.
* B1 (beacon 1) sends a **SOF (start-of-frame)** message to mark the beginning of a
time frame.
* The robots and other beacons **synchronize** by searching for SOF and sends messages
at specific time slots after that.

The sequence of messages in a time frame is fixed and defined as shown is the
table below.

*TX*: the module sends a message. <br>
*RX*: the module receives a message (only beacons can send back a response in
the same time slot, when ranging). <br>
*(RX)*: the beacon activate its receiver only if it has been configured so
(see remote serial port). <br>


| Time slot | Message | B1 | B2 | B3 | BB | SB | BF | SF |
|-----------|---------|----|----|----|----|----|----|----|
| 0         | SOF     | TX | RX | RX | RX | RX | RX | RX |
| 1         | ranging |    |    | RX |    |    | TX |    |
| 2         | ranging |    | RX |    |    |    | TX |    |
| 3         | ranging | RX |    |    |    |    | TX |    |
| 4         | data    |(RX)|(RX)|(RX)| RX | RX | TX |    |
| 5         | ranging |    |    | RX |    |    |    | TX |
| 6         | ranging |    | RX |    |    |    |    | TX |
| 7         | ranging | RX |    |    |    |    |    | TX |
| 8         | data    |(RX)|(RX)|(RX)| RX | RX |    | TX |
| 9         | ranging |    |    | RX | TX |    |    |    |
| 10        | ranging |    | RX |    | TX |    |    |    |
| 11        | ranging | RX |    |    | TX |    |    |    |
| 12        | data    |(RX)|(RX)|(RX)| TX | RX |    |    |
| 13        | ranging |    |    | RX |    | TX |    |    |
| 14        | ranging |    | RX |    |    | TX |    |    |
| 15        | ranging | RX |    |    |    | TX |    |    |
| 16        | data    |(RX)|(RX)|(RX)| RX | TX |    |    |


## Synchronisation mechanism

When modules start or recover from lost connection, they synchronize by searching
for a SOF message. When a valid SOF has been received, the module can send and
receive messages normally.

To avoid excessive heating of the modules, during sync the receiver is turned on
for up to 50ms, then sleeps for 500ms if no valid SOF was detected.


## Ranging mechanism

To measure the distance from a robot to a beacon, a single-sided two way ranging
is implemented (see DW1000 user manual, appendix 3) :

* the robot module sends a ranging message to a beacon, and save the value of the
ranging timer when the R marker is sent
* the beacon receives the message, and saves the value of the ranging timer
when the R marker is received
* the beacon sends the response after a known time has elapsed
* the robot module receives the response, save the value of the ranging timer
when the R marker is received and computes the time of flight

![twr-ss](/twr-ss.jpeg)

## Remote serial port

To allow communication with the robots in an environnement where WiFi doesn't work
properly (for example in the robotic cup), beacons can be configured as relay to
open a serial port wirelessly on the robots. The user is then able to use beacon's
serial port as if it were the robot's (with some data rate limitations).

Data from the user to the robot is sent when the beacon responds to the robot
ranging and data from the robot is sent in its *data* message.

## Messages

### General format

| Message ID | Payload ... |
|------------|-------------|

Each type of message has a different message ID.


### Start-of-frame (SOF)

Sent by B1 at the beginning of a frame, it sets the time origin for a frame.
Precise moments to listen to are computed from this reference by robot modules and other beacons.

| 0x50 |
|------|


### Ranging

Message sent by a robot module to do a distance measurement.

**Robot module send :**

| 0x23 |
|------|

**Response from beacon:**

| 0x23 | RX timestamp (2 LSbytes) | serial payload (0..20 bytes) |
| ---- | ------------------------ | ---------------------------- |

**RX timestamp :** required for ranging as message are sent ignoring the 9LSbits of the target TX time (see Decawave user manual) <br>
**serial payload:** serial data from the user (connected to the beacon)

### Data

Message sent from a robot module to send data to the beacons (for the remote
serial port) and to others robots.

| 0x32 | X (2 bytes) | Y (2 bytes) | serial Length | serial payload (0..50 bytes) | payload (0..50 bytes) |
| ---- | ----------- | ----------- | ------------- | ---------------------------- | --------------------- |

**X, Y:** current absolute coordinates of the robot (in cm, 16 bits signed)<br>
**serial length:** number of bytes from the remote serial port (always 0 for BF and SF) <br>
**serial payload:** serial data from the robot (from 0 to 50bytes) <br>
**payload:** generic payload to allow communication between the robots (from 0 to 50 bytes)
