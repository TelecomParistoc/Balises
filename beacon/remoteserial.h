#ifndef REMOTESERIAL_H
#define REMOTESERIAL_H

#include "ch.h"

/* shell callback : open a remote serial on the big robot, USAGE : big */
void openRemoteSerialBig(BaseSequentialStream *chp, int argc, char **argv);

/* shell callback : open a remote serial on the small robot, USAGE : small */
void openRemoteSerialSmall(BaseSequentialStream *chp, int argc, char **argv);

/* when using remote serial, send data from the user to the robot
 * (called by radio loop in radiocomms.c) */
int sendSerialData(uint8_t* dataBuffer, int senderID);

/* when using remote serial, read data from the robot and write it to the serial
 * port (called by parseRobotData() in radiocomms.c) */
void receiveSerialData(uint8_t* dataBuffer, int size, int senderID);

#endif
