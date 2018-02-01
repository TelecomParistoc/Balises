#ifndef RADIOCOMMS_H
#define RADIOCOMMS_H

#include "ch.h"

/* container for data received/sent to master beacon */
struct robotData {
	// sent to the robot
	int16_t x;
	int16_t y;
	uint8_t flags;
	// sent to master beacon
	uint8_t status;
};

/* last data received/to send */
extern struct robotData radioData;

/* distances to the anchors */
extern int16_t distances[3];

/* initialize decawave module and start radio communication thread */
void startRadio(void);

/* compute the absolute coordinates of the robot from its measured distances to the beacons */
void computeCoordinates(void);

#endif
