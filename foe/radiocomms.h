#ifndef RADIOCOMMS_H
#define RADIOCOMMS_H

#include "ch.h"

/* container for data received/sent to master beacon */
struct robotData {
	// sent to the robot
	int16_t x;
	int16_t y;
	float xVect[2][1];
	uint8_t flags;
	// sent to master beacon
	uint8_t status;
};

/* last data received/to send */
extern struct robotData radioData;

/* distances to the anchors */
extern int distances[3];

extern float P[2][2];
extern float Q[2][2];
extern float R[3][3];
extern float D[3][1];

void kalman(void);

void invert33Matrix(float a[][3], float b[][3]);

void multiplyMatrices(int aRows, int innerDim, int bColumns, float a[][innerDim], float b[][bColumns], float c[][bColumns]);

void transposeMatrix(int rows, int columns, float a[][columns], float b[][rows]);

void addMatrices(int rows, int columns, float a[][columns], float b[][columns], float c[][columns], int subtract);

/* initialize decawave module and start radio communication thread */
void startRadio(void);

/* compute the absolute coordinates of the robot from its measured distances to the beacons */
void computeCoordinates(void);

#endif
