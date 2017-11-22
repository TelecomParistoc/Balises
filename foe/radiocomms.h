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

/* distances to the beacons */
extern int distances[];

/* last data received/to send */
extern struct robotData radioData;

/* ID of the device we read data from (for serial or position) */
extern uint8_t dataID;

/* initialize decawave module and start radio communication thread */
void startRadio(void);

/* compute the absolute coordinates of the robot from its measured distances to the beacons */
void computeCoordinates(void);

/* update the filtered data with the currently stored distances to the beacons */
void kalman(void);

/* put the result of a + b in c */
void addMatrices(int rows, int columns, double a[][columns], double b[][columns], double c[][columns]);

/* put the result of a^(T) in b */
void transposeMatrix(int rows, int columns, double a[][columns], double b[][rows]);

/* put the result of a * b in c */
void multiplyMatrices(int aRows, int innerDim, int bColumns, double a[][innerDim], double b[][bColumns], double c[][bColumns]);

/* put the result of a^(-1) in b */
void invert33Matrix(double a[][3], double b[][3]);

/* ###################### shell callbacks ###################### */

/* show connected devices, USAGE : list */
void dumpConnectedDevices(BaseSequentialStream *chp, int argc, char **argv);

/* show postion of a robot, USAGE : {bbpos | sbpos | bfpos | sfpos} */
void showPositionBig(BaseSequentialStream *chp, int argc, char **argv) ;
void showPositionSmall(BaseSequentialStream *chp, int argc, char **argv);
void showPositionBigFoe(BaseSequentialStream *chp, int argc, char **argv);
void showPositionSmallFoe(BaseSequentialStream *chp, int argc, char **argv);

#endif
