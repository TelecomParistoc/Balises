#ifndef RADIOCOMMS_H
#define RADIOCOMMS_H

#include "ch.h"

/* ID of the device we read data from (for serial or position) */
extern uint8_t dataID;

/* initialize decawave module and start radio communication thread */
void startRadio(void);

/* ###################### shell callbacks ###################### */

/* show connected devices, USAGE : list */
void dumpConnectedDevices(BaseSequentialStream *chp, int argc, char **argv);

/* show postion of a robot, USAGE : {bbpos | sbpos | bfpos | sfpos} */
void showPositionBig(BaseSequentialStream *chp, int argc, char **argv) ;
void showPositionSmall(BaseSequentialStream *chp, int argc, char **argv);
void showPositionBigFoe(BaseSequentialStream *chp, int argc, char **argv);
void showPositionSmallFoe(BaseSequentialStream *chp, int argc, char **argv);

#endif
