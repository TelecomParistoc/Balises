#ifndef RADIOCOMMS_H
#define RADIOCOMMS_H

#include "ch.h"

/* initialize decawave module and start radio communication thread */
void startRadio(void);

/* shell callback, USAGE : list */
void dumpConnectedDevices(BaseSequentialStream *chp, int argc, char **argv);

#endif
