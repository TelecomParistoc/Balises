#ifndef NONVOLATILE_H
#define NONVOLATILE_H

#include "ch.h"

/* ID of the device, see defines in shared/radioconf.h */
extern uint8_t deviceUID;

/* ############## Shell command callbacks ############## */

/* set ID of the device (write it in flash).
 * USAGE: setid <NEW ID> */
void setDeviceUID(BaseSequentialStream *chp, int argc, char **argv);
/* print ID of the device
 * USAGE: getid */
void getDeviceUID(BaseSequentialStream *chp, int argc, char **argv);

#endif
