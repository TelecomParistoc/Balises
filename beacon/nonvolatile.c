#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "chprintf.h"

#include "nonvolatile.h"
#include "../shared/flash.h"

// ID of the beacon
volatile uint8_t deviceUID __attribute__((section(".flashdata")));

/* set ID of the device (write it in flash).
* USAGE: setid <NEW ID> */
void setDeviceUID(BaseSequentialStream *chp, int argc, char **argv) {
	int ret, newDeviceUID;

	(void) chp;
	(void) argc;
	(void) argv;

	if(argc != 1) {
		chprintf(chp, "USAGE: setid <NEW ID>\n");
		return;
	}

	// erase page
	chSysLock();
	ret = flashPageErase(FLASHDATA_PAGE);
	chSysUnlock();
	if(ret) {
		chprintf(chp, "Couln't erase flash.\n");
		return;
	}

	// write deviceUID in flash from deviceUIDinRAM
	newDeviceUID = atoi(argv[0]);
	chSysLock();
	ret = flashWrite((flashaddr_t) &deviceUID, (char*) &newDeviceUID, sizeof(deviceUID));
	chSysUnlock();
	if(ret) {
		chprintf(chp, "Couln't write flash.\n");
		return;
	}

	chprintf(chp, "OK\n");
}

/* print ID of the device
* USAGE: getid */
void getDeviceUID(BaseSequentialStream *chp, int argc, char **argv) {
	(void) argc;
	(void) argv;

	chprintf(chp, "Device ID = %d\n", deviceUID);
}
