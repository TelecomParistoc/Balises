#include "ch.h"
#include "chvt.h"
#include "chevents.h"
#include "chprintf.h"

#include "usbconf.h"
#include "exticonf.h"
#include "../shared/radioconf.h"
#include "../shared/decaplatform.h"
#include "../shared/decafunctions.h"
#include "nonvolatile.h"


// register the device sending messages when they are supposed to
uint8_t connectedDevices = 0;

//
int sendShellData(void) {
	// TODO
	return 0;
}

void parseRobotData(int robotID, int size) {
	(void) robotID;
	(void) size;
	// TODO
}

static THD_WORKING_AREA(waRadio, 512);
static THD_FUNCTION(radioThread, th_data) {
	event_listener_t evt_listener;
	int ret;

	(void) th_data;
	chRegSetThreadName("Radio");

	// initialize decawave module
	chEvtRegisterMask(&deca_event, &evt_listener, EVENT_MASK(0));
	if(decaInit() == 0) palSetLine(LINE_LED_BATT); // TODO : remove palSetLine

	while(1) {
		synchronizeOnSOF(deviceUID == BEACON1_ID);
		for(int i=1; i<FRAME_LENGTH; i++) {
			// if beacon is suppose to receive a message
			if(deviceUID & RXtimeTable[i]) {
				ret = messageReceive(i*TIMESLOT_LENGTH);
				// if it's a ranging message
				if(ret == 1 && radioBuffer[0] == 0x23 && deviceUID == RXtimeTable[i]) {
					messageAnswer(sendShellData()+2);
					connectedDevices |= TXtimeTable[i];
				} else if(ret > 1) { // if it's a robot data message
					parseRobotData(TXtimeTable[i], ret);
					connectedDevices |= TXtimeTable[i];
				} else {
					connectedDevices &= ~TXtimeTable[i];
				}
			}
		}
	}
}

void dumpConnectedDevices(BaseSequentialStream *chp, int argc, char **argv) {
	(void) argc;
	(void) argv;

	if(connectedDevices & BIGFOE_ID)
		chprintf(chp, "Big Foe   ... connected\n");
	else
		chprintf(chp, "Big Foe   ... NOT connected\n");
	if(connectedDevices & SMALLFOE_ID)
		chprintf(chp, "Small Foe ... connected\n");
	else
		chprintf(chp, "Small Foe ... NOT connected\n");
	if(connectedDevices & BIGBOT_ID)
		chprintf(chp, "Big bot   ... connected\n");
	else
		chprintf(chp, "Big bot   ... NOT connected\n");
	if(connectedDevices & SMALLBOT_ID)
		chprintf(chp, "Small bot ... connected\n");
	else
		chprintf(chp, "Small bot ... NOT connected\n");
}

void startRadio(void) {
	chThdCreateStatic(waRadio, sizeof(waRadio), NORMALPRIO+1, radioThread, NULL);
}
