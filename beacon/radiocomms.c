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
#include "remoteserial.h"

// register the device sending messages when they are supposed to
static uint8_t connectedDevices = 0;
// not zero if position tracing is active
static int showPosActive = 0;
// ID of the device we read data from (for serial or position)
uint8_t dataID = 0;

void parseRobotData(int senderID, int size) {
	(void) size;

	if(showPosActive)
		printf("x:%d, y: %d\r", *((int16_t*) &radioBuffer[1]), *((int16_t*) &radioBuffer[3]));
	// if there are remote serial data sent
	if(radioBuffer[5] > 0)
		receiveSerialData(&radioBuffer[6], radioBuffer[5], senderID);
}

static THD_WORKING_AREA(waRadio, 512);
static THD_FUNCTION(radioThread, th_data) {
	event_listener_t evt_listener;
	int ret;

	(void) th_data;
	chRegSetThreadName("Radio");

	// initialize decawave module
	chEvtRegisterMask(&deca_event, &evt_listener, EVENT_MASK(0));
	decaInit();

	while(1) {
		synchronizeOnSOF(deviceUID == BEACON1_ID);
		for(int i=1; i<FRAME_LENGTH; i++) {
			// skip useless data messages for the beacon
			if(RXtimeTable[i] != deviceUID && TXtimeTable[i] != dataID)
				continue;
			// if beacon is supposed to receive a message
			if(deviceUID & RXtimeTable[i]) {
				ret = messageReceive(i*TIMESLOT_LENGTH);
				// if it's a ranging message
				if(ret == 1 && radioBuffer[0] == RANGE_MSG && deviceUID == RXtimeTable[i]) {
					messageAnswer(sendSerialData(&radioBuffer[3], TXtimeTable[i])+3);
					connectedDevices |= TXtimeTable[i];
				} else if(ret > 1 && radioBuffer[0] == DATA_MSG) { // if it's a robot data message
					parseRobotData(TXtimeTable[i], ret);
					connectedDevices |= TXtimeTable[i];
				} else {
					connectedDevices &= ~TXtimeTable[i];
				}
			}
		}

		// for BEACON 1, update SYNC LED status
		if(deviceUID == BEACON1_ID)
			palWriteLine(LINE_LED_SYNC, connectedDevices != 0);
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

static void showPosition(int argc, uint8_t deviceID) {
	if(argc > 0) {
		printf("Too many arguments: Expected 0, got %d\n", argc);
		return;
	}
	dataID = deviceID;
	showPosActive = 1;

	// wait until a key is pressed to exit position tracing
	printf("Press any key to stop ...\r\n");
	chSequentialStreamGet(USBserial);
	// clear state variables
	showPosActive = 0;
	dataID = 0;
}
void showPositionBig(BaseSequentialStream *chp, int argc, char **argv) {
	(void) chp;
	(void) argv;
	showPosition(argc, BIGBOT_ID);
}
void showPositionSmall(BaseSequentialStream *chp, int argc, char **argv) {
	(void) chp;
	(void) argv;
	showPosition(argc, SMALLBOT_ID);
}
void showPositionBigFoe(BaseSequentialStream *chp, int argc, char **argv) {
	(void) chp;
	(void) argv;
	showPosition(argc, BIGFOE_ID);
}
void showPositionSmallFoe(BaseSequentialStream *chp, int argc, char **argv) {
	(void) chp;
	(void) argv;
	showPosition(argc, SMALLFOE_ID);
}

void startRadio(void) {
	chThdCreateStatic(waRadio, sizeof(waRadio), NORMALPRIO+1, radioThread, NULL);
}
