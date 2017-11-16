#include "ch.h"
#include "chvt.h"
#include "chevents.h"
#include "chprintf.h"

#include "usbconf.h"
#include "exticonf.h"
#include "../shared/radioconf.h"
#include "../shared/decaplatform.h"
#include "../shared/decafunctions.h"
#include "../shared/decadriver/deca_device_api.h"
#include "nonvolatile.h"
#include "remoteserial.h"
#include "radiocomms.h"

// register the device sending messages when they are supposed to
static uint8_t connectedDevices = 0;
// not zero if position tracing is active
static int showPosActive = 0;
// ID of the device we read data from (for serial or position)
uint8_t dataID = 0;
// distances to the beacons
int distances[3];
// information of the robot
struct robotData radioData;

void computeCoordinates() {
	double x, y;

	x = (distances[0]*distances[0]-distances[1]*distances[1]+X2*X2)/(2*X2);
	y = (distances[0]*distances[0]-distances[2]*distances[2]+X3*X3+Y3*Y3-2*X3*x)/(2*Y3);

	radioData.x = (int16_t)x;
	radioData.y = (int16_t)y;

	if(showPosActive)
		printf("x: %d, y: %d\r", (int16_t) x, (int16_t) y);
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

	// Set expected response's delay and timeout
	dwt_setrxaftertxdelay(POLL_TO_RESP_RX);
	dwt_setrxtimeout(RX_TIMEOUT);

	while(1) {
		synchronizeOnSOF(deviceUID == BEACON1_ID);

		for(int i=1; i<FRAME_LENGTH; i++) {
			// check for time to send message
			if(TXtimeTable[i] & deviceUID) {
				// check if we are in a data time slot
				// TODO: make it not hardcoded
				if (i == 4 || i == 8) {
					computeCoordinates();
					radioBuffer[0] = DATA_MSG;
					radioBuffer[1] = radioData.x;
					radioBuffer[2] = radioData.x >> 8;
					radioBuffer[3] = radioData.y;
					radioBuffer[4] = radioData.y >> 8;
					radioBuffer[5] = 0;

					// send data message
					ret = messageSend(i*TIMESLOT_LENGTH, 0, 6); // TODO: add payload
					if(ret == -4)
						printf("Transmission error, frame = %u\r\n", i);
					else if (ret == -1)
						printf("Reception error, frame = %u\r\n", i);
				}

				else {
					radioBuffer[0] = RANGE_MSG;
					// something else to put in radioBuffer here?

					// send ranging message
					ret = messageSend(i*TIMESLOT_LENGTH, 1, 1);
					if(ret == -4)
						printf("Transmission error, frame = %u\r\n", i);
					else if (ret == -1)
						printf("Reception error, frame = %u\r\n", i);
					// check frame is actually our response
					else if (radioBuffer[0] == RANGE_MSG) {
						int distanceInMm;
						int32_t tx_ts, rx_ts, beacon_rx_ts, beacon_hold_time;

						// Retrieve poll transmission and response reception timestamps
						tx_ts = dwt_readtxtimestamplo32();
						rx_ts = dwt_readrxtimestamplo32();

						// retrieve beacon RX timestamp
						beacon_rx_ts = (int) radioBuffer[1] + ((int) radioBuffer[2] << 8);
						// compute precise time between beacon poll RX and response TX
						beacon_hold_time = POLL_TO_RESP_DLY + TX_ANT_DLY - (beacon_rx_ts & 0x1FF);

						// compute distance
						distanceInMm = (rx_ts - tx_ts - beacon_hold_time) * 1000 / 2.0 * DWT_TIME_UNITS * SPEED_OF_LIGHT;
						printf("Distance = %u\r\n", distanceInMm);
					}
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
