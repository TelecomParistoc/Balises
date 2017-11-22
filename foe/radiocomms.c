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
#include <math.h>

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

void kalman() {
	double x = (double) radioData.x;
	double y = (double) radioData.y;

	// Compute H
	double H[3][2];
	double dist[3][1];
	dist[0][0] = -pow(pow(x, 2) + pow(y, 2), 0.5);
	H[0][0] = -x/dist[0][0];
	H[0][1] = -y/dist[0][0];
	dist[1][0] = -pow(pow(x - X2, 2) + pow(y, 2), 0.5);
	H[1][0] = (X2 - x)/dist[1][0];
	H[1][1] = -y/dist[1][0];
	dist[2][0] = -pow(pow(x - X3, 2) + pow(y - Y3, 2), 0.5);
	H[2][0] = (X3 - x)/dist[2][0];
	H[2][1] = (Y3 - y)/dist[2][0];

	// Compute S
	double Hprime[2][3];
	transposeMatrix(3, 2, H, Hprime);
	double QH[2][3];
	multiplyMatrices(2, 2, 3, Q, Hprime, QH);
	double tmp[3][3];
	multiplyMatrices(3, 2, 3, H, QH, tmp);
	double S[3][3];
	addMatrices(3, 3, tmp, R, S);

	// Compute K
	invert33Matrix(S, tmp);
	double K[2][3];
	multiplyMatrices(2, 3, 3, QH, tmp, K);

	// Compute X
	double Y[3][1];
	addMatrices(3, 1, D, dist, Y);
	double tmp2[2][1];
	multiplyMatrices(2, 3, 1, K, Y, tmp2);
	double X[2][1];
	addMatrices(2, 1, Q, tmp2, X);

	// TODO: compute P?

	// TODO: define R, D and Q

}

void invert33Matrix(double a[][3], double b[][3]) {
	double det = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
							 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
							 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

	b[0][0] = (a[1][1] * a[2][2] - a[2][1] * a[1][2]) / det;
	b[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det;
	b[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
	b[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det;
	b[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
	b[1][2] = (a[1][0] * a[0][2] - a[0][0] * a[1][2]) / det;
	b[2][0] = (a[1][0] * a[2][1] - a[2][0] * a[1][1]) / det;
	b[2][1] = (a[2][0] * a[0][1] - a[0][0] * a[2][1]) / det;
	b[2][2] = (a[0][0] * a[1][1] - a[1][0] * a[0][1]) / det;
}

void multiplyMatrices(int aRows, int innerDim, int bColumns, double a[][innerDim], double b[][bColumns], double c[][bColumns]) {
	int i, j, k;

	//initialize c to 0
	for (i=0;i<aRows;i++) {
		for (j=0;j<bColumns;j++) {
			c[i][j] = 0;
		}
	}

	//compute c
	for (i=0;i<aRows;i++) {
		for (j=0;j<bColumns;j++) {
			for (k=0;k<innerDim;k++) {
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

void transposeMatrix(int rows, int columns, double a[][columns], double b[][rows]) {
	int i, j;

	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			b[i][j] = a[j][i];
		}
	}
}

void addMatrices(int rows, int columns, double a[][columns], double b[][columns], double c[][columns]) {
	int i, j;

	for (i=0;i<rows;i++) {
		for (j=0;j<columns;j++) {
			c[i][j] = a[i][j] + b[i][j];
		}
	}
}

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
						printf("Distance: %u, frame = %u\r\n", distanceInMm, i);
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
