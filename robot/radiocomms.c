#include "ch.h"
#include "chvt.h"
#include "chevents.h"

#include "exticonf.h"
#include "../shared/decafunctions.h"
#include "../shared/decadriver/deca_device_api.h"
#include "../shared/decadriver/deca_regs.h"
#include "../shared/radioconf.h"
#include "radiocomms.h"
#include "led.h"
#include "dance.h"
#include "imu.h"

// event triggered when new data has been received
EVENTSOURCE_DECL(radioEvent);

// measure the distance from the beacon to a robot and send some data if needed
static int rangeRobot(int robotUID, int dataLength) {
	int ret;

	// make sure TX done bit is cleared
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

	// Send poll, and enable reception automatically to receive the answer
	radioBuffer[0] = RANGING_MSG_ID;
	radioBuffer[1] = robotUID;
	decaSend(2 + dataLength, radioBuffer, 1, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	// receive response
	if((ret = decaReceive(RADIO_BUF_LEN, radioBuffer, NO_RX_ENABLE)) < 0)
		return 0;

	// check frame is actually our response
	if (radioBuffer[0] == RANGING_MSG_ID && radioBuffer[1] == 0) {
		int distanceInMm;
		int32_t tx_ts, rx_ts, beacon_rx_ts, beacon_hold_time;

		// Retrieve poll transmission and response reception timestamps
		tx_ts = dwt_readtxtimestamplo32();
		rx_ts = dwt_readrxtimestamplo32();

		// retrieve beacon RX timestamp
		beacon_rx_ts = (int) radioBuffer[2] + ((int) radioBuffer[3] << 8);
		// compute precise time between beacon poll RX and response TX
		beacon_hold_time = POLL_TO_RESP_DLY + TX_ANT_DLY - (beacon_rx_ts & 0x1FF);

		// compute distance
		distanceInMm = (rx_ts - tx_ts - beacon_hold_time) * 1000 / 2.0 * DWT_TIME_UNITS * SPEED_OF_LIGHT;

		return distanceInMm;
	}
	return 0;
}

static THD_WORKING_AREA(waRadio, 512);
static THD_FUNCTION(radioThread, th_data) {
	event_listener_t evt_listener;
	int ret;

	(void) th_data;
	chRegSetThreadName("Radio");

	// initialize decawave module
	decaInit();
	// Set expected response's delay and timeout
	dwt_setrxaftertxdelay(POLL_TO_RESP_RX);
	dwt_setrxtimeout(RX_TIMEOUT);

	chEvtRegisterMask(&deca_event, &evt_listener, EVENT_MASK(0));

	while(1) {
		// if last start-of-frame time isn't known or robot isn't registered
		if(sofTS == -1 || registered == 0) {
			synchronizeRadio();
		} else {
			if((ret = messageRead(SOF_MSG_ID, 0xFF, FRAME_LENGTH)) > 0)
				parseSOF(ret);

			// if something went wrong, restart synchronisation
			if(registered == 0 || ret < 0) {
				sofTS = -1;
				chThdSleepMilliseconds(FRAME_LENGTH - 4);
				continue;
			}
		}

		ret = messageRead(RANGING_MSG_ID, deviceID, (registered*3)*TIMESLOT_LENGTH);
		if(ret > 0) {
			parseRadioData();
			rangingResponse(1);

			// send radio event (new data available)
			chEvtBroadcastFlags(&radioEvent, EVENT_MASK(0));

			// process payload
			if(radioData.flags & RB_FLAGS_PTSTR)
				storeMoves(&radioBuffer[7], (ret - 7)/11);
			else if(radioData.flags & RB_FLAGS_CLSTR)
				storeColors(&radioBuffer[7], (ret - 7)/6);
			else if(radioData.flags & RB_FLAGS_WF) {
				saveIMUcalibration();
				writeStoredData();
				writeIMUcalibration();
			}
		} else {
			sofTS = -1;
		}

		if(messageRead(RANGING_MSG_ID, deviceID, (registered*3+1)*TIMESLOT_LENGTH) > 0) {
			rangingResponse(0);
		}
		if(messageRead(RANGING_MSG_ID, deviceID, (registered*3+2)*TIMESLOT_LENGTH) > 0) {
			rangingResponse(0);
		}
	}
}

float getDate(void) {
	float dateSinceSOF;

	if (sofSystime == 0xFFFFFFFF)
	 	return 0;

	dateSinceSOF = chVTTimeElapsedSinceX(sofSystime)*10/CH_CFG_ST_FREQUENCY;
	return date*512/499.2 + dateSinceSOF;
}

void startRadio(void) {
	chThdCreateStatic(waRadio, sizeof(waRadio), NORMALPRIO+1, radioThread, NULL);
}
