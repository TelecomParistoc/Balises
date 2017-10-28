#include "ch.h"
#include "hal.h"
#include "chevents.h"

#include "decaplatform.h"
#include "decafunctions.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "radioconf.h"

// RX/TX buffer
uint8_t radioBuffer[RADIO_BUF_LEN];

// timestamp of the last start-of-frame
static int64_t sofTS = -1;
static systime_t sofSystime = -1;

static void startOfFrameTime(int isSOFsender) {
	// get start-of-frame reception time
	sofTS = isSOFsender ? getTXtimestamp() : getRXtimestamp();
	sofSystime = chVTGetSystemTime();
}

void synchronizeOnSOF(int isSOFsender) {
	int ret;

	if(isSOFsender) {
		radioBuffer[0] = SOF_MSG;
		if(sofTS == -1) {
			decaSend(1, radioBuffer, 1, DWT_START_TX_IMMEDIATE);
		} else {
			messageSend(FRAME_LENGTH*TIMESLOT_LENGTH, 0, 1);
		}
		startOfFrameTime(1); // store SOF time
	} else {
		if(sofTS != -1) { // if we are already synchronized
			ret = messageReceive(FRAME_LENGTH*TIMESLOT_LENGTH);
			if(ret == 1 && radioBuffer[0] == SOF_MSG) // check we actually received a SOF
				startOfFrameTime(0); // if that the case store SOF time
			else // if SOF hasn't be received, resync
				sofTS = -1;
		} else {
			dwt_setrxtimeout(SYNC_RX_TIMEOUT);
			while(sofTS == -1) { // while we're not synchronized with SOF
				palClearLine(LINE_LED_SYNC);
				ret = decaReceive(RADIO_BUF_LEN, radioBuffer, DWT_START_RX_IMMEDIATE);
				if(ret < 0) // on timeout, allow module to cool down ten times longer
					chThdSleepMilliseconds(SYNC_RX_TIMEOUT/100);
				else if(ret == 1 && radioBuffer[0] == SOF_MSG) // check we actually received a SOF
					startOfFrameTime(0); // if that the case store SOF time
			}
			dwt_setrxtimeout(RX_TIMEOUT);
		}
	}
	palSetLine(LINE_LED_SYNC);
}

int messageReceive(uint64_t timeInFrame) {
	int ret;

	sleepUntil(sofSystime, timeInFrame - 1);
	dwt_setdelayedtrxtime((sofTS  + timeInFrame*MS_TO_DWT - AHEAD_OF_TX_MARGIN) >> 8);

	ret = decaReceive(RADIO_BUF_LEN, radioBuffer, DWT_START_RX_DELAYED);
	if(ret > 0)
		return ret;

	return -1;
}

int messageAnswer(int size) {
	// Retrieve poll reception timestamp
	uint64_t rxTS = getRXtimestamp();
	// set response message transmission time
	dwt_setdelayedtrxtime((rxTS + POLL_TO_RESP_DLY) >> 8);
	// send the response message
	radioBuffer[0] = rxTS;
	radioBuffer[1] = rxTS >> 8;
	return decaSend(size, radioBuffer, 1, DWT_START_TX_DELAYED);
}

int messageSend(int timeInFrame, int expectAnswer, int size) {
	sleepUntil(sofSystime, timeInFrame - 1);
	dwt_setdelayedtrxtime((sofTS  + timeInFrame*MS_TO_DWT) >> 8);

	// make sure TX done bit is cleared
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

	// send message and receive answer if expected
	if(expectAnswer) {
		if(decaSend(size, radioBuffer, 1, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) < 0)
			return -4;
		return decaReceive(RADIO_BUF_LEN, radioBuffer, NO_RX_ENABLE);
	} else if(decaSend(size, radioBuffer, 1, DWT_START_TX_DELAYED) < 0)
		return -4;

	return 0;
}
