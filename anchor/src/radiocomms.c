#include "ch.h"
#include "chvt.h"
#include "chevents.h"
#include "chprintf.h"

#include "usbconf.h"
#include "exticonf.h"
#include "../shared/radioconf.h"
#include "../shared/decaplatform.h"
#include "../shared/decafunctions.h"
#include "../shared/decadriver/deca_regs.h"
#include "../shared/nonvolatile.h"
#include "remoteserial.h"

// register the device sending messages when they are supposed to
static uint8_t connectedDevices = 0;
// not zero if position tracing is active
static int showPosActive = 0;
// ID of the device we read data from (for serial or position)
uint8_t dataID = 0;

void parseRobotData(int senderID, int size) {
	(void) size;

	if(showPosActive) {
    for (int i = 0; i < 11; i++) {
      printf("%i,", *((int16_t*) &radioBuffer[2*i+1]));
    }
    printf("%i\r\n", *((int16_t*) &radioBuffer[23]));
  }

	// if there are remote serial data sent
	if(radioBuffer[25] > 0)
		receiveSerialData(&radioBuffer[26], radioBuffer[25], senderID);
}

static void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
  for (int i = 0; i < 4; i++) {
    ts_field[i] = (uint8_t) ts;
    ts >>= 8;
  }
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
		for(int i = 1; i < FRAME_LENGTH; i++) {
      if(deviceUID & TXtimeTable[i]) {
        radioBuffer[0] = POLL_MSG;

        messageSend(timeslotTable[i], 0, 1);

        uint64_t poll_tx_ts, resp_rx_ts;
        uint8_t receiveBuffer[RADIO_BUF_LEN];
        dwt_setrxtimeout((uint16_t) (1.5*POLL_RX_TO_RESP_TX_DLY_UUS));

        for (int j = 0; j < 3; j++) {
          int ret = decaReceive(RADIO_BUF_LEN, receiveBuffer, DWT_START_RX_IMMEDIATE);
          printf("%u\r\n", receiveBuffer[0]);

          if (ret > 0 && receiveBuffer[0] == RESP_MSG) {
            connectedDevices |= (1 << (j+4));

            /* Retrieve poll transmission and response reception timestamp. */
            poll_tx_ts = getTXtimestamp();
            resp_rx_ts = getRXtimestamp();

            /* Write all timestamps in the final message. */
            final_msg_set_ts(&radioBuffer[1], poll_tx_ts);
            final_msg_set_ts(&radioBuffer[5+4*j], resp_rx_ts);
          }
          else {
            connectedDevices &= ~(1 << (j+4));
          }
        }

        radioBuffer[0] = FINAL_MSG;

        /* Compute final message transmission time. */
        uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(final_tx_time);

        /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        final_msg_set_ts(&radioBuffer[17], final_tx_ts);

        // make sure TX done bit is cleared
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        /* Write and send final message. */
        decaSend(21, radioBuffer, 1, DWT_START_TX_DELAYED);
      }

			// if beacon is supposed to receive a message
			if(deviceUID & RXtimeTable[i]) {
        dwt_setrxtimeout(RX_TIMEOUT);
				ret = messageReceive(timeslotTable[i]);
				// if it's a ranging message
				if(ret == 1 && radioBuffer[0] == RANGE_MSG && deviceUID == RXtimeTable[i]) {
					messageAnswer(sendSerialData(&radioBuffer[3], TXtimeTable[i])+3);
					connectedDevices |= TXtimeTable[i];
				} else if(ret > 1 && radioBuffer[0] == DATA_MSG) { // if it's a robot data message
					parseRobotData(TXtimeTable[i], ret);
					connectedDevices |= TXtimeTable[i];
				} else {
					// connectedDevices &= ~TXtimeTable[i];
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
