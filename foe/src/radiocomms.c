#include "ch.h"
#include "chvt.h"
#include "chevents.h"
#include "chprintf.h"

#include "exticonf.h"
#include "../shared/radioconf.h"
#include "../shared/decaplatform.h"
#include "../shared/decafunctions.h"
#include "../shared/decadriver/deca_device_api.h"
#include "../shared/decadriver/deca_regs.h"
#include "../shared/nonvolatile.h"
#include "radiocomms.h"
#include "../shared/kalman.h"
#include "usbconf.h"
#include <math.h>

#define CALIBRATION_STEPS 100

// Initial position of the robots
#define BFX 200
#define BFY 1800
#define BFZ 400
#define SFX 200
#define SFY 1600
#define SFZ 400

// Distances to anchors
int16_t distances[3] = {0, 0, 0};

// information of the robot
struct robotData radioData;
int calibration = 0;

void computeCoordinates() {
  // Wikipedia formula:
  radioData.x = (int16_t) ((distances[0]*distances[0]-distances[1]*distances[1]+X2*X2)/(2*X2));
  radioData.y = (int16_t) ((distances[0]*distances[0]-distances[2]*distances[2]+X3*X3+Y3*Y3-2*X3*radioData.x)/(2*Y3));
  // int z2 = distances[0]*distances[0]-radioData.x*radioData.x-radioData.y*radioData.y;

  // TODO: compare z2 to the real height of the beacon to exclude incoherent input
}

static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    *ts = 0;
    for (int i = 0; i < 4; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

static THD_WORKING_AREA(waRadio, 1024);
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

  initKalmanCst();

  float averageCalibration[3] = {0, 0, 0};

  int offset1 = 952;
  int offset2 = 758;
  int offset3 = 456;

  while(1) {
    synchronizeOnSOF(0);

    for(int i=1; i<FRAME_LENGTH; i++) {
      // check for time to send message
      if(TXtimeTable[i] & deviceUID) {
        // check if we are in a data time slot
        if (i % 4 == 0) {
          kalmanIteration(distances[0] - offset1, distances[1] - offset2, distances[2] - offset3);

          radioBuffer[0] = DATA_MSG;
          radioBuffer[1] = (uint16_t) xVect[0][0];
          radioBuffer[2] = ((uint16_t) xVect[0][0]) >> 8;
          radioBuffer[3] = (uint16_t) xVect[1][0];
          radioBuffer[4] = ((uint16_t) xVect[1][0]) >> 8;
          radioBuffer[5] = 0;

          // send data message
          ret = messageSend(i*TIMESLOT_LENGTH, 0, 6);
          if(ret == -4)
            printf("TXerr, f= %u\r\n", i);
          else if (ret < 0)
            printf("RXerr, f= %u\r\n", i);
        }

        else {
          radioBuffer[0] = RANGE_MSG;
          // something else to put in radioBuffer here?

          // send ranging message
          ret = messageSend(i*TIMESLOT_LENGTH, 1, 1);
          if(ret == -4)
            printf("TXerr, f= %u\r\n", i);
          else if (ret <= 0)
            printf("RXerr, f= %u\r\n", i);
          // check frame is actually our response
          // condition ret>0 useless with previous else if
          else if (ret > 0 && radioBuffer[0] == RANGE_MSG) {
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

            if (deviceUID == BIGFOE_ID) {
              distances[3-i] = distanceInMm;
              if (calibration > 0) {
                averageCalibration[3-i] = (averageCalibration[3-i]*(calibration-1) + distanceInMm)/calibration;
                calibration ++;
              }
            }
            else if (deviceUID == SMALLFOE_ID) {
              distances[7-i] = distanceInMm;
              if (calibration > 0) {
                averageCalibration[7-i] = (averageCalibration[7-i]*(calibration-1) + distanceInMm)/calibration;
                calibration ++;
              }
            }
          }
        }
      }

      // if beacon is supposed to receive a message
      else if(deviceUID & RXtimeTable[i]) {
        ret = messageReceive(i*TIMESLOT_LENGTH);
        // if it's a robot data message
        if(ret > 1 && radioBuffer[0] == DATA_MSG) {
          // if BigBot sends a calibration order
          if (radioBuffer[57] == CAL_MSG && calibration == 0)
            calibration = 1;
        }

        else if (ret > 0 && radioBuffer[0] == POLL_MSG) {
          /* Retrieve poll reception timestamp. */
          uint64_t poll_rx_ts = getRXtimestamp();

          /* Set send time for response. */
          uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
          dwt_setdelayedtrxtime(resp_tx_time);

          /* Set expected delay and timeout for final message reception. */
          if (deviceUID == BIGFOE_ID) {
            dwt_setrxaftertxdelay(RESP_RX_TO_FINAL_TX_DLY_UUS + RESP_TX_TO_FINAL_RX_DLY_UUS);
            dwt_setrxtimeout(2*(RESP_RX_TO_FINAL_TX_DLY_UUS + 200));
          }
          else if (deviceUID == SMALLFOE_ID) {
            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
            dwt_setrxtimeout(RESP_RX_TO_FINAL_TX_DLY_UUS + 200);
          }

          /* Write and send the response message. */
          radioBuffer[0] = RESP_MSG;

          // make sure TX done bit is cleared
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
          /* Write and send final message. */
          decaSend(1, radioBuffer, 1, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
          ret = decaReceive(RADIO_BUF_LEN, radioBuffer, NO_RX_ENABLE);

          if (ret > 1 && radioBuffer[0] == FINAL_MSG) {
            uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts, poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
            double Ra, Rb, Da, Db, tof, distance;
            int64_t tof_dtu, resp_tx_ts, final_rx_ts;

            /* Retrieve response transmission and final reception timestamps. */
            resp_tx_ts = getTXtimestamp();
            final_rx_ts = getRXtimestamp();

            /* Get timestamps embedded in the final message. */
            final_msg_get_ts(&radioBuffer[1], &poll_tx_ts);
            if (deviceUID == BIGFOE_ID)
              final_msg_get_ts(&radioBuffer[9], &resp_rx_ts);
            else if (deviceUID == SMALLFOE_ID)
              final_msg_get_ts(&radioBuffer[13], &resp_rx_ts);
            final_msg_get_ts(&radioBuffer[17], &final_tx_ts);

            /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. */
            poll_rx_ts_32 = (uint32_t)poll_rx_ts;
            resp_tx_ts_32 = (uint32_t)resp_tx_ts;
            final_rx_ts_32 = (uint32_t)final_rx_ts;
            Ra = (double)(resp_rx_ts - poll_tx_ts);
            Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            Da = (double)(final_tx_ts - resp_rx_ts);
            Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
            tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

            tof = tof_dtu * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
          }
        }
      }
    }

    if (calibration == CALIBRATION_STEPS + 1) {
      calibration = 0;
      int x, y, z;
      if (deviceUID == BIGFOE_ID) {
        x = BFX;
        y = BFY;
        z = BFZ;
      }
      else if (deviceUID == SMALLFOE_ID) {
        x = SFX;
        y = SFY;
        z = SFZ;
      }
      offset1 = averageCalibration[0] - pow((X1-x)*(X1-x) + (Y1-y)*(Y1-y) + (Z1-z)*(Z1-z), 0.5);
      offset2 = averageCalibration[1] - pow((X2-x)*(X2-x) + (Y2-y)*(Y2-y) + (Z2-z)*(Z2-z), 0.5);
      offset3 = averageCalibration[2] - pow((X3-x)*(X3-x) + (Y3-y)*(Y3-y) + (Z3-z)*(Z3-z), 0.5);
    }
  }
}

void startRadio(void) {
  chThdCreateStatic(waRadio, sizeof(waRadio), NORMALPRIO+1, radioThread, NULL);
}
