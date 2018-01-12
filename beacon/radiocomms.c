#include "ch.h"
#include "chvt.h"
#include "chevents.h"
#include "chprintf.h"

#include "exticonf.h"
#include "../shared/radioconf.h"
#include "../shared/decaplatform.h"
#include "../shared/decafunctions.h"
#include "../shared/decadriver/deca_device_api.h"
#include "nonvolatile.h"
#include "radiocomms.h"
#include "kalman.h"
#include "usbconf.h"
#include <math.h>

#define CALIBRATION_STEPS 100

#define BBX 200
#define BBY 1800
#define SBX 200
#define SBY 1600

// Distances to anchors
int16_t distances[3] = {0, 0, 0};
// information of the robot
struct robotData radioData;

void computeCoordinates() {
  radioData.x = (int16_t) ((distances[0]*distances[0]-distances[1]*distances[1]+X2*X2)/(2*X2));
  radioData.y = (int16_t) ((distances[0]*distances[0]-distances[2]*distances[2]+X3*X3+Y3*Y3-2*X3*radioData.x)/(2*Y3));
  // int z2 = distances[0]*distances[0]-radioData.x*radioData.x-radioData.y*radioData.y;

  // TODO: compare z2 to the real height of the beacon to exclude incoherent input

  if(1)
    // printf("%u,%u\r\n", radioData.x, radioData.y);
    printf("%i,%i,%i\r\n", distances[0], distances[1], distances[2]);
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

  int calibration = 0;
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
          printf("%u,%u\r\n", (uint16_t) xVect[0][0], (uint16_t) xVect[1][0]);

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
                averageCalibration[3-i] = (averageCalibration[3-i]*calibration + distanceInMm)/(calibration+1);
              }
            }
            else if (deviceUID == SMALLFOE_ID) {
              distances[7-i] = distanceInMm;
              if (calibration > 0) {
                averageCalibration[7-i] = (averageCalibration[7-i]*calibration + distanceInMm)/(calibration+1);
              }
            }
          }
        }
      }
    }
    if (calibration > 0 && calibration < CALIBRATION_STEPS)
      calibration++;
    else if (calibration > 0) {
      calibration = 0;
      if (deviceUID == BIGBOT_ID) {
        offset1 = averageCalibration[0] - pow(BBX*BBX + BBY*BBY, 0.5);
        offset2 = averageCalibration[1] - pow((X2-BBX)*(X2-BBX) + BBY*BBY, 0.5);
        offset3 = averageCalibration[2] - pow((X3-BBX)*((X3-BBX)) + (Y3-BBY)*(Y3-BBY), 0.5);
      }
      else {
        offset1 = averageCalibration[0] - pow(SBX*SBX + SBY*SBY, 0.5);
        offset2 = averageCalibration[1] - pow((X2-SBX)*(X2-SBX) + SBY*SBY, 0.5);
        offset3 = averageCalibration[2] - pow((X3-SBX)*((X3-SBX)) + (Y3-SBY)*(Y3-SBY), 0.5);
      }
    }
  }
}

void startRadio(void) {
  chThdCreateStatic(waRadio, sizeof(waRadio), NORMALPRIO+1, radioThread, NULL);
}
