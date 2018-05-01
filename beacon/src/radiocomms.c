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
#include "../shared/kalman.h"
#include "usbconf.h"
#include <math.h>

#define CALIBRATION_STEPS 100

// Initial position of the robots
#define BBX 200
#define BBY 1800
#define BBZ 400
#define SBX 200
#define SBY 1600
#define SBZ 400

// Distances to anchors
int16_t distances[3] = {0, 0, 0};
int16_t unfiltered[9];

int16_t SFCoordinates[2];
int16_t BFCoordinates[2];

// information of the robot
struct robotData radioData;
volatile int calibration = 0;

void computeCoordinates() {
  // Wikipedia formula:
  unfiltered[0] = (int16_t) ((distances[0]*distances[0]-distances[1]*distances[1]+X2*X2)/(2*X2));
  unfiltered[1] = (int16_t) ((distances[0]*distances[0]-distances[2]*distances[2]+X3*X3+Y3*Y3-2*X3*xVect[0][0])/(2*Y3));
  unfiltered[2] = pow(fabs(pow(distances[0], 2)-pow(unfiltered[0], 2)-pow(unfiltered[1], 2)), 0.5);

  // int z2 = distances[0]*distances[0]-radioData.x*radioData.x-radioData.y*radioData.y;

  // TODO: compare z2 to the real height of the beacon to exclude incoherent input
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
          // computeCoordinates();
          kalmanIteration(distances[0] - offset1, distances[1] - offset2, distances[2] - offset3);
          // printf("%u,%u\r\n", (uint16_t) xVect[0][0], (uint16_t) xVect[1][0]);

          radioBuffer[0] = DATA_MSG;
          for (int j = 0; j < 3; j++) {
            radioBuffer[2*j+1] = (uint16_t) distances[j];
            radioBuffer[2*j+2] = ((uint16_t) distances[j]) >> 8;
          }
          for (int j = 0; j < 6; j++) {
            radioBuffer[2*j+7] = (uint16_t) unfiltered[j];
            radioBuffer[2*j+8] = ((uint16_t) unfiltered[j]) >> 8;
          }
          for (int j = 0; j < 2; j++) {
            radioBuffer[2*j+19] = (uint16_t) xVect[j][0];
            radioBuffer[2*j+20] = ((uint16_t) xVect[j][0]) >> 8;
          }
          radioBuffer[23] = radioBuffer[24] = 0;
          radioBuffer[25] = 0;

          if (calibration == 1)
            radioBuffer[57] = CAL_MSG;
          else
            radioBuffer[57] = 0;

          // send data message
          ret = messageSend(i*TIMESLOT_LENGTH, 0, 26);
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

            if (deviceUID == BIGBOT_ID) {
              distances[11-i] = distanceInMm;
              if (calibration > 0) {
                averageCalibration[11-i] = (averageCalibration[11-i]*(calibration-1) + distanceInMm)/calibration;
                calibration ++;
              }
            }
            else if (deviceUID == SMALLBOT_ID) {
              distances[15-i] = distanceInMm;
              if (calibration > 0) {
                averageCalibration[15-i] = (averageCalibration[15-i]*(calibration-1) + distanceInMm)/calibration;
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
          // store coordinates of foes
          if (BIGFOE_ID & TXtimeTable[i]) {
            BFCoordinates[0] = radioBuffer[1];
            BFCoordinates[0] |= (radioBuffer[2] << 8);
            BFCoordinates[1] = radioBuffer[3];
            BFCoordinates[1] |= (radioBuffer[4] << 8);
          }
          else if (SMALLFOE_ID & TXtimeTable[i]) {
            SFCoordinates[0] = radioBuffer[1];
            SFCoordinates[0] |= (radioBuffer[2] << 8);
            SFCoordinates[1] = radioBuffer[3];
            SFCoordinates[1] |= (radioBuffer[4] << 8);
          }
        }
      }
    }

    if (calibration == CALIBRATION_STEPS + 1) {
      calibration = 0;
      int x, y, z;
      if (deviceUID == BIGBOT_ID) {
        x = BBX;
        y = BBY;
        z = BBZ;
      }
      else if (deviceUID == SMALLBOT_ID) {
        x = SBX;
        y = SBY;
        z = SBZ;
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
