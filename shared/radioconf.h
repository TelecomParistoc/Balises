#ifndef RADIOCONF_H
#define RADIOCONF_H

/* Delay between ranging poll RX and response TX */
#define POLL_TO_RESP_DLY 800*UUS_TO_DWT_TIME

/* Delay between poll and RX activation */
#define POLL_TO_RESP_RX 150

/* RX timeout in us */
#define RX_TIMEOUT 600

/* time waiting for start-of-frame in us */
#define SYNC_RX_TIMEOUT 50000

/* required time between RX enable and actual RX */
#define AHEAD_OF_TX_MARGIN 200*UUS_TO_DWT_TIME

#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* message IDs : */
#define SOF_MSG 0x50
#define RANGE_MSG 0x23
#define DATA_MSG 0x32
#define POLL_MSG 0x29
#define RESP_MSG 0xCD
#define FINAL_MSG 0xD5
#define CAL_MSG 0x36

/* system IDs : */
#define BEACON1_ID  0x01
#define BEACON2_ID  0x02
#define BEACON3_ID  0x04
#define BIGFOE_ID   0x08
#define SMALLFOE_ID 0x10
#define BIGBOT_ID   0x20
#define SMALLBOT_ID 0x40

/* beacons position */
#define X1 (-60)
#define Y1 1000
#define X2 3060
#define Y2 1950
#define X3 3060
#define Y3 50
#define Z1 430
#define Z2 430
#define Z3 430

/* time slot length in ms */
#define TIMESLOT_LENGTH 2
#define DS_TWR_LENGTH 12

/* total length of a frame in number of time slots */
#define FRAME_LENGTH 7

/* parts that should be transmitting in each time slot (defined in radioconf.c) */
extern const char TXtimeTable[];
/* parts that should be listening in each time slot (defined in radioconf.c) */
extern const char RXtimeTable[];

extern const int timeslotTable[];

#endif
