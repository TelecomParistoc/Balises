#ifndef RADIOCONF_H
#define RADIOCONF_H

/* Delay between ranging poll RX and response TX */
#define POLL_TO_RESP_DLY 800*UUS_TO_DWT_TIME

/* Delay between poll and RX activation */
#define POLL_TO_RESP_RX 600

/* RX timeout in us */
#define RX_TIMEOUT 1000

/* time waiting for start-of-frame in us */
#define SYNC_RX_TIMEOUT 50000

/* required time between RX enable and actual RX */
#define AHEAD_OF_TX_MARGIN 200*UUS_TO_DWT_TIME

/* battery state codes */
#define BATTERY_VERYLOW 0
#define BATTERY_LOW 1
#define BATTERY_OK 2
#define BATTERY_HIGH 3

/* system IDs : */
#define BEACON1_ID  0x01
#define BEACON2_ID  0x02
#define BEACON3_ID  0x04
#define BIGFOE_ID   0x08
#define SMALLFOE_ID 0x10
#define BIGBOT_ID   0x20
#define SMALLBOT_ID 0x40

/* time slot length in ms */
#define TIMESLOT_LENGTH 2

/* total length of a frame in number of time slots */
#define FRAME_LENGTH 17

/* parts that should be transmitting in each time slot (defined in radioconf.c) */
extern const char TXtimeTable[];
/* parts that should be listening in each time slot (defined in radioconf.c) */
extern const char RXtimeTable[];

#endif
