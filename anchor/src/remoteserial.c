#include "ch.h"
#include "chprintf.h"
#include "usbconf.h"
#include "radiocomms.h"
#include "../shared/radioconf.h"

// not zero when remote serial is active (in use).
static int remoteSerialActive = 0;

// open a remote serial on one of the robots
static void openRemoteSerial(int robotID, int argc) {
	if(argc > 0) {
		printf("Too many arguments: Expected 0, got %d\n", argc);
		return;
	}

	remoteSerialActive = 1;
	dataID = robotID;
	chThdSleep(TIME_INFINITE);
}

// shell callback : open a remote serial on the big robot, USAGE : big
void openRemoteSerialBig(BaseSequentialStream *chp, int argc, char **argv) {
	(void) chp;
	(void) argv;
	openRemoteSerial(BIGBOT_ID, argc);
}

// shell callback : open a remote serial on the small robot, USAGE : small
void openRemoteSerialSmall(BaseSequentialStream *chp, int argc, char **argv) {
	(void) chp;
	(void) argv;
	openRemoteSerial(SMALLBOT_ID, argc);
}

// hack to avoid waiting if there's not enough data ready
static size_t readUSBserial(void *ip, uint8_t *bp, size_t n) {
	if (usbGetDriverStateI(((SerialUSBDriver *)ip)->config->usbp) != USB_ACTIVE)
		return 0;

	return ibqReadTimeout(&((SerialUSBDriver *)ip)->ibqueue, bp, n, TIME_IMMEDIATE);
}

// when using remote serial, send data from the user to the robot
// (called by radio loop in radiocomms.c)
int sendSerialData(uint8_t* dataBuffer, int senderID) {
	// if remote serial is not connected or sender doesn't match, skip
	if(dataID != senderID || !remoteSerialActive)
		return 0;
	// read up to 20 character from Serial-over-USB port
	return readUSBserial(USBserial, dataBuffer, 20);
}

// when using remote serial, read data from the robot and write it to the serial
// port (called by parseRobotData() in radiocomms.c)
void receiveSerialData(uint8_t* dataBuffer, int size, int senderID) {
	// if remote serial is connected and sender match, transfer data to serial
	if(dataID == senderID && remoteSerialActive)
		chSequentialStreamWrite(USBserial, dataBuffer, size);
}
