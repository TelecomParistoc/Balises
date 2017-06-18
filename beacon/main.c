#include "ch.h"
#include "hal.h"
#include "chthreads.h"
#include "shell.h"

#include "usbconf.h"
#include "exticonf.h"
#include "radiocomms.h"
// #include "robot.h"
// #include "battery.h"

static THD_WORKING_AREA(waShell, 1024);

static const ShellCommand SBshCmds[] = {
	{NULL, NULL}
};

static ShellConfig shConfig = {
	(BaseSequentialStream *) &SDU1,
	SBshCmds
};

int main(void) {
	thread_t *sh = NULL;

	// initialize ChibiOS
	chSysInit();
	halInit();
	shellInit();

	// initialize hardware
	initExti();
	// initBattery();
	initUSB();

	// start radio thread
	startRadio();

	while(1) {
		if(!sh && SDU1.config->usbp->state == USB_ACTIVE) {
			sh = shellCreateStatic(&shConfig, waShell, 1024, NORMALPRIO);
		}
		palSetLine(LINE_LED_SYNC);
		palSetLine(LINE_LED_RXTX);
		palSetLine(LINE_LED_BATT);
		chThdSleepMilliseconds(50);
		palClearLine(LINE_LED_SYNC);
		palClearLine(LINE_LED_RXTX);
		palClearLine(LINE_LED_BATT);
		chThdSleepMilliseconds(50);
	}
}
