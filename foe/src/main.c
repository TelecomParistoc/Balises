#include "ch.h"
#include "hal.h"
#include "chthreads.h"
#include "shell.h"

#include "usbconf.h"
#include "exticonf.h"
#include "radiocomms.h"
#include "../shared/nonvolatile.h"
#include "../shared/battery.h"

static THD_WORKING_AREA(waShell, 1024);

static const ShellCommand shCmds[] = {
	{"setid", setDeviceUID},
	{"getid", getDeviceUID},
	{NULL, NULL}
};

static ShellConfig shConfig = {
	(BaseSequentialStream *) &SDU1,
	shCmds
};

int main(void) {
	thread_t *sh = NULL;

	// initialize ChibiOS
	chSysInit();
	halInit();
	shellInit();

	// initialize hardware
	initExti();
	initBattery();
	initUSB();

	// start radio thread
	startRadio();

	while(1) {
		if(!sh && SDU1.config->usbp->state == USB_ACTIVE) {
			sh = shellCreateStatic(&shConfig, waShell, 1024, NORMALPRIO);
		}

		palSetLine(LINE_LED_RXTX);
		chThdSleepMilliseconds(500);
		palClearLine(LINE_LED_RXTX);
		chThdSleepMilliseconds(500);
	}
}
