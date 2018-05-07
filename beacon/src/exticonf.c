#include "ch.h"
#include "hal.h"

// Events sources
EVENTSOURCE_DECL(deca_event);
EVENTSOURCE_DECL(spi_event);

/* Decawave EXTI callback */
static void decaIRQ_cb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

	chSysLockFromISR();
	chEvtBroadcastFlagsI(&deca_event, EVENT_MASK(0));
	chSysUnlockFromISR();
}

static void SPI_cb(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  chSysLockFromISR();
  chEvtBroadcastFlagsI(&spi_event, EVENT_MASK(1));
  chSysUnlockFromISR();
}

// external interrupts configuration
static const EXTConfig extcfg = {
	{
		{EXT_CH_MODE_DISABLED, NULL}, // 0
		{EXT_CH_MODE_DISABLED, NULL}, // 1
		{EXT_CH_MODE_DISABLED, NULL}, // 2
		{EXT_CH_MODE_DISABLED, NULL}, // 3
		{EXT_CH_MODE_DISABLED, NULL}, // 4
		{EXT_CH_MODE_DISABLED, NULL}, // 5
		{EXT_CH_MODE_DISABLED, NULL}, // 6
		{EXT_CH_MODE_DISABLED, NULL}, // 7
		{EXT_CH_MODE_DISABLED, NULL}, // 8
		{EXT_CH_MODE_DISABLED, NULL}, // 9
		{EXT_CH_MODE_DISABLED, NULL}, // 10
		{EXT_CH_MODE_DISABLED, NULL}, // 11
		{EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, SPI_cb}, // 12
		{EXT_CH_MODE_DISABLED, NULL}, // 13
		{EXT_CH_MODE_DISABLED, NULL}, // 14
		{EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, decaIRQ_cb}, // 15
	}
};

void initExti(void) {
	extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, 12);
	extChannelEnable(&EXTD1, 15);
}
