/* deca-plateform.c :
* This implements all the platform specific functions required by the Decawave
* driver and low level methods
*/

#include "ch.h"
#include "hal.h"

#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "exticonf.h"
#include "decaplatform.h"
#include <math.h>
#include "nonvolatile.h"

// 2.25MHz SPI configuration, CPHA=0, CPOL=0
static const SPIConfig slowspiconfig = {
  false,
	NULL,
	GPIOB,
	GPIOB_DWM_nCS,
	SPI_CR1_BR_1 | SPI_CR1_BR_0,
	0
};
// 18MHz SPI configuration, CPHA=0, CPOL=0
static const SPIConfig spiconfig = {
  false,
	NULL,
	GPIOB,
	GPIOB_DWM_nCS,
	0,
	0
};

const int DLY[6] = {
  24080,
  22501,
  23058,
  23580,
  23277,
  23508
};

static void initDecaPlatform(void) {
	// make sure chip select is not active
	palSetLine(LINE_DWM_nCS);

	// init SPI3 with SPI clock at 2.6MHz
	spiStart(&SPID3, &slowspiconfig);
}

static void useFastSPI(void) {
	spiStart(&SPID3, &spiconfig);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {
	spiSelect(&SPID3);                  // Slave Select assertion.

	spiSend(&SPID3, headerLength, headerBuffer);
	spiSend(&SPID3, bodylength, bodyBuffer);

	spiUnselect(&SPID3);                // Slave Select de-assertion.
	return 0;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
	spiSelect(&SPID3);                  // Slave Select assertion.

	spiSend(&SPID3, headerLength, headerBuffer);
	spiReceive(&SPID3, readlength, readBuffer);

	spiUnselect(&SPID3);                // Slave Select de-assertion.
	return 0;
}

decaIrqStatus_t decamutexon(void) { return 0; }

void decamutexoff(decaIrqStatus_t s) { (void) s; }

void deca_sleep(unsigned int time_ms) { chThdSleepMilliseconds(time_ms); }


// communication configuration
static dwt_config_t radioConfig = {
	2,               // Channel number
	DWT_PRF_64M,     // Pulse repetition frequency
	DWT_PLEN_256,    // Preamble length. Used in TX only
	DWT_PAC16,        // Preamble acquisition chunk size. Used in RX only
	9,               // TX preamble code. Used in TX only
	9,               // RX preamble code. Used in RX only
	1,               // 0 to use standard SFD, 1 to use non-standard SFD
	DWT_BR_6M8,      // Data rate
	DWT_PHRMODE_STD, // PHY header mode
	(257 + 8 - 16)    // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only
};

int decaInit(void) {
	// initialize peripherals used by decawave module
	initDecaPlatform();
	// reset module
	palClearLine(LINE_DWM_RST);
	chThdSleepMilliseconds(3);
	palSetLine(LINE_DWM_RST);
	chThdSleepMilliseconds(3);
	// initialize for ranging
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
		return -1;
	}
	// after initialize, use fast SPI for optimum performance
	useFastSPI();

	dwt_configure(&radioConfig);
	// Apply default antenna delay value.
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	// activate interrupts for tx done, rx done, rx errors and timeout
	dwt_setinterrupt(SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_TXFRS, 1);

	return 0;
}

int decaSend(int size, uint8_t *buffer, int ranging, int flags) {
	int ret;
	dwt_writetxdata(size + 2, buffer, 0);
	dwt_writetxfctrl(size + 2, 0, ranging);
	ret = dwt_starttx(flags);

	if(ret == DWT_SUCCESS) {
		// wait until message is sent
		chEvtWaitAny(ALL_EVENTS);
		// Clear TXFRS event
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	}
	return ret;
}

int decaReceive(int maxSize, uint8_t *buffer, int flag) {
	int messageLength;
	uint32_t status_reg;

	// activate reception
	if(flag != NO_RX_ENABLE)
		dwt_rxenable(flag);

	chEvtWaitAny(ALL_EVENTS);
	status_reg = dwt_read32bitreg(SYS_STATUS_ID);

	if (status_reg & SYS_STATUS_RXFCG) {
		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		// read the message
		messageLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (messageLength <= maxSize) {
			dwt_readrxdata(buffer, messageLength, 0);
			return messageLength - 2;
		} else {
			return -3; // error : buffer is too small
		}
	} else {
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		// Reset RX to properly reinitialise LDE operation. (User Manual p.34)
		dwt_rxreset();
		return (status_reg & SYS_STATUS_ALL_RX_TO) ? -2 : -1;
	}
}

// adapted from a code by Samuel Tardieu (see rfc1149.net)
void sleepUntil(systime_t previous, int period) {
	systime_t future = previous + period*CH_CFG_ST_FREQUENCY/1000*512/499.2;

	chSysLock();
	systime_t now = chVTGetSystemTimeX();

	int mustDelay = now < previous ?
		(now < future && future < previous) :
		(now < future || future < previous);
	if (mustDelay)
		chThdSleepS(future - now);
	chSysUnlock();
}
