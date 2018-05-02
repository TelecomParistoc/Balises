#include "hal.h"
#include "../shared/kalman.h"
#include "radiocomms.h"

#include "chprintf.h"
#include "usbconf.h"
#include "spicomms.h"

#define TL_PACKET_SIZE_MOSI 6
#define TL_PACKET_SIZE_MISO 12

#define CALIBRATION_SYMBOL 0b1101
#define REPOSITION_SYMBOL 0b1011

#define EVT_nCS EVENT_MASK(12)

#define MODE 2

/*
 * SPI TX and RX buffers.
 */
static uint8_t mosiBuff[TL_PACKET_SIZE_MOSI];
static uint8_t misoBuff[TL_PACKET_SIZE_MISO];

#if MODE == 1

static const SPIConfig spi2_master = {
   .slave_mode = false,
   .end_cb     = NULL,
   .ssport     = GPIOB,
   .sspad      = GPIOB_MOT_nCS,
   .cr1        = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0, // 140kHz
   .cr2        = 0
};

static THD_WORKING_AREA(waSPI, 256);
static THD_FUNCTION(spi_thread, p) {

  (void)p;

  chRegSetThreadName("SPI thread master");

  uint8_t cmpt = 0;

  spiStart(&SPID2, &spi2_master);

  while (true) {
    spiSelect(&SPID2);
    mosiBuff[0] = 230;  // synchronization byte
    mosiBuff[1] = cmpt; // calibration and reposition byte
    if (calibration == 1) {
      mosiBuff[1] &= 0xf0;
      mosiBuff[1] |= CALIBRATION_SYMBOL;
    }
    // TODO: reposition
    mosiBuff[2] = 153;
    mosiBuff[3] = misoBuff[0];
    spiSend(&SPID2, TL_PACKET_SIZE_MOSI, mosiBuff);
    spiReceive(&SPID2, TL_PACKET_SIZE_MISO, misoBuff);
    cmpt++;
    spiUnselect(&SPID2);
    chThdSleepMilliseconds(10);
  }
}

#elif MODE == 2

static const SPIConfig spi2_slave = {
   .slave_mode = true,
   .end_cb     = NULL,
   .ssport     = GPIOB,
   .sspad      = GPIOB_MOT_nCS,
   .cr1        = 0,
   .cr2        = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 // 8-bits
};

#if (SPI_USE_WAIT != TRUE)
#error enable SPI_USE_WAIT in the halconf.h
#endif // SPI_USE_WAIT

/**
 * @brief   Sends data over the SPI bus.
 * @details This synchronous function performs a transmit operation.
 * @pre     In order to use this function the option @p SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @api
 */
static void sspiSend(SPIDriver *spip, size_t n, const void *txbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_READY, "not ready");
  osalDbgAssert(spip->config->end_cb == NULL, "has callback");
  spiStartSendI(spip, n, txbuf);
  // assert_dsp_rdy();
  (void) osalThreadSuspendS(&spip->thread);
  // clear_dsp_rdy();
  osalSysUnlock();
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This synchronous function performs a receive operation.
 * @pre     In order to use this function the option @p SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
static void sspiReceive(SPIDriver *spip, size_t n, void *rxbuf) {

  osalDbgCheck((spip != NULL) && (n > 0U) && (rxbuf != NULL));

  osalSysLock();
  osalDbgAssert(spip->state == SPI_READY, "not ready");
  osalDbgAssert(spip->config->end_cb == NULL, "has callback");
  spiStartReceiveI(spip, n, rxbuf);
  // assert_dsp_rdy();
  (void) osalThreadSuspendS(&spip->thread);
  // clear_dsp_rdy();
  osalSysUnlock();
}

/**
 * tl_start_receive - setup SPI for data reception.
 */
static void tl_start_receive(void)
{
  spiStart(&SPID2, &spi2_slave);
  sspiReceive(&SPID2, TL_PACKET_SIZE_MOSI, mosiBuff);
}

/**
 * tl_start_send - setup SPI for data transmission.
 */
static void tl_start_send(void)
{
  spiStart(&SPID2, &spi2_slave);
  sspiSend(&SPID2, TL_PACKET_SIZE_MISO, misoBuff);
}

static THD_WORKING_AREA(waSPI, 256);
static THD_FUNCTION(spi_thread, th_data) {
  (void) th_data;

  chRegSetThreadName("SPI slave thread");
  uint8_t cmpt = 0;

  while (true) {
    // wait for slave select
    chEvtWaitAny(EVT_nCS);

    tl_start_receive();

    // calibration
    if ((mosiBuff[1] >> 4) == CALIBRATION_SYMBOL) {
      if (calibration == 0)
        calibration = 1;
    }
    // reposition
    if ((mosiBuff[1] & ~(0b1111 << 4)) == REPOSITION_SYMBOL) {
      // TODO
    }

    // coordinates of our robot, from kalman result
    misoBuff[0] = (uint16_t) xVect[0][0];
    misoBuff[1] = ((uint16_t) xVect[0][0]) >> 8;
    misoBuff[2] = (uint16_t) xVect[1][0];
    misoBuff[3] = ((uint16_t) xVect[1][0]) >> 8;

    // coordinates of small foe
    misoBuff[4] = (uint16_t) SFCoordinates[0];
    misoBuff[5] = ((uint16_t) SFCoordinates[0]) >> 8;
    misoBuff[6] = (uint16_t) SFCoordinates[1];
    misoBuff[7] = ((uint16_t) SFCoordinates[1]) >> 8;

    // coordinates of big foe
    misoBuff[8] = (uint16_t) BFCoordinates[0];
    misoBuff[9] = ((uint16_t) BFCoordinates[0]) >> 8;
    misoBuff[10] = (uint16_t) BFCoordinates[1];
    misoBuff[11] = ((uint16_t) BFCoordinates[1]) >> 8;

    misoBuff[0] = cmpt;
    misoBuff[1] = mosiBuff[1];

    tl_start_send();
    cmpt ++;
  }
}

#endif

void startSPI(void) {
  chThdCreateStatic(waSPI, sizeof(waSPI), NORMALPRIO+1, spi_thread, NULL);
}
