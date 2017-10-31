#ifndef DECAFUNTIONS_H
#define DECAFUNTIONS_H

/* RX/TX buffer (size = RADIO_BUF_LEN)
 * write here bytes to send, read here bytes received */
#define RADIO_BUF_LEN 108
extern uint8_t radioBuffer[];

/* generate or wait for start-of-frame message. Calling this is required to give
 * a time reference to the other functions.
 *    isSOFsender : 1 if device emits start-of-frame message, 0 otherwise */
void synchronizeOnSOF(int isSOFsender);

/* Receive a message at the given time and write it in radioBuffer.
 * Returns the number of bytes sent or -1 if receiver timed out.
 *    timeInFrame : delay after SOF in UWB ms (1 UWB ms = 1.02 ms) when message
 *      is received */
int messageReceive(uint64_t timeInFrame);

/* Answer to a message. Answer data shall be stored in radioBuffer.
 * Answer is sent POLL_TO_RESP_DLY after message reception.
 * First three bytes of the answer will be used to send RX timestamp and MUST BE
 * left blank, but INCLUDED IN THE SIZE.
 * Returns -1 for transmission error.
 *    size : size of the answer message in bytes */
int messageAnswer(int size);

/* send a message (read from radioBuffer) and receive answer if required.
 * Returns -4 for transmission error, -1 for reception error or number of bytes
 * received if no error happened.
 *    timeInFrame : delay after SOF in UWB ms (1 UWB ms = 1.02 ms) when message
 *      is sent
 *    expectAnswer : 1 if receiver will sent back an answer, 0 otherwise.
 *    size : size of the message in bytes (message shall be written in
 *      radioBuffer) */
int messageSend(int timeInFrame, int expectAnswer, int size);

#endif
