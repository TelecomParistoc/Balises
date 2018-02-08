#ifndef I2C_INTERFACE_GEN_C
#define I2C_INTERFACE_GEN_C

static void i2c_vt_cb(void* param)
{
    (void)param;

    if (rx_buffer[0] != NO_DATA) {
        switch(rx_buffer[0])
        {
        default:
            rx_special_cases(rx_buffer[0]);
            break;
        }
    }

    /* Free the rx_buffer */
    rx_buffer[0] = NO_DATA;
    rx_buffer[1] = NO_DATA;
    rx_buffer[2] = NO_DATA;
}


static void i2c_address_match(I2CDriver* i2cp) {
    (void)i2cp;
    uint16_t value;
    bool single_byte = FALSE;

    if (rx_buffer[0] != NO_DATA) {
        /* Start of the read part of a read-after-write exchange */
        chSysLockFromISR();
        chVTResetI(&i2c_vt);
        chSysUnlockFromISR();

        /* Prepare the answer */
        switch (rx_buffer[0]) {
        case CUR_POS_X_ADDR:
            value = cur_pos.x;
            break;
        case CUR_POS_Y_ADDR:
            value = cur_pos.y;
            break;
        default:
            tx_special_cases(rx_buffer[0], &value);
            break;
        }

        if (single_byte == FALSE) {
            tx_buffer[1] = (uint8_t)((value & 0xFF00) >> 8);
            tx_buffer[0] = (uint8_t)(value & 0x00FF);
            i2c_response.size = 2U;
        } else {
            tx_buffer[0] = (uint8_t)(value & 0x00FF);
            i2c_response.size = 1U;
        }

        /* Free the rx buffer */
        rx_buffer[0] = NO_DATA;
    } else {
        /* Start of a write exchange */
        chSysLockFromISR();
        chVTSetI(&i2c_vt, US2ST(800), i2c_vt_cb, NULL);
        chSysUnlockFromISR();
    }
}

#endif /* I2C_INTERFACE_GEN_C */
