#ifndef EXTICONF_H
#define EXTICONF_H

#include "ch.h"

/* Events sources */
extern event_source_t deca_event; /* sent on decawave IRQ rising edge */
extern event_source_t spi_event; /* sent on GPIOB_MOT_nCS falling edge */

/* enable external interrupts */
void initExti(void);

#endif
