#ifndef BEACON_H
#define BEACON_H

#include "hal.h"

typedef struct  {
  uint16_t x;
  uint16_t y;
} cur_pos_t;

extern volatile cur_pos_t cur_pos;

#endif
