#ifndef TON_H
#define TON_H

#include "stdint.h"

typedef struct
{
  uint32_t since;
  uint32_t aux;
} ton_t;

uint8_t TON(ton_t *obj, uint8_t in, uint32_t now, uint32_t preset_time);

#endif /* TON_H */
