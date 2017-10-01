#ifndef _RANDOM_H_
#define _RANDOM_H_

static uint8_t y8 = 1;
static uint16_t y16 = 1;
 
// returns values from 1 to 255 inclusive, period is 255
uint8_t xorshift8() {
  y8 ^= (y8 << 7);
  y8 ^= (y8 >> 5);
  return y8 ^= (y8 << 3);
}

#endif


