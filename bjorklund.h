#ifndef _BJORKLUND_H_
#define _BJORKLUND_H_

#include "defs.h"

uint8_t patternSC;

void buildPattern(bool pattern[], int8_t counts[], int8_t remainders[], int8_t level) {
  if (level == -1) {
    pattern[patternSC++] = false;
  } else if (level == -2) {
    pattern[patternSC++] = true;
  } else {
    for (int i=0; i<counts[level]; i++) {
      buildPattern(pattern, counts, remainders, level - 1);
    }

    if (remainders[level] != 0) {
      buildPattern(pattern, counts, remainders, level - 2);
    }
  }
}

void bjorklund(bool pattern[], Euclidean rhytm) {
  patternSC = 0;
  int8_t counts[RHYTM_LENGTH] = { 0 };
  uint8_t countsSC = 0;
  int8_t remainders[RHYTM_LENGTH] = { 0 };
  uint8_t remaindersSC = 0;

  int8_t level = 0;
  int8_t divisor = (rhytm.steps + 1) - (rhytm.pulses + 1);

  remainders[remaindersSC++] = (rhytm.pulses + 1);

  while(1) {
    int8_t count = divisor / remainders[level];

    counts[countsSC++] = count;

    int8_t remainder = divisor % remainders[level];
    remainders[remaindersSC++] = remainder;

    divisor = remainders[level];
    level += 1;

    if (remainders[level] <= 1) {
      break;
    }
  };

  counts[countsSC++] = divisor;

  buildPattern(pattern, counts, remainders, level);
}

#endif
