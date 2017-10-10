#ifndef _DEFS_H_
#define _DEFS_H_

#define POT1          (2)
#define POT2          (1)
#define POT3          (0)
#define PIN3          (3)

#define LED_PIN       (13)
#define PWM_PIN       (11)

#define GT_PW         (20) // ~ms

#define RHYTM_LENGTH  (16)

struct Euclidean {
  uint8_t steps;
  uint8_t pulses;
  uint8_t offset;
};

#endif
