/*

Euclidean sequencer for Ginko Synthese Grains

pot 1 - steps
pot 2 - pulses
pot 3 - offset
jack 3 - clock in
jack 4 - gate out

*/

#include <digitalWriteFast.h>

#include "defs.h"
#include "bjorklund.h"

Euclidean rhytm;
Euclidean rhytmTemp;

bool pattern[RHYTM_LENGTH];
uint16_t potTemp;
uint16_t potMax;
uint8_t potTimer;
uint8_t output;
bool gateHigh;
bool running;
uint8_t index;
uint8_t patternIndex;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  // Serial.begin(9600);

  output = 0;
  gateHigh = false;
  index = 0;
  patternIndex = index;

  potTemp = 0;
  potMax = 500;
  potTimer = 0;

  rhytm.steps = 16;
  rhytm.pulses = 4;
  rhytm.offset = 0;
  rhytmTemp.steps = rhytm.steps;
  rhytmTemp.pulses = rhytm.pulses;
  rhytmTemp.offset = rhytm.offset;

  getPattern(pattern);
}

uint16_t inline getGate() {
  return analogRead(PIN3);
}

uint8_t inline readPot(uint8_t pot, uint8_t _max) {
  potTemp = analogRead(pot);

  if (potTemp > potMax) {
    potMax = potTemp + 2;
  }

  return map(potTemp, 0, potMax, 0, _max);
}

void handlePots() {
  rhytmTemp.steps = readPot(POT1, RHYTM_LENGTH - 1);
  rhytmTemp.pulses = readPot(POT2, rhytm.steps);
  rhytmTemp.offset = readPot(POT3, RHYTM_LENGTH - 1);

  if (rhytmTemp.steps != rhytm.steps
    || rhytmTemp.pulses != rhytm.pulses
    || rhytmTemp.offset != rhytm.offset) {
    updateRhytm(rhytmTemp);
  }
}

void updateRhytm(Euclidean newRhytm) {
  rhytm.steps = newRhytm.steps;
  rhytm.pulses = newRhytm.pulses;
  rhytm.offset = newRhytm.offset;

  if (!running) {
    running = true;
    getPattern(pattern);
    // printPattern(pattern);
    running = false;
  }
}

void getPattern(bool pattern[]) {
  bjorklund(pattern, rhytm);
}

void printPattern(bool pattern[]) {
  for (uint8_t i=0; i<=rhytm.steps; i++) {
    Serial.print(pattern[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void printRhytm(Euclidean rhytm) {
  Serial.print(rhytm.steps);
  Serial.print(" ");
  Serial.print(rhytm.pulses);
  Serial.print(" ");
  Serial.println(rhytm.offset);
}

void loop() {
  if (++potTimer > 10) {
    handlePots();
    potTimer = 0;
  }

  if (getGate() > 64) {
    if (!gateHigh) {
      gateHigh = true;

      if (++index > rhytm.steps) {
        index = 0;
      }

      patternIndex = (index + rhytm.offset) % (rhytm.steps + 1);
      if (pattern[patternIndex]) {
        digitalWriteFast(PWM_PIN, HIGH);
        digitalWriteFast(LED_PIN, HIGH);
        delay(GT_PW);
      }

      digitalWriteFast(PWM_PIN, LOW);
      digitalWriteFast(LED_PIN, LOW);
    }
  } else {
    gateHigh = false;
  }
}
