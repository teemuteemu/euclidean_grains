#define POT1          (2)
#define POT2          (1)
#define POT3          (0)
#define PIN3          (3)

#define PWM_PIN       (11)
#define PWM_VALUE     OCR2A
#define PWM_INTERRUPT TIMER2_OVF_vect

#define POT_MAX       (590)
#define RHYTM_LENGTH  (16)

struct Euclidean {
  uint8_t steps;
  uint8_t hits;
  uint8_t offset;
};

Euclidean rhytm;
Euclidean rhytmTemp;

uint16_t potTemp;
uint16_t output;
uint16_t gate;
bool gateHigh;
uint8_t index;

void setup() {
  TCCR2A = _BV(COM2A1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);

  pinMode(PWM_PIN, OUTPUT);

  output = 0;
  gate = 0;
  gateHigh = false;
  index = 0;

  rhytm.steps = 16;
  rhytm.hits = 4;
  rhytm.offset = 0;
  rhytmTemp.steps = rhytm.steps;
  rhytmTemp.hits = rhytm.hits;
  rhytmTemp.offset = rhytm.offset;

  Serial.begin(9600);
}

SIGNAL(PWM_INTERRUPT) {
  PWM_VALUE = output;
}

uint8_t readPot(uint8_t pot, uint8_t _max) {
  potTemp = analogRead(pot);
  return map(potTemp, 0, POT_MAX, 0, _max);
}

void handlePots() {
  rhytmTemp.steps = readPot(POT1, RHYTM_LENGTH - 1);
  rhytmTemp.hits = readPot(POT2, rhytm.steps);
  rhytmTemp.offset = readPot(POT3, RHYTM_LENGTH - 1);

  if (rhytmTemp.steps != rhytm.steps
    || rhytmTemp.hits != rhytm.hits
    || rhytmTemp.offset != rhytm.offset) {
    updateRhytm(rhytmTemp);
  }
}

void updateRhytm(Euclidean newRhytm) {
  rhytm.steps = newRhytm.steps;
  rhytm.hits = newRhytm.hits;
  rhytm.offset = newRhytm.offset;

  Serial.print(newRhytm.steps);
  Serial.print(" ");
  Serial.print(newRhytm.hits);
  Serial.print(" ");
  Serial.println(newRhytm.offset);
}

void loop() {
  handlePots();
  gate = analogRead(PIN3);

  /*
  if (gate > 64) {
    if (!gateHigh) {
      gateHigh = true;

      Serial.print(rhytm.steps);
      Serial.print(" ");
      Serial.print(rhytm.hits);
      Serial.print(" ");
      Serial.print(rhytm.offset);
      Serial.print(" ");
      Serial.println(index);

      if (++index >= (rhytm.steps + 1)) {
        index = 0;
      }
    }
  } else {
    gateHigh = false;
  }
  */
}
