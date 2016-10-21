
/**
 * Re-code of the firmware for the exii hackberry hand.
 * This recode brings a better signal filtering, and removes the need for calibration.
 * It also uses 2 sensors to control the opening/closing of the hand.
 */

#include <RunningMedian.h>
#include <Servo.h>

// RunningMedia configuration
#define MEDIAN_MAX_SIZE     41

// Magic numbers configuration
#define ACTIVE_EDGE_DETECTION 10 // if the sensor is idle and a difference greater than this value is detected, switch to MYO_ACTIVE state
#define IDLE_EDGE_DETECTION 15   // if the sensor is active and its value goes back to its (pre-active value + 15) or lower, switch back to MYO_IDLE state
#define SERVO_TICK_MS 15         // servo value update cycle

// tick counter
unsigned long counter = 0;

/**
 * Fingers structure and functions
 */

struct finger {
  Servo servo;
  float min;
  float max;

  float angle;
  RunningMedian median = RunningMedian(3);

  unsigned long lastUpdate;
};

void setupFinger(struct finger *f, int pin, float min, float max) {
  f->servo.attach(pin);
  f->min = min;
  f->max = max;
  f->angle = min + (max - min) / 2;
  f->lastUpdate = millis();
}

void loopFinger(struct finger *f) {
  f->angle = min(f->max, max(f->min, f->angle));  // limit values between min/max for this servo
  f->median.add(f->angle);                        // servo angle smooth filtering

  if (millis() - f->lastUpdate > SERVO_TICK_MS) { // update servo value, every SERVO_TICK_MS ms
    f->lastUpdate = millis();
    f->servo.write(f->median.getAverage());
  }
}

/**
 * Myo structure and functions
 */

#define MYO_IDLE 0
#define MYO_ACTIVE 1

struct myo {
  RunningMedian shortMedian = RunningMedian(41);   // short term filtering
  RunningMedian longMedian = RunningMedian(3);     // long term filtering, used as a reference, to detect changes
  RunningMedian diffMedian = RunningMedian(21);    // used to to do the median of the difference between shortMedian and longMedian
  RunningMedian minDiffMedian = RunningMedian(21); // used to to do the median of the difference between shortMedian and minimumThreshold, to detect the return to normal

  int state;                                       // holds the current state for the myo, either MYO_IDLE or MYO_ACTIVE
  int pin;                                         // analog pin to read the sensor value
  float minimumThreshold;                          // holds the value used as reference for the return to normal, calculated when state goes to MYO_ACTIVE
};

void setupMyo(struct myo *m, int pin) {
  m->pin = pin;
}

/**
 * This function updates the state of a myo
 * multiple layers of filtering are used to detect changes of intensity and return to normal
 */
bool loopMyo(struct myo *m) {
  // update all median filtering calculations
  m->shortMedian.add(analogRead(m->pin));
  if (counter % 10 == 0) {
    m->longMedian.add(m->shortMedian.getMedian());
  }
  m->diffMedian.add(m->shortMedian.getMedian() - m->longMedian.getMedian());
  m->minDiffMedian.add(m->shortMedian.getMedian() - m->minimumThreshold);

  // Edges detection
  if (m->state == MYO_IDLE && m->diffMedian.getMedian() > ACTIVE_EDGE_DETECTION) { // if the median of the difference between shortMedian and longMedian is over 10
    m->state = MYO_ACTIVE;
    m->minimumThreshold = m->longMedian.getMedian();
    m->minDiffMedian.clear();
    m->minDiffMedian.add(ACTIVE_EDGE_DETECTION);
    return true;
  } else if (m->state == MYO_ACTIVE && m->minDiffMedian.getAverage() < IDLE_EDGE_DETECTION) {
    m->state = MYO_IDLE;
  }
  return false;
}

void resetMyo(struct myo *m) {
  m->state = MYO_IDLE;
  m->minimumThreshold = 0;
  m->shortMedian.clear();
  m->longMedian.clear();
  m->diffMedian.clear();
  m->minDiffMedian.clear();
}

/**
 * State variable holders
 */
struct myo m1;
struct myo m2;

struct finger thumb;
struct finger index;
struct finger middle;

/**
 * Arduino standard functions
 */

void setup() {
  pinMode(7, INPUT);
  digitalWrite(7, HIGH);
  
  setupMyo(&m1, A0);
  setupMyo(&m2, A6);

  setupFinger(&thumb, 6, 60, 158);
  setupFinger(&index, 3, 27, 150);
  setupFinger(&middle, 5, 35, 150);

  Serial.begin(115200);
}

void loop() {
  if (digitalRead(7) == LOW) {
    resetMyo(&m1);
    resetMyo(&m2);
  }

  // if a sensor turns active, set the other one to MYO_IDLE
  if (loopMyo(&m1)) {
    m2.state = MYO_IDLE;
  }
  if (loopMyo(&m2)) {
    m1.state = MYO_IDLE;
  }

  // update servos angles
  if (m1.state == MYO_ACTIVE) {
    index.angle += m1.minDiffMedian.getMedian() / 700;
    middle.angle = map(index.angle, index.min, index.max, middle.max, middle.min);
  }

  if (m2.state == MYO_ACTIVE) {
    index.angle -= m2.minDiffMedian.getMedian() / 700;
    middle.angle = map(index.angle, index.min, index.max, middle.max, middle.min);
  }

  // Finger loop functions
  loopFinger(&thumb);
  loopFinger(&index);
  loopFinger(&middle);

  Serial.print(m1.shortMedian.getMedian());
  Serial.print(", ");
  Serial.print(m1.minimumThreshold);
  Serial.print(", ");
  Serial.print(m2.shortMedian.getMedian());
  Serial.print(", ");
  Serial.print(m2.minimumThreshold);
  //Serial.print(index.median.getMedian());
  Serial.println();
  ++counter;
}
