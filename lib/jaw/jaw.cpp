#include "Jaw.h"

// Smootherstep: 6t^5 - 15t^4 + 10t^3.
//
// Replaces the cosine ease. Cosine reaches zero velocity at each end
// but its ACCELERATION is at maximum there, so chaining gestures gave
// a small jolt at every boundary. Smootherstep has zero velocity and
// zero acceleration at both ends, so gestures flow into each other.
static inline float smootherstep(float t) {
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

Jaw::Jaw(JointBank &b, Joint l, Joint r, int shut, int gape)
  : bank(b), left(l), right(r),
    shutAngle(shut), gapeAngle(gape), currentPercent(0.0f),
    count(0), index(0), fromPercent(0.0f), toPercent(0.0f),
    durationMs(0), gestureStart(0), speaking(false) {}

void Jaw::begin() {
  setGape(0.0f);
}

float Jaw::gapeToAngle(float percent) const {
  return (float)shutAngle +
         ((float)gapeAngle - (float)shutAngle) * (percent / 100.0f);
}

void Jaw::setGape(float percent) {
  if (percent < 0.0f)   percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  float angle = gapeToAngle(percent);
  bank.set(left, angle);
  bank.set(right, angle);
  currentPercent = percent;
}

// Build a phrase as a list of gape targets and durations.
// Roughly: consonants close the jaw, vowels open it, stressed vowels
// open widest. Only bilabials (m, b, p) fully close - most consonants
// leave the mouth slightly open, which is what stops it looking like
// a snapping puppet.
void Jaw::buildPhrase() {
  count = 0;
  int words = (int)random(2, 5);

  for (int w = 0; w < words && count < MAX_GESTURES - 2; w++) {
    int syllables = (int)random(1, 4);
    for (int s = 0; s < syllables && count < MAX_GESTURES - 2; s++) {
      bool stressed = (s == 0) || (random(100) < 25);

      gestures[count].gape = (random(100) < 30) ? 0 : (uint8_t)random(10, 30);
      gestures[count].ms   = (uint16_t)random(50, 90);
      count++;

      gestures[count].gape = stressed ? (uint8_t)random(65, 101)
                                      : (uint8_t)random(35, 65);
      gestures[count].ms   = stressed ? (uint16_t)random(110, 180)
                                      : (uint16_t)random(80, 130);
      count++;
    }
  }

  // Settle relaxed, not clamped shut. A clamped mouth looks dead.
  gestures[count].gape = 6;
  gestures[count].ms   = 250;
  count++;
}

void Jaw::startGesture(int i) {
  fromPercent  = currentPercent;
  toPercent    = (float)gestures[i].gape;
  durationMs   = (int)gestures[i].ms;
  gestureStart = millis();
}

void Jaw::speak() {
  if (speaking) return;
  buildPhrase();
  index = 0;
  startGesture(0);
  speaking = true;
}

void Jaw::update() {
  if (!speaking) return;

  // Rollover-safe: unsigned subtraction stays correct across the
  // ~49 day millis() wrap.
  unsigned long elapsed = millis() - gestureStart;

  if (durationMs <= 0 || elapsed >= (unsigned long)durationMs) {
    setGape(toPercent);
    index++;
    if (index >= count) { speaking = false; return; }
    startGesture(index);
    return;
  }

  float t = (float)elapsed / (float)durationMs;
  setGape(fromPercent + (toPercent - fromPercent) * smootherstep(t));
}
