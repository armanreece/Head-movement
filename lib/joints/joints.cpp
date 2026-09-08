#include "Joints.h"

// Replace with measured values from the calibration harness
static const int SERVO_MIN = 150;
static const int SERVO_MAX = 600;

// The board runs at 50 Hz, so one 20 ms period is 4096 counts and one
// count is about 4.88 us. SERVO_MIN/MAX above are on the same scale
// (150 = ~730 us, 600 = ~2930 us), so this and set() agree with each
// other even though the PCA9685's internal oscillator is never exactly
// on frequency - everything is calibrated empirically against the same
// clock, so the error cancels.
static const float COUNTS_PER_US = 4096.0f / 20000.0f;

// setPWM(ch, 0, 4096) is the library's "fully off" - no pulse at all.
static const uint16_t COUNT_OFF = 4096;

JointBank::JointBank(uint8_t i2cAddress) : pwm(i2cAddress) {
  for (uint8_t i = 0; i < 16; i++) lastCount[i] = 0xFFFF;   // "unknown"
}

void JointBank::begin() {
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // An Uno reset does not reset the PCA9685, and the library's begin()
  // only restarts it: every channel keeps whatever pulse it last had.
  // For a position servo that is harmless. For the continuous swivel
  // servo it means "keep turning at the speed you were told before the
  // reset" until something says otherwise, so say otherwise first.
  for (uint8_t i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, COUNT_OFF);
    lastCount[i] = COUNT_OFF;
  }
}

float JointBank::countsPerDegree() {
  return (float)(SERVO_MAX - SERVO_MIN) / 180.0f;
}

void JointBank::set(const Joint &j, float angle) {
  // Clamp to the joint's mechanical range first. Nothing upstream can
  // drive a joint past what the mechanism physically allows.
  float lo = (float)min(j.limitA, j.limitB);
  float hi = (float)max(j.limitA, j.limitB);
  if (angle < lo) angle = lo;
  if (angle > hi) angle = hi;

  // Then per-servo correction
  float out = angle + (float)j.trim;
  if (j.reversed) out = 180.0f - out;
  if (out < 0.0f)   out = 0.0f;
  if (out > 180.0f) out = 180.0f;

  // Rounded, not truncated. Truncation biases every command downward
  // by up to a full count, which shows up as a slight stutter.
  int counts = SERVO_MIN + (int)((out / 180.0f) * (SERVO_MAX - SERVO_MIN) + 0.5f);
  if (counts < SERVO_MIN) counts = SERVO_MIN;
  if (counts > SERVO_MAX) counts = SERVO_MAX;

  // The servo cannot tell the difference if the count has not changed,
  // so skip the write. Less I2C traffic means less frame-time jitter.
  if (j.channel < 16) {
    if (lastCount[j.channel] == (uint16_t)counts) return;
    lastCount[j.channel] = (uint16_t)counts;
  }

  pwm.setPWM(j.channel, 0, counts);
}

void JointBank::setPulseUs(uint8_t channel, float us) {
  if (channel >= 16) return;

  uint16_t counts;
  if (us < 1.0f) {
    counts = COUNT_OFF;
  } else {
    // Clamped to the servo's real range, NOT to set()'s count window -
    // see SERVO_PULSE_MIN_US in Joints.h. An axis whose rest sits near one
    // end needs the whole 500-2500 us, and the count window would quietly
    // eat the last 230 us of it.
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
    int c = (int)(us * COUNTS_PER_US + 0.5f);
    counts = (uint16_t)c;
  }

  if (lastCount[channel] == counts) return;
  lastCount[channel] = counts;
  pwm.setPWM(channel, 0, counts);
}
