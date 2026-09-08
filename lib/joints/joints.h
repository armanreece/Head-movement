#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// The real pulse limits of a hobby servo, in microseconds. setPulseUs()
// clamps to these.
//
// They are deliberately NOT derived from the SERVO_MIN/SERVO_MAX counts that
// set() uses. Those describe set()'s own 0-180 map, and the 150-600 count
// window works out at 732-2930 us: too narrow at the bottom to reach a servo
// end stop, and past what a servo will accept at the top. That was invisible
// until an axis needed its rest near one end of the range - a horn seated a
// quarter turn out - at which point the bottom of its travel silently
// clamped and the axis appeared to have lost movement for no reason.
// *** Widened 4 Sep 2026. ***
static const float SERVO_PULSE_MIN_US = 500.0f;
static const float SERVO_PULSE_MAX_US = 2500.0f;

// One servo, plus everything that is true about how it is mounted.
// This is the per-servo calibration data.
struct Joint {
  uint8_t channel;
  int16_t limitA;      // one end of safe travel
  int16_t limitB;      // the other end
  bool    reversed;    // mounted mirrored
  int8_t  trim;        // per-servo offset, degrees
};

// Owns one PCA9685. Subsystems are handed a reference to this; they
// never create their own. That is what lets two subsystems share a
// board, or sit on different boards, without their code changing.
class JointBank {
public:
  explicit JointBank(uint8_t i2cAddress = 0x40);
  void begin();

  // Angle is a FLOAT on purpose. The PCA9685 has 12-bit resolution,
  // which over the servo's 180 degrees works out around 0.4 degrees
  // per step. Passing whole degrees threw most of that away - on the
  // jaw, whose whole range is 15 degrees, it meant just 16 possible
  // positions and visibly stepped motion however good the easing was.
  void set(const Joint &j, float angle);

  // Raw pulse width on one channel, in microseconds, bypassing the
  // angle map completely. For CONTINUOUS-ROTATION servos, whose pulse
  // means speed rather than position.
  //
  // Why this exists: set() maps 0-180 onto counts 150-600, so "90" comes
  // out at roughly 1830 us. A continuous servo reads 1830 us as "full
  // speed one way", not "stand still". Anything that drives one of those
  // has to talk in microseconds, and 1500 us is the nominal stop.
  //
  // Pass 0 (or anything below 1) to switch the channel fully off - no
  // pulse at all. For a continuous servo that is the only stop that is
  // guaranteed not to creep.
  void setPulseUs(uint8_t channel, float us);

  // Number of PCA9685 counts per degree, for reference.
  static float countsPerDegree();

private:
  Adafruit_PWMServoDriver pwm;
  uint16_t lastCount[16];   // skip redundant I2C writes
};
