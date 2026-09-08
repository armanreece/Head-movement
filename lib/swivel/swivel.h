#pragma once
#include "Joints.h"

// Head swivel, second build (3 Sep 2026): a CONTINUOUS-ROTATION micro
// servo with a pinion glued to its horn, driving the ring gear the head's
// base is bolted to, at roughly 4:1. This is the gear drive Martin
// proposed after the crank version ran out of rotation.
//
// A continuous servo is a fundamentally different actuator from every
// other servo on the head, and the whole class is shaped by that:
//
//   * A position servo is told an ANGLE and goes there on its own. A
//     continuous servo is told a SPEED - the pulse width sets how fast it
//     spins and which way - and it spins until told otherwise. It has no
//     idea where it is, and neither does the Uno.
//
//   * So position is DEAD-RECKONED. Every frame, the speed the servo was
//     told to run at is multiplied by the frame time and added to an
//     estimate. The estimate is only as good as the speed calibration,
//     and it drifts: every stall, knock, brownout and Uno reset makes it
//     wrong. Nothing in this file can know the head has been bumped.
//
//   * Drift is not just cosmetic. Every other servo's lead runs through
//     the middle of the ring gear. A head that believes it is centred
//     while actually sitting 90 degrees round will wind the harness up on
//     its next glance. The software limit (maxDeg) is what prevents that,
//     and it can only ever be enforced against the estimate.
//
//   * Hence the HOME SENSOR hook. A switch or sensor that changes state
//     as the head passes straight ahead snaps the estimate back to zero
//     every time it does, so drift can never build up beyond one
//     excursion. Without one, the head must be straightened by hand
//     before power-up and re-zeroed (Z command) whenever it wanders.
//     FIT THE SENSOR. It is one part, one wire and one printed cam.
//
//   * When the head is NOT moving the channel is switched fully off - no
//     pulse at all. A continuous servo parked on its "stop" pulse creeps
//     unless that pulse is exactly right, hums, and draws current doing
//     it. No pulse means none of that, and a yaw axis has no gravity load
//     to hold against, so nothing is lost.
//
// Motion is planned as a VELOCITY profile, which is the natural language
// of a speed-controlled actuator: accelerate at a fixed rate, cruise at a
// cap, and brake on a square-root curve so the head arrives with the same
// deceleration it left with. Ease-in and ease-out for free, and because
// the plan is remade every frame, a target that moves mid-turn - the Pi
// tracking a face - is followed smoothly rather than restarting the turn.
//
// NON-BLOCKING: call update() every pass through loop().

// What the home sensor tells us.
//   HOME_SIDE   - active over one whole HALF of the travel, e.g. a microswitch
//                 riding a half-moon cam, or a slot sensor with a half-disc
//                 flag. The edge IS centre. Best option: one read at boot
//                 says which way to turn, so homing never overshoots, and
//                 every crossing re-zeros the estimate.
//   HOME_WINDOW - active only within a few degrees of centre, e.g. a Hall
//                 sensor and one magnet. Works, but at boot the head cannot
//                 know which side it is on and has to search blind, which
//                 can carry it past the limit on the wrong side first.
enum HomeKind : uint8_t { HOME_NONE = 0, HOME_SIDE = 1, HOME_WINDOW = 2 };

struct SwivelConfig {
  uint8_t  channel;          // PCA9685 channel of the continuous servo
  int8_t   dir;              // +1 or -1. Flip if +yaw turns the head LEFT.
  float    stopUs;           // pulse at which the servo stands still, nominal 1500
  float    slowUs;           // offset from stopUs giving the slowest reliable motion
  float    fastUs;           // offset from stopUs giving full speed
  // The same two offsets for the OTHER direction. *** Added 4 Sep 2026. ***
  // A continuous servo does not sit symmetrically inside its own
  // deadband: stopUs is a nominal 1500 and the true stop is wherever the
  // trim pot was left, so if the true stop is below stopUs the servo is
  // already being nudged one way at "stop", and the same offset either
  // side lands harder on that side than the other. The head then turns
  // noticeably further one way for the same command.
  //
  // Note the error is ADDITIVE, not proportional: a true stop 20 us below
  // stopUs makes every clockwise pulse 20 us stronger and every
  // anticlockwise one 20 us weaker, whatever the offset. So the correction
  // adds microseconds rather than scaling them, and adding 2x the stop
  // error here is exactly equivalent to correcting stopUs - just without
  // touching the direction that already works.
  //
  // 0 in either falls back to the + value.
  float    slowUsNeg;
  float    fastUsNeg;
  float    slowDegPerS;      // HEAD speed measured at slowUs, turning +
  float    fastDegPerS;      // HEAD speed measured at fastUs, turning +
  // The SAME two measurements taken turning the other way. *** Added
  // 4 Sep 2026. *** A continuous servo is not symmetric: its stop point
  // is a trim pot somebody set by hand, and the response either side of
  // the deadband differs, so the same pulse offset gives different head
  // speeds in the two directions - 10 or 20 percent apart is ordinary.
  // calibrate() always spun both ways and asked for the degrees moved,
  // but there was only one pair of numbers to write them into, so one
  // measurement was necessarily discarded. With a single speed model the
  // slower direction's estimate runs ahead of the head, the planner
  // believes it has arrived early, and that direction consistently turns
  // SHORT. Leave these 0 to fall back to the + values.
  float    slowDegPerSNeg;
  float    fastDegPerSNeg;
  float    maxDeg;           // software limit either side of centre
  int8_t   homePin;          // Uno pin of the sensor, -1 for none
  HomeKind homeKind;
  int8_t   homeLowSide;      // HOME_SIDE: which side reads LOW, -1 left / +1 right
  float    homeHalfWidthDeg; // HOME_WINDOW: half the arc the sensor reads over
};

class Swivel {
public:
  Swivel(JointBank &bank, const SwivelConfig &cfg);

  // Parks the channel off and takes wherever the head is as centre.
  void begin();
  // Blocking search for the home sensor. Returns true if found. With no
  // sensor configured it returns false and begin()'s assumption stands.
  bool homeNow();
  void update();                       // call every loop

  // ---- Position mode ----
  // -100 = full left, 0 = straight ahead, +100 = full right, as a
  // percentage of maxDeg. Sets a target; update() drives toward it.
  void setYaw(float percent);
  void setYawDeg(float deg);

  // ---- Rate mode ----
  // -100..+100 percent of the driven speed cap, negative = left. This is
  // the primitive for closing a loop around the camera: the Pi sends a
  // rate proportional to how far off-centre the face is, and zero when
  // it is centred. A rate expires after RATE_HOLD_MS, so if the Pi stops
  // talking mid-turn the head brakes rather than running to the limit.
  void setRate(float percent);

  // Come to a smooth stop where we are. Used on every mode change.
  void hold();
  // Pulse off this instant. Fault path; not smooth.
  void stopNow();

  float yawDeg()     const { return posDeg; }
  float yawPercent() const;
  bool  isMoving()   const { return pulseOn; }
  bool  atTarget()   const;
  float remainingDeg() const { return intentDeg - posDeg; }

  // Declare the current physical position to be centre.
  void zeroHere();
  bool isHomed() const { return homed; }

  // Idle glances when the head is running on its own. Off when the Pi is
  // driving, for the same reason as the neck wander.
  void setWander(bool on);

  // True exactly once, on the frame an idle glance has been decided but
  // before the head starts to move. The eyes get this beat's head start;
  // remainingDeg() then says how far ahead of the head the target is.
  bool glanceStarted();

  // Speed caps in HEAD degrees per second, and the shared acceleration.
  // Idle glances should be unhurried; a driven turn can be brisker.
  void setIdleSpeed(float degPerS)   { idleSpeed   = degPerS; }
  void setDrivenSpeed(float degPerS) { drivenSpeed = degPerS; }
  void setAccel(float degPerS2)      { accel = degPerS2; }
  // How long the eyes lead the head on an idle glance.
  void setEyeLead(unsigned int ms)   { eyeLeadMs = ms; }

  // Idle glances are PAIRED: every turn away from centre is followed by
  // the return that undoes it, to exactly the same place, after a pause.
  // Turn 40 clockwise, hold, turn 40 back, rest, and the next excursion
  // goes anticlockwise. No coin anywhere in it, so the commanded arc is
  // equal in the two directions by construction and the estimate sits at
  // zero between excursions.
  //
  // Replaced a home-interval timer and a side-swap timer on 4 Sep 2026.
  // Pairing makes both unnecessary: the head is never away from centre
  // for longer than one hold, and never goes the same way twice.
  void setLookHold(unsigned long minMs, unsigned long maxMs) {
    lookHoldMin = minMs; lookHoldMax = maxMs;
  }
  void setHomeRest(unsigned long minMs, unsigned long maxMs) {
    homeRestMin = minMs; homeRestMax = maxMs;
  }

  void demo();                         // blocking direction check
  void calibrate();                    // blocking timed spins for slow/fast speeds

  // Stale rate commands die after this long.
  static const unsigned long RATE_HOLD_MS = 600;

private:
  enum Mode { POSITION, RATE };

  void  drive(float degPerS);          // map a head speed onto a pulse
  float slowFor(float degPerS) const;  // per-direction speed calibration
  float slowUsFor(float degPerS) const;
  float fastUsFor(float degPerS) const;
  float fastFor(float degPerS) const;
  void  pollHome(unsigned long now);
  void  chooseGlance(unsigned long now);
  bool  homeSweep(int8_t direction, float maxDistanceDeg, bool startState);
  bool  sensorActive() const;
  void  runFor(unsigned long ms);      // blocking: update() until time is up
  float sgn(float v) const { return v < 0.0f ? -1.0f : 1.0f; }

  JointBank   &bank;
  SwivelConfig cfg;

  // Estimate and plan
  float posDeg;        // dead-reckoned head angle, 0 = centre
  float vPhys;         // head speed the current pulse is modelled to give
  float vCmd;          // planner's velocity state (may sit below the servo floor)
  bool  pulseOn;
  Mode  mode;
  float targetDeg;     // active position target
  float intentDeg;     // where we are going, including a not-yet-started glance
  float rateTarget;    // deg/s wanted in rate mode
  unsigned long rateUntil;
  float settleDeg;     // "close enough" - derived from the floor speed

  // Tuning
  float idleSpeed, drivenSpeed, accel;
  unsigned int eyeLeadMs;
  unsigned long lookHoldMin, lookHoldMax;   // pause at the far end
  unsigned long homeRestMin, homeRestMax;   // pause back at centre

  // Idle glances
  bool   awayFromHome;           // an excursion is out, so the next glance
                                 // is its matching return
  int8_t lastLookSide;           // side of the last excursion, so the next
                                 // one goes the other way
  bool  wanderOn;
  bool  glanceFlag;
  bool  glancePending;
  unsigned long glanceAt;
  unsigned long nextGlance;

  // Home sensor
  bool homed;
  bool sensorWas;

  unsigned long lastUpdate;
};