#pragma once
#include "Joints.h"

// The eyes: three servos per side driving each eye frame through wire
// pushrods.
//
//   pan   - eyes left / right
//   tilt  - eyes up / down
//   lid   - eyelids
//
// ONE logical pair of eyes, up to SIX servos. Callers ask for a gaze or a
// blink; they never see channels, sides or mirroring. The two banks are
// physically mirror images of each other, so the right eye's servos run
// backwards relative to the left - but rather than a reversed flag, each
// side carries its OWN rest angles and its own signed travels. Horns are
// never seated identically on both sides, so giving each side independent
// calibration is the difference between eyes that track together and eyes
// that almost do.
//
// EVERY AXIS IS REST + OFFSET. Each servo has one rest angle, which is
// where it sits at power-up and where every movement returns to, and a
// signed travel saying how far and which way it goes. Zero offset always
// means the position the eyes were assembled in.
//
// What makes eyes read as alive is not smoothness - it is the opposite.
// Real eyes FIXATE and JUMP (saccades), they do not glide, so the gaze
// easing here is deliberately fast and the idle wander picks a point,
// stares at it, then snaps somewhere else. Blinks are asymmetric: the lid
// drops fast and rises slower, which is how actual blinks work.
//
// Blinking is involuntary, so the auto-blink keeps running even when the
// Pi owns the gaze. A blink always ends back at the lid's rest position.
//
// NON-BLOCKING: call update() every pass through loop().

// One eye's worth of hardware and calibration.
struct EyeBank {
  Joint pan, tilt, lid;
  int panRest,  panTravel;    // +ve travel = eyes RIGHT
  int tiltRest, tiltTravel;   // +ve travel = eyes UP
  int lidRest,  lidTravel;    // rest = open, travel = toward shut
};

class Eyes {
public:
  // One eye, for bench testing a single bank.
  Eyes(JointBank &bank, EyeBank left);

  // Both eyes, driven together.
  Eyes(JointBank &bank, EyeBank left, EyeBank right);

  void begin();
  void update();                    // call every loop

  // Gaze, as an offset from rest. x: -100 = left, +100 = right.
  // y: -100 = down, +100 = up. (0, 0) is always the assembled position.
  void look(float x, float y);

  // Everything back to rest, immediately. Gaze centred, lids open.
  void rest();

  // One blink, starting now. Safe to call any time, even mid-blink.
  void blink();

  // Base eyelid position, 0 = rest (open) .. 100 = fully shut. An
  // EXPRESSION control: half-shut reads as sleepy or unimpressed. Blinks
  // ride on top of it and return here afterwards.
  void setLids(float percent);

  // Idle saccades on/off. Auto-blink is separate and stays on by default.
  void setWander(bool on)    { wanderOn = on; }
  void setAutoBlink(bool on) { autoBlink = on; }

  // Where the gaze currently is, percent of travel from rest. The head
  // swivel reads these so it can move the eyes on one axis without
  // disturbing the other.
  float gazeX() const { return xNow; }
  float gazeY() const { return yNow; }

  void demo();                      // blocking direction check

private:
  void writeGaze();
  void writeLid(float pct);
  float lidTargetNow() const;

  JointBank &bank;
  EyeBank left, right;
  bool hasRight;

  // Gaze state, percent of travel from rest
  float xNow, yNow, xTarget, yTarget;

  // Lid state, percent shut from rest
  float lidNow, lidBase;

  enum BlinkPhase { IDLE, CLOSING, CLOSED, OPENING };
  BlinkPhase phase;
  unsigned long phaseStart;

  bool wanderOn, autoBlink;
  unsigned long nextSaccade, nextBlink, lastUpdate;
};