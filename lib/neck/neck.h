#pragma once
#include "Joints.h"

// The neck: the head sits on a two-axis pivot and TWO servos tilt it, one
// per axis, each turning a pulley with a string loop over it. Turning the
// pulley one way pulls one side of the platform down and pays the other
// side out by the same amount, so ONE servo does both directions of its
// axis. There is no antagonistic pair, nothing to keep slack, and nothing
// to keep taut - that is what makes this mechanism simpler than the
// four-cable neck it replaced (3 Sep 2026).
//
//   tilt  ch 4  - head forward / back  (nodding)
//   pan   ch 5  - head left / right    (a sideways lean, ear toward shoulder)
//
// NOTE WHAT "PAN" NOW MEANS. On the old neck "pan" was looking left and
// right. Rotation is now the swivel's job (the Y command); the neck's
// left/right axis is a LEAN. The N command keeps its name and shape so the
// Pi side did not have to change, but a Pi that wants the head to turn
// toward someone should send Y, not N.
//
// THE NECK IS DRIVEN IN MICROSECONDS, NOT THROUGH JointBank::set().
// set() maps a 0-180 "angle" onto PCA9685 counts 150-600, and that scale is
// NOT servo degrees: its "90" is a ~1830 us pulse, which on a standard
// servo is 30-60 degrees past centre, and its "45 degrees" of travel is
// 50-100 real degrees. That is exactly what put both neck servos hard
// against their strings on 4 Sep 2026. The eyes get away with the same map
// because their rest angles were found by sweeping, so the units never
// mattered. For the neck the units DO matter - the string limit is a real
// angle - so each axis is described in real terms:
//
//   restUs     the pulse, in microseconds, at which the head is upright on
//              that axis with both sides of the string evenly taut.
//              1500 us is the nominal centre of every hobby servo.
//   travelDeg  signed degrees the HORN turns for a full-scale (+100)
//              command. SIGN = direction, MAGNITUDE = the string limit.
//   usPerDeg   the servo's own scale: pulse change per degree of horn.
//
// Nothing can drive an axis past rest +/- |travelDeg| - the percent is
// clamped in look(), and the pulse is clamped again in servoUs().
//
// EVERY MOVE IS EASED, in here, not by the caller. look() only sets a
// target; update() carries the head there with a speed and acceleration
// limit, so nothing upstream - not the Pi, not a typo in a serial monitor -
// can snap the head from one side to the other and shock-load the strings.
// A target that changes mid-move is simply followed, with no stutter.
//
// IDLE. A resting head is mostly still. It drifts a little (breathing),
// now and then takes up a new pose - a slight lean, a small nod - holds it,
// and keeps coming back to upright. While the head is TALKING it does
// something else: small nods and shifts of weight, riding on top of
// whatever pose the Pi has asked for. And while it is LISTENING it does
// nothing at all, because servo whine sits in the speech band and the mic
// is a few inches away. See NeckIdle.
//
// NON-BLOCKING: call update() every pass through loop().

struct NeckAxis {
  uint8_t channel;
  float   restUs;      // head upright: pulse in microseconds
  float   travelDeg;   // signed horn degrees at +100. Sign = direction, |x| = limit
  float   usPerDeg;    // this servo's microseconds per horn degree
};

enum NeckIdle : uint8_t {
  NECK_STILL,   // hold the commanded pose. Listening.
  NECK_TALK,    // small nods and shifts around the commanded pose. Speaking.
  NECK_REST     // full idle: breathe, take up poses, return to upright.
};

class Neck {
public:
  Neck(JointBank &bank, NeckAxis pan, NeckAxis tilt);

  void begin();
  void update();                       // call every loop

  // Set the pose the head should carry. Eased; returns immediately.
  //   pan  : -100 = lean left,    0 = upright, +100 = lean right
  //   tilt : -100 = head forward, 0 = upright, +100 = head back
  // In NECK_REST mode the idle poses overwrite this. In NECK_TALK mode
  // the talking gestures ride on top of it.
  void look(float pan, float tilt);

  // Stop generating any motion of our own and settle on the current
  // commanded pose (gestures and drift ease out; the pose stays).
  void hold();

  // Where the head is right now, percent of travel. The pose plus any
  // gesture or drift currently on top of it.
  float pan()  const { return curPan; }
  float tilt() const { return curTilt; }
  bool  atTarget() const;

  // The same, as horn degrees from upright and as the pulse being sent.
  // For printing while calibrating; nothing in the motion path uses them.
  float panDeg()  const { return panAx.travelDeg  * curPan  / 100.0f; }
  float tiltDeg() const { return tiltAx.travelDeg * curTilt / 100.0f; }
  float panUs()   const { return servoUs(panAx,  curPan);  }
  float tiltUs()  const { return servoUs(tiltAx, curTilt); }

  // Idle behaviour. setWander() is the older on/off form and maps to
  // NECK_REST / NECK_STILL.
  void setIdle(NeckIdle mode);
  NeckIdle idle() const                { return mode; }
  void setWander(bool on)              { setIdle(on ? NECK_REST : NECK_STILL); }
  bool isWandering() const             { return mode == NECK_REST; }

  // Pace. Speeds in percent-of-travel per second, accel in %/s^2. Idle
  // speed is used for the head's own poses; driven speed for look() when
  // the Pi is in charge; accel is shared. Lower accel = softer starts and
  // stops, and less load on the strings.
  void setIdleSpeed(float pctPerS)     { idleSpeed = pctPerS; }
  void setDrivenSpeed(float pctPerS)   { drivenSpeed = pctPerS; }
  void setAccel(float pctPerS2)        { accel = pctPerS2; }

  // How far the idle poses reach, percent of travel, FOUR values - one per
  // axis per direction. Split four ways on 4 Sep 2026 because the two
  // directions of an axis are rarely equal in practice: one may be the
  // pretty one, and one may be all the mechanism physically has (with a
  // rest position near the end of a servo's range, the short side can be
  // half the long one, and asking for more just parks it on the clamp).
  //
  // Arguments are by POSE SIGN, not by rotation: panPos is how far a
  // positive pan pose reaches. Which rotation that is depends on the sign
  // of that axis's TRAVEL_DEG, so main.cpp does the mapping and keeps the
  // rotation names in one place.
  void setIdleReach(float panPos, float panNeg, float tiltPos, float tiltNeg) {
    reachPanPos = panPos; reachPanNeg = panNeg;
    reachTiltPos = tiltPos; reachTiltNeg = tiltNeg;
  }

  // Named direction check, one axis at a time, eased like real motion.
  // Blocking on purpose: it is a setup check, not normal running.
  void demo();

private:
  struct Axis {
    float cur, vel, target;    // percent of travel
  };

  void  write();
  void  stepAxis(Axis &a, float vmax, float dt);
  void  planRestPose(unsigned long now);
  void  planTalkGesture(unsigned long now);
  static void clampGesture(float &gest, float pose, float drift);
  // Reach in the direction that dips the head forward / tips it back, in
  // percent of travel, whichever pose sign that happens to be.
  float tiltForwardReach() const { return reachTiltNeg; }
  float tiltBackReach()    const { return reachTiltPos; }
  float servoUs(const NeckAxis &ax, float pct) const;

  JointBank &bank;
  NeckAxis   panAx, tiltAx;

  // Layers. What the head actually aims for is pose + gesture + drift.
  float posePan, poseTilt;         // what look() or the idle planner set
  float gestPan, gestTilt;         // talking nods / shifts, decay to 0
  float driftPan, driftTilt;       // slow breathing sway, tiny
  Axis  pan_, tilt_;               // the eased result, and current speed
  float curPan, curTilt;           // public copies of pan_.cur, tilt_.cur

  NeckIdle mode;
  float idleSpeed, drivenSpeed, accel;
  float reachPanPos, reachPanNeg;   // percent of travel, by pose sign
  float reachTiltPos, reachTiltNeg;
  bool  poseAway;                   // last pose was off-centre, so the next
                                    // one is the matching return
  float moveSpeed;                 // speed for the pose currently in progress

  // Idle planning
  unsigned long nextPose;          // NECK_REST: when to choose again
  unsigned long nextGesture;       // NECK_TALK: when the next nod starts
  unsigned long gestureEnd;        // when the current nod releases
  int8_t lastPanSign, lastTiltSign; // which side the last pose went, so the
                                   // next one can be drawn away from it
  float driftPhaseA, driftPhaseB;  // accumulated, see neck.cpp
  unsigned long lastUpdate;
  bool firstFrame;
};