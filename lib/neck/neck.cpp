#include "Neck.h"

// ---- Breathing drift ----
// Two slow sines with periods that are not multiples of each other, so the
// combined sway never visibly repeats. Rates are radians per millisecond.
// The phases are ACCUMULATED rather than derived from millis(): a float
// holds ~7 significant digits, so millis() itself stops being exactly
// representable after about 4.6 hours and a sin(millis()*k) wander starts
// moving in visible steps late in a long demo.
static const float DRIFT_PAN        = 4.0f;       // percent of travel
static const float DRIFT_TILT       = 3.0f;
static const float DRIFT_RATE_A     = 0.00085f;   // ~7.4 s period
static const float DRIFT_RATE_B     = 0.00061f;   // ~10.3 s period
static const float DRIFT_TALK_SCALE = 0.6f;       // subtler while talking

// ---- Idle poses (NECK_REST) ----
static const unsigned long HOLD_UPRIGHT_MIN = 4000, HOLD_UPRIGHT_MAX = 12000;
static const unsigned long HOLD_POSE_MIN    = 3000, HOLD_POSE_MAX    = 8000;
// Poses now STRICTLY ALTERNATE with upright: every pose away from centre is
// followed by the return that undoes it, by exactly the same angle, before
// another pose is chosen. *** 4 Sep 2026, replacing a 35/70 percent return
// coin. *** The coin allowed pose-to-pose chains, so a move out was not
// necessarily matched by a move back and the head could work its way across
// and stay there; and because each pose was drawn relative to CENTRE rather
// than to where the head was, those chains made the two axes drift out of
// step with each other. Alternating fixes both, and makes the return
// exactly equal and opposite by construction rather than on average.

// How strongly the next pose is drawn AWAY from the side the last one went.
// 50 would be a fair coin, which is what this was before 4 Sep 2026 - and a
// fair coin takes the same side three times running once every four poses,
// so an axis spends long stretches on one side and only crosses through
// centre on its way home. That reads as a bias even though the long-run
// average is even. 100 would be strict alternation, which reads as a
// metronome. 70 wanders.
static const int OPPOSITE_CHANCE = 70;

// Odds an idle tilt gesture is a tip BACK rather than a dip forward. Only
// at rest: while the head is talking a gesture should be a nod, and a nod
// is forward.
static const int TIP_BACK_CHANCE_REST = 45;

// ---- Gestures ----
// A nod: a dip forward that releases after a short hold. A shift: a small
// change of lean, held a little longer. While talking these come every
// second or two; at rest, rarely.
static const unsigned long GEST_TALK_GAP_MIN = 1200, GEST_TALK_GAP_MAX = 3500;
static const unsigned long GEST_REST_GAP_MIN = 9000, GEST_REST_GAP_MAX = 22000;
static const unsigned long NOD_HOLD_MIN   = 220,  NOD_HOLD_MAX   = 480;
static const unsigned long SHIFT_HOLD_MIN = 600,  SHIFT_HOLD_MAX = 1600;
static const int NOD_CHANCE_TALK = 70;       // % of talk gestures that are nods
// Raised by ~11 points (5 horn degrees) on 4 Sep 2026, not the 10 the idle
// poses gained. A gesture rides ON TOP of the pose, and pose + gesture +
// drift is clamped at 100 percent - which is the string limit. Given a
// two-thirds-of-reach pose, 25 leaves headroom; 36 would spend a chunk of
// every big gesture pressed against the clamp, holding the string at its
// tightest. Raise them if the nods still read as too polite.
static const float NOD_MIN = 19.0f,  NOD_MAX = 29.0f;   // percent, forward
static const float SHIFT_MIN = 17.0f, SHIFT_MAX = 25.0f; // percent, lean

static float clamp100(float v) {
  if (v < -100.0f) return -100.0f;
  if (v >  100.0f) return  100.0f;
  return v;
}

static float frand(float lo, float hi) {
  return lo + (hi - lo) * (float)random(0, 1000) / 1000.0f;
}

// A direction, weighted away from the one last used. With no history it is
// a fair coin.
static int8_t pickDir(int8_t last) {
  if (last == 0) return random(0, 2) ? 1 : -1;
  return (random(0, 100) < OPPOSITE_CHANCE) ? (int8_t)(-last) : last;
}

Neck::Neck(JointBank &b, NeckAxis p, NeckAxis t)
  : bank(b), panAx(p), tiltAx(t),
    posePan(0), poseTilt(0), gestPan(0), gestTilt(0),
    driftPan(0), driftTilt(0),
    curPan(0), curTilt(0),
    mode(NECK_STILL),
    idleSpeed(70.0f), drivenSpeed(180.0f), accel(600.0f),
    reachPanPos(45.0f), reachPanNeg(45.0f),
    reachTiltPos(20.0f), reachTiltNeg(35.0f), poseAway(false),
    moveSpeed(70.0f),
    nextPose(0), nextGesture(0), gestureEnd(0),
    lastPanSign(0), lastTiltSign(0),
    driftPhaseA(0), driftPhaseB(0), lastUpdate(0), firstFrame(true) {
  pan_.cur  = pan_.vel  = pan_.target  = 0.0f;
  tilt_.cur = tilt_.vel = tilt_.target = 0.0f;
}

// Percent of travel -> pulse. The only place the axis description is
// turned into something the servo understands, and the second of the two
// clamps: look() already limits the percent to +/-100, and this refuses to
// emit a pulse outside rest +/- |travel| whatever it is handed, so the
// string limit holds even if a future caller bypasses look().
float Neck::servoUs(const NeckAxis &ax, float pct) const {
  float span = ax.travelDeg * ax.usPerDeg;          // signed us at +100
  float us   = ax.restUs + span * pct / 100.0f;
  float lo   = ax.restUs - fabs(span);
  float hi   = ax.restUs + fabs(span);
  if (us < lo) us = lo;
  if (us > hi) us = hi;
  return us;
}

void Neck::write() {
  // Straight to the pulse width. NOT bank.set(): its 0-180 scale is not
  // servo degrees, and reading it as if it were is what drove both neck
  // servos into their strings on 4 Sep 2026. See neck.h.
  bank.setPulseUs(panAx.channel,  servoUs(panAx,  pan_.cur));
  bank.setPulseUs(tiltAx.channel, servoUs(tiltAx, tilt_.cur));
}

void Neck::begin() {
  lastUpdate = millis();
  firstFrame = true;
  // Both servos go to rest. A servo has no idea where it was left, so this
  // first move is at the servo's own full speed whatever the easing says -
  // expect a twitch at power-up and keep fingers off the strings.
  // bank.begin() switched every channel off first, so the pulse written
  // here is the first thing the servo sees after power-up.
  write();
}

void Neck::look(float pan, float tilt) {
  posePan  = clamp100(pan);
  poseTilt = clamp100(tilt);
  pan_.target  = clamp100(posePan  + gestPan  + driftPan);
  tilt_.target = clamp100(poseTilt + gestTilt + driftTilt);
}

void Neck::hold() {
  gestPan = gestTilt = 0.0f;
  driftPan = driftTilt = 0.0f;
  gestureEnd = 0;
  mode = NECK_STILL;
}

bool Neck::atTarget() const {
  return fabs(pan_.target  - pan_.cur)  < 0.5f && fabs(pan_.vel)  < 2.0f &&
         fabs(tilt_.target - tilt_.cur) < 0.5f && fabs(tilt_.vel) < 2.0f;
}

void Neck::setIdle(NeckIdle m) {
  if (m == mode) return;
  unsigned long now = millis();
  mode = m;
  gestPan = gestTilt = 0.0f;       // any gesture in progress eases out
  gestureEnd = 0;
  switch (mode) {
    case NECK_STILL:
      break;                       // drift is zeroed in update()
    case NECK_TALK:
      nextGesture = now + random(400, 1200);
      break;
    case NECK_REST:
      moveSpeed = idleSpeed;
      nextPose    = now + random(1500, 4000);
      nextGesture = now + random(GEST_REST_GAP_MIN, GEST_REST_GAP_MAX);
      break;
  }
}

// Pick the next resting pose. Mostly upright, sometimes a lean or a slight
// dip, and the further off-centre the head already is, the likelier the
// next move is back to upright - both because that is what people do and
// because holding a lean is the one thing that loads these servos
// continuously.
void Neck::planRestPose(unsigned long now) {
  if ((long)(now - nextPose) < 0) return;

  if (poseAway) {
    // The return. Exactly equal and opposite to the pose it undoes, because
    // it is the same place both axes started from - not a fresh draw that
    // happens to be near centre.
    posePan  = 0.0f;
    poseTilt = 0.0f;
    poseAway = false;
    nextPose = now + random(HOLD_UPRIGHT_MIN, HOLD_UPRIGHT_MAX);
  } else {
    // BOTH AXES MOVE ON EVERY POSE, and both return together. There used to
    // be a one-in-four chance of silencing each axis, which is what made the
    // neck look as though one servo did the work while the other joined in
    // occasionally.
    //
    // Each axis: a direction, then a magnitude from a quarter of that
    // direction's reach up to all of it, squared to bias toward the smaller
    // end. The quarter-of-reach FLOOR matters more than the bias - a pose of
    // two or three degrees disappears into any slack in the string, so
    // without a floor the axis reads as dead rather than as subtle.
    float rp = frand(0.0f, 1.0f);
    float rt = frand(0.0f, 1.0f);
    int8_t panDir  = pickDir(lastPanSign);
    int8_t tiltDir = pickDir(lastTiltSign);

    float panReach  = (panDir  > 0) ? reachPanPos  : reachPanNeg;
    float tiltReach = (tiltDir > 0) ? reachTiltPos : reachTiltNeg;

    float pan  = (float)panDir  * panReach  * (0.25f + 0.75f * rp * rp);
    float tilt = (float)tiltDir * tiltReach * (0.25f + 0.75f * rt * rt);

    if (pan  != 0.0f) lastPanSign  = (pan  > 0.0f) ? 1 : -1;
    if (tilt != 0.0f) lastTiltSign = (tilt > 0.0f) ? 1 : -1;

    posePan  = pan;
    poseTilt = tilt;
    poseAway = true;
    nextPose = now + random(HOLD_POSE_MIN, HOLD_POSE_MAX);
  }
  moveSpeed = idleSpeed * frand(0.55f, 1.0f);   // lazy to brisk, varies
}

void Neck::planTalkGesture(unsigned long now) {
  if (gestureEnd != 0) {
    if ((long)(now - gestureEnd) >= 0) {           // release
      gestPan = gestTilt = 0.0f;
      gestureEnd = 0;
      nextGesture = now + ((mode == NECK_TALK)
                           ? random(GEST_TALK_GAP_MIN, GEST_TALK_GAP_MAX)
                           : random(GEST_REST_GAP_MIN, GEST_REST_GAP_MAX));
    }
    return;
  }
  if ((long)(now - nextGesture) < 0) return;

  bool nod = (mode == NECK_REST) || random(0, 100) < NOD_CHANCE_TALK;
  if (nod) {
    // A nod is forward - that is what a nod IS, and while the head is
    // speaking it is the only kind worth having. Idling is different: a
    // resting head tips back about as often as it dips forward, and making
    // every idle gesture a forward dip pushed ch 4 to one side on top of
    // whatever the pose was already doing. Back gestures are scaled by the
    // same ratio as the reaches, so this follows setIdleReach() rather than
    // needing its own number.
    bool back = (mode == NECK_REST) && (random(0, 100) < TIP_BACK_CHANCE_REST);
    float mag = frand(NOD_MIN, NOD_MAX);
    float fwd = (tiltForwardReach() > 0.01f) ? tiltForwardReach() : 1.0f;
    float upScale = tiltBackReach() / fwd;
    gestTilt = back ? (mag * upScale) : -mag;
    gestPan  = frand(-3.0f, 3.0f);
    gestureEnd = now + random(NOD_HOLD_MIN, NOD_HOLD_MAX);
  } else {
    gestPan  = (random(0, 2) ? 1.0f : -1.0f) * frand(SHIFT_MIN, SHIFT_MAX);
    gestTilt = frand(-5.0f, 3.0f);
    gestureEnd = now + random(SHIFT_HOLD_MIN, SHIFT_HOLD_MAX);
  }

  // A gesture rides on top of the pose, and pose + gesture + drift is
  // clamped at full travel - which IS the string limit. Scale each gesture
  // into whatever headroom the current pose leaves, rather than letting it
  // spend its peak pressed against the clamp: pinned there it is invisible
  // anyway, and it is the tightest the string ever gets. This is what lets
  // the reaches be set generously without the two interacting.
  clampGesture(gestPan,  posePan,  DRIFT_PAN);
  clampGesture(gestTilt, poseTilt, DRIFT_TILT);
}

// Trim one gesture so pose + gesture + drift stays inside +/-100.
void Neck::clampGesture(float &gest, float pose, float drift) {
  float hi =  100.0f - drift - pose;
  float lo = -100.0f + drift - pose;
  if (hi < 0.0f) hi = 0.0f;
  if (lo > 0.0f) lo = 0.0f;
  if (gest > hi) gest = hi;
  if (gest < lo) gest = lo;
}

// One axis, one frame. Accelerate toward the target, cruise at vmax, and
// brake on the curve that arrives with zero speed - so every move eases in
// and out, and a target that changes mid-move is simply followed. Nothing
// here can exceed accel, which is the real protection for the strings.
void Neck::stepAxis(Axis &a, float vmax, float dt) {
  float rem  = a.target - a.cur;
  float dist = fabs(rem);
  float dir  = (rem >= 0.0f) ? 1.0f : -1.0f;

  // Close and slow: land and stop, rather than hunt around the target.
  if (dist < 0.15f && fabs(a.vel) < accel * dt * 2.0f) {
    a.cur = a.target;
    a.vel = 0.0f;
    return;
  }

  // Fastest speed we can still stop from - evaluated for where we will be
  // at the END of this frame, not where we are now. Using the current
  // distance lets the speed lag the braking curve by one frame every frame,
  // and the move ends by hitting the target at 50 %/s and stopping dead.
  float vStop = -accel * dt + sqrt(accel * accel * dt * dt + 2.0f * accel * dist);
  float vCap  = (vStop < vmax) ? vStop : vmax;
  float vWant = dir * vCap;

  float dv    = vWant - a.vel;
  float maxDv = accel * dt;
  if (dv >  maxDv) dv =  maxDv;
  if (dv < -maxDv) dv = -maxDv;
  a.vel += dv;

  // No "snap to target on crossing". Against a fixed target the braking
  // curve above arrives with no overshoot by itself; the only way to cross
  // is a target that jumped CLOSER mid-move (a gesture releasing early), and
  // a head that is already moving overshoots by a fraction and comes back
  // rather than stopping dead - which is what a real one does too.
  a.cur = clamp100(a.cur + a.vel * dt);
}

void Neck::update() {
  unsigned long now = millis();
  unsigned long dt  = now - lastUpdate;      // rollover-safe
  lastUpdate = now;
  if (firstFrame) { dt = 0; firstFrame = false; }
  if (dt > 50UL) dt = 50UL;                  // something blocked: take one modest
                                             // step, not one that covers the gap
  float dts = (float)dt / 1000.0f;

  // Breathing drift, or none while listening.
  if (mode == NECK_STILL) {
    driftPan = driftTilt = 0.0f;
  } else {
    driftPhaseA += (float)dt * DRIFT_RATE_A;
    driftPhaseB += (float)dt * DRIFT_RATE_B;
    while (driftPhaseA >= TWO_PI) driftPhaseA -= TWO_PI;
    while (driftPhaseB >= TWO_PI) driftPhaseB -= TWO_PI;
    float s = (mode == NECK_TALK) ? DRIFT_TALK_SCALE : 1.0f;
    driftPan  = sin(driftPhaseA) * DRIFT_PAN  * s;
    driftTilt = sin(driftPhaseB) * DRIFT_TILT * s;
  }

  if (mode == NECK_REST)  planRestPose(now);
  if (mode != NECK_STILL) planTalkGesture(now);

  pan_.target  = clamp100(posePan  + gestPan  + driftPan);
  tilt_.target = clamp100(poseTilt + gestTilt + driftTilt);

  // Idle poses go at their own unhurried pace; anything the Pi asks for,
  // and any gesture, goes at driven speed.
  float vmax = (mode == NECK_REST && gestureEnd == 0) ? moveSpeed : drivenSpeed;

  if (dt > 0) {
    stepAxis(pan_,  vmax, dts);
    stepAxis(tilt_, vmax, dts);
  }
  curPan  = pan_.cur;
  curTilt = tilt_.cur;
  write();
}

// Slow, deliberate, one axis at a time, at HALF travel so a wrong sign or
// a wrong usPerDeg cannot slam anything into a string limit. Names each
// direction BEFORE moving, so you can stand where you like and simply
// watch which way the head goes. Prints the horn degrees and pulse it is
// asking for, so a protractor on the horn tells you whether usPerDeg is
// right: at half travel the horn should turn half of |travelDeg|.
void Neck::demo() {
  struct { float pan, tilt; const char *name; } poses[] = {
    { 0,  0, "centre" }, {  50,  0, "RIGHT (lean)" }, { 0, 0, "centre" },
    {-50,  0, "LEFT (lean)"  }, {  0,  50, "BACK"   }, { 0, 0, "centre" },
    { 0, -50, "FORWARD" },      {  0,   0, "centre" }
  };

  NeckIdle was = mode;
  hold();                               // no drift or gestures during the check
  Serial.println(F("neck: if RIGHT goes left, negate PAN_TRAVEL_DEG."));
  Serial.println(F("      if FORWARD goes back, negate TILT_TRAVEL_DEG."));
  Serial.println(F("      if the horn turns more or less than printed, fix NECK_US_PER_DEG."));

  for (unsigned p = 0; p < sizeof(poses) / sizeof(poses[0]); p++) {
    Serial.print(F("  "));
    Serial.print(poses[p].name);
    Serial.print(F("   pan "));
    Serial.print(panAx.travelDeg * poses[p].pan / 100.0f, 0);
    Serial.print(F(" deg ("));
    Serial.print(servoUs(panAx, poses[p].pan), 0);
    Serial.print(F(" us)  tilt "));
    Serial.print(tiltAx.travelDeg * poses[p].tilt / 100.0f, 0);
    Serial.print(F(" deg ("));
    Serial.print(servoUs(tiltAx, poses[p].tilt), 0);
    Serial.println(F(" us)"));
    look(poses[p].pan, poses[p].tilt);
    unsigned long giveUp = millis() + 4000;
    do {
      update();
      delay(20);
    } while (!atTarget() && (long)(millis() - giveUp) < 0);
    delay(900);
  }
  lastUpdate = millis();
  setIdle(was);
}