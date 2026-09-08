#include "Eyes.h"

// ---- Gaze dynamics ----
// Saccadic, not smooth: a gaze change should complete in roughly 100-200 ms.
// EASE gives the snap-then-settle shape, RATE caps the peak speed.
static const float GAZE_EASE = 18.0f;    // per second
static const float GAZE_RATE = 600.0f;   // percent per second

// ---- Blink dynamics ----
// Down fast, pause, up slower. Total ~300 ms, which is a real blink.
static const unsigned int CLOSE_MS = 80;
static const unsigned int HOLD_MS  = 70;
static const unsigned int OPEN_MS  = 150;

// ---- Lid expression slew ----
static const float LID_BASE_RATE = 250.0f;  // percent per second

// Lids can track vertical gaze slightly - looking down droops them. It is a
// nice cue, but it means the lid servo sits a few percent off rest whenever
// the eyes are not level, and rest being EXACTLY rest matters more here.
// Set it to something like 0.15 once the lid rest angle is confirmed good.
static const float LID_TRACK = 0.0f;

Eyes::Eyes(JointBank &b, EyeBank l)
  : bank(b), left(l), right(l), hasRight(false),
    xNow(0), yNow(0), xTarget(0), yTarget(0),
    lidNow(0), lidBase(0),
    phase(IDLE), phaseStart(0),
    wanderOn(false), autoBlink(true),
    nextSaccade(0), nextBlink(0), lastUpdate(0) {}

Eyes::Eyes(JointBank &b, EyeBank l, EyeBank r)
  : bank(b), left(l), right(r), hasRight(true),
    xNow(0), yNow(0), xTarget(0), yTarget(0),
    lidNow(0), lidBase(0),
    phase(IDLE), phaseStart(0),
    wanderOn(false), autoBlink(true),
    nextSaccade(0), nextBlink(0), lastUpdate(0) {}

void Eyes::begin() {
  lastUpdate  = millis();
  nextSaccade = millis() + 2000;
  nextBlink   = millis() + 3000;
  // Rest is the assembled position: this is the first and only thing the
  // servos are told at boot, so nothing jumps anywhere unexpected.
  rest();
}

void Eyes::rest() {
  xNow = yNow = xTarget = yTarget = 0.0f;
  lidBase = 0.0f;
  phase = IDLE;
  writeGaze();
  writeLid(0.0f);
}

void Eyes::blink() {
  phase = CLOSING;
  phaseStart = millis();
}

void Eyes::setLids(float percent) {
  lidBase = constrain(percent, 0.0f, 100.0f);
}

void Eyes::look(float x, float y) {
  xTarget = constrain(x, -100.0f, 100.0f);
  yTarget = constrain(y, -100.0f, 100.0f);
}

// Rest plus a signed fraction of travel, per side. At x = y = 0 every servo
// is commanded to exactly its own rest angle - no offset, no drift. The two
// sides get the SAME logical percentage; their travel signs and rest angles
// turn that into mirrored physical motion.
void Eyes::writeGaze() {
  bank.set(left.pan,  (float)left.panRest  + (xNow / 100.0f) * (float)left.panTravel);
  bank.set(left.tilt, (float)left.tiltRest + (yNow / 100.0f) * (float)left.tiltTravel);
  if (hasRight) {
    bank.set(right.pan,  (float)right.panRest  + (xNow / 100.0f) * (float)right.panTravel);
    bank.set(right.tilt, (float)right.tiltRest + (yNow / 100.0f) * (float)right.tiltTravel);
  }
}

// pct: 0 = rest (open) .. 100 = fully shut. lidTravel carries the sign, so
// which way the lids close is a calibration value rather than a code path.
void Eyes::writeLid(float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  bank.set(left.lid, (float)left.lidRest + (pct / 100.0f) * (float)left.lidTravel);
  if (hasRight) {
    bank.set(right.lid, (float)right.lidRest + (pct / 100.0f) * (float)right.lidTravel);
  }
  lidNow = pct;
}

// Where the lid is trying to be right now, blink aside: the expression
// base, nudged by vertical gaze. Clamped at 0 so it can never be driven
// past rest in the open direction.
float Eyes::lidTargetNow() const {
  return constrain(lidBase - yNow * LID_TRACK, 0.0f, 100.0f);
}

void Eyes::update() {
  unsigned long now = millis();
  float dt = (float)(now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  if (dt > 0.2f) dt = 0.2f;

  // ---- idle behaviours ----
  if (wanderOn && (long)(now - nextSaccade) >= 0) {
    long r = random(0, 100);
    if (r < 55) {
      // small shift near where we already are - reading, thinking
      look(xNow + random(-25, 26), yNow + random(-18, 19));
    } else if (r < 85) {
      // proper glance somewhere else
      look(random(-70, 71), random(-45, 46));
    } else {
      // back to roughly ahead
      look(random(-12, 13), random(-10, 11));
    }
    nextSaccade = now + (unsigned long)random(700, 3500);
  }

  if (autoBlink && phase == IDLE && (long)(now - nextBlink) >= 0) {
    blink();
    // Occasional double blink: schedule the next one almost immediately.
    nextBlink = now + (random(0, 100) < 15 ? 400UL
                                           : (unsigned long)random(2500, 7000));
  }

  // ---- gaze easing: snap, then settle ----
  float ex = xTarget - xNow;
  float ey = yTarget - yNow;
  if (ex > 0.4f || ex < -0.4f || ey > 0.4f || ey < -0.4f) {
    float maxStep = GAZE_RATE * dt;
    float sx = constrain(ex * GAZE_EASE * dt, -maxStep, maxStep);
    float sy = constrain(ey * GAZE_EASE * dt, -maxStep, maxStep);
    xNow += sx;
    yNow += sy;
    writeGaze();
  }

  // ---- lids ----
  switch (phase) {
    case CLOSING: {
      // Compute the position BEFORE deciding whether to advance the phase.
      // Doing it the other way round resets phaseStart first, which makes
      // the fraction zero and snaps the lid back open for a single frame -
      // a visible flicker in the middle of every blink.
      float t = (float)(now - phaseStart) / (float)CLOSE_MS;
      if (t >= 1.0f) {
        writeLid(100.0f);
        phase = CLOSED;
        phaseStart = now;
      } else {
        writeLid(lidTargetNow() + (100.0f - lidTargetNow()) * t);
      }
      break;
    }

    case CLOSED:
      writeLid(100.0f);
      if (now - phaseStart >= HOLD_MS) { phase = OPENING; phaseStart = now; }
      break;

    case OPENING: {
      float t = (float)(now - phaseStart) / (float)OPEN_MS;
      if (t >= 1.0f) { phase = IDLE; writeLid(lidTargetNow()); }
      else writeLid(100.0f + (lidTargetNow() - 100.0f) * t);
      break;
    }

    case IDLE: {
      // Drift back toward the expression base at a civilised rate. With no
      // expression set that base is 0, so the lids always settle at rest.
      float target = lidTargetNow();
      float e = target - lidNow;
      if (e > 0.5f || e < -0.5f) {
        float step = constrain(e, -LID_BASE_RATE * dt, LID_BASE_RATE * dt);
        writeLid(lidNow + step);
      }
      break;
    }
  }
}

// Blocking, named, one axis at a time, and back to rest between each.
// If an axis moves the wrong way, flip the SIGN of its travel in main.cpp.
void Eyes::demo() {
  struct { float x, y; const char *name; } gaze[] = {
    {   0,   0, "rest"  },
    { -80,   0, "left"  }, { 0, 0, "rest" },
    {  80,   0, "right" }, { 0, 0, "rest" },
    {   0,  70, "up"    }, { 0, 0, "rest" },
    {   0, -70, "down"  }, { 0, 0, "rest" },
  };

  for (unsigned i = 0; i < sizeof(gaze) / sizeof(gaze[0]); i++) {
    Serial.println(gaze[i].name);
    look(gaze[i].x, gaze[i].y);
    unsigned long until = millis() + 900;
    while (millis() < until) { update(); delay(5); }
  }

  Serial.println(F("blink x2"));
  for (int b = 0; b < 2; b++) {
    blink();
    unsigned long until = millis() + 700;
    while (millis() < until) { update(); delay(5); }
  }

  Serial.println(F("lids half, then rest"));
  setLids(55);
  unsigned long until = millis() + 1200;
  while (millis() < until) { update(); delay(5); }
  setLids(0);
  until = millis() + 800;
  while (millis() < until) { update(); delay(5); }

  rest();
  lastUpdate = millis();
}