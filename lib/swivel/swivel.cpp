#include "Swivel.h"

// How far past the software limit the homing sweep is allowed to look.
// The centre can only be within maxDeg of wherever the head booted if it
// was inside its limits when power was lost, so this covers a nudge.
static const float HOME_MARGIN_DEG = 10.0f;

// Idle glances are PAIRED. *** 4 Sep 2026 ***
//
// Every excursion is an absolute target away from centre; the glance after
// it is always the return to exactly 0; the one after that goes the other
// way. The commanded arc is therefore equal clockwise and anticlockwise by
// construction - not on average, exactly - and the estimate is back at zero
// between every pair.
//
// This replaced a probabilistic return, a random-walk "small shift" branch,
// and two timers. The measurement that killed them: over ten simulated
// minutes that planner already commanded 1109 degrees each way - a 0.0
// percent imbalance - and still left the head physically 166 degrees round.
// The bias was never in the pattern.
//
// SO WHAT PAIRING IS REALLY FOR: it makes the actual fault visible. The two
// directions of a continuous servo run at slightly different speeds, so an
// out-and-back leaves a residual, and in an unpaired wander that residual is
// buried in a walk nobody can eyeball. Paired, the head returns to the SAME
// place every time, so whatever it has slipped by after the return is the
// direction imbalance on its own - once per cycle, in plain sight. Watch a
// few cycles and trim SWIVEL_NEG_SCALE in main.cpp.
static const int GLANCE_MIN_PCT = 45;    // of maxDeg
static const int GLANCE_MAX_PCT = 100;
static const unsigned long LOOK_HOLD_MIN = 2000, LOOK_HOLD_MAX = 5000;
static const unsigned long HOME_REST_MIN = 5000, HOME_REST_MAX = 14000;

static const unsigned long CAL_SLOW_MS = 2000;
static const unsigned long CAL_FAST_MS = 500;

Swivel::Swivel(JointBank &b, const SwivelConfig &c)
  : bank(b), cfg(c),
    posDeg(0.0f), vPhys(0.0f), vCmd(0.0f), pulseOn(false),
    mode(POSITION), targetDeg(0.0f), intentDeg(0.0f),
    rateTarget(0.0f), rateUntil(0), settleDeg(1.0f),
    idleSpeed(35.0f), drivenSpeed(70.0f), accel(110.0f), eyeLeadMs(180),
    lookHoldMin(LOOK_HOLD_MIN), lookHoldMax(LOOK_HOLD_MAX),
    homeRestMin(HOME_REST_MIN), homeRestMax(HOME_REST_MAX),
    awayFromHome(false), lastLookSide(0),
    wanderOn(false), glanceFlag(false), glancePending(false),
    glanceAt(0), nextGlance(0),
    homed(false), sensorWas(false), lastUpdate(0) {}

void Swivel::begin() {
  if (cfg.homePin >= 0) pinMode(cfg.homePin, INPUT_PULLUP);

  // The last leg of every move runs at the floor speed, so "close enough"
  // has to be at least one frame's travel at that speed or the head will
  // step over the target and come back.
  settleDeg = cfg.slowDegPerS * 0.045f;
  if (settleDeg < 1.0f) settleDeg = 1.0f;

  // Wherever the head is right now is centre until something says
  // otherwise. This is the ONLY assumption the estimate ever rests on,
  // which is why the head should be straight before power is applied.
  posDeg = 0.0f;
  stopNow();
  homed = false;
  sensorWas = (cfg.homePin >= 0) ? sensorActive() : false;
  lastUpdate = millis();
  nextGlance = millis() + 5000;
  awayFromHome = false;          // begin() has just declared here to be centre
}

// ---------------------------------------------------------------- commands

void Swivel::setYawDeg(float deg) {
  if (deg >  cfg.maxDeg) deg =  cfg.maxDeg;
  if (deg < -cfg.maxDeg) deg = -cfg.maxDeg;
  targetDeg = intentDeg = deg;
  mode = POSITION;
  glancePending = false;
}

void Swivel::setYaw(float percent) {
  if (percent < -100.0f) percent = -100.0f;
  if (percent >  100.0f) percent =  100.0f;
  setYawDeg(percent / 100.0f * cfg.maxDeg);
}

float Swivel::yawPercent() const {
  return posDeg / cfg.maxDeg * 100.0f;
}

void Swivel::setRate(float percent) {
  if (percent < -100.0f) percent = -100.0f;
  if (percent >  100.0f) percent =  100.0f;
  rateTarget = percent / 100.0f * drivenSpeed;
  rateUntil  = millis() + RATE_HOLD_MS;
  mode = RATE;
  glancePending = false;
  intentDeg = posDeg;
}

// The natural stopping point at the current acceleration limit, so the
// head eases out rather than reversing to hold an exact spot. A head that
// is already resting on a position target is left exactly as it is.
void Swivel::hold() {
  rateTarget = 0.0f;
  glancePending = false;
  if (mode == POSITION && !pulseOn) return;
  float stopDist = (vCmd * vCmd) / (2.0f * accel);
  setYawDeg(posDeg + sgn(vCmd) * stopDist);
}

void Swivel::stopNow() {
  bank.setPulseUs(cfg.channel, 0.0f);
  pulseOn = false;
  vPhys = vCmd = 0.0f;
  targetDeg = intentDeg = posDeg;
  rateTarget = 0.0f;
  mode = POSITION;
  glancePending = false;
}

void Swivel::zeroHere() {
  awayFromHome = false;   // here is centre now, so nothing is owed
  stopNow();
  posDeg = 0.0f;
  targetDeg = intentDeg = 0.0f;
  homed = true;
}

bool Swivel::atTarget() const {
  float e = intentDeg - posDeg;
  return e > -settleDeg && e < settleDeg;
}

void Swivel::setWander(bool on) {
  wanderOn = on;
  if (on) {
    nextGlance = millis() + (unsigned long)random(2000, 5000);
    // The Pi may have left the head anywhere. Owe a return immediately
    // rather than starting a fresh excursion from wherever it was
    // abandoned: the return target is absolute 0, so this brings the head
    // home first whatever the Pi did with it.
    awayFromHome = true;
  } else {
    glancePending = false;
  }
}


bool Swivel::glanceStarted() {
  bool f = glanceFlag;
  glanceFlag = false;
  return f;
}

// ---------------------------------------------------------------- servo

// Turn a wanted head speed into a pulse. The servo cannot go slower than
// slowDegPerS, so anything below that is either the floor or off, and
// pulseOn / vPhys always describe what the servo is REALLY being asked to
// do, because that is what the dead-reckoning has to integrate.
// Speeds for the direction being asked for. A zero in the Neg pair means
// "not measured", so it falls back to the + numbers and behaves exactly
// as it did before these fields existed.
float Swivel::slowFor(float degPerS) const {
  if (degPerS < 0.0f && cfg.slowDegPerSNeg > 0.01f) return cfg.slowDegPerSNeg;
  return cfg.slowDegPerS;
}
float Swivel::fastFor(float degPerS) const {
  if (degPerS < 0.0f && cfg.fastDegPerSNeg > 0.01f) return cfg.fastDegPerSNeg;
  return cfg.fastDegPerS;
}

// How hard to push, per direction. This is the one that fixes a head that
// physically travels further one way than the other; the speed pair above
// only fixes what the estimate BELIEVES it did.
float Swivel::slowUsFor(float degPerS) const {
  if (degPerS < 0.0f && cfg.slowUsNeg > 0.01f) return cfg.slowUsNeg;
  return cfg.slowUs;
}
float Swivel::fastUsFor(float degPerS) const {
  if (degPerS < 0.0f && cfg.fastUsNeg > 0.01f) return cfg.fastUsNeg;
  return cfg.fastUs;
}

void Swivel::drive(float degPerS) {
  float mag  = fabs(degPerS);
  float slow = slowFor(degPerS);
  float fast = fastFor(degPerS);

  if (mag < 0.5f * slow) {
    if (pulseOn) bank.setPulseUs(cfg.channel, 0.0f);
    pulseOn = false;
    vPhys = 0.0f;
    return;
  }

  if (mag < slow) mag = slow;
  if (mag > fast) mag = fast;

  float span = fast - slow;
  float f    = (span > 0.1f) ? (mag - slow) / span : 1.0f;
  float slowOff = slowUsFor(degPerS);
  float fastOff = fastUsFor(degPerS);
  float us   = cfg.stopUs + sgn(degPerS) * (float)cfg.dir
             * (slowOff + f * (fastOff - slowOff));

  bank.setPulseUs(cfg.channel, us);
  pulseOn = true;
  vPhys = sgn(degPerS) * mag;
}

bool Swivel::sensorActive() const {
  // Switch or open-collector Hall sensor to ground, internal pull-up.
  return digitalRead(cfg.homePin) == LOW;
}

void Swivel::pollHome(unsigned long now) {
  (void)now;
  if (cfg.homePin < 0 || cfg.homeKind == HOME_NONE) return;
  bool active = sensorActive();

  if (cfg.homeKind == HOME_SIDE) {
    // Any change of state means we are on the edge, which is centre.
    if (active != sensorWas) {
      posDeg = 0.0f;
      homed = true;
    }
  } else if (active && !sensorWas) {
    // Just entered the window, from the side we were travelling from, so
    // we are at its near edge: half a width short of centre.
    float side = (vPhys > 0.0f) ? -1.0f : (vPhys < 0.0f ? 1.0f : 0.0f);
    posDeg = side * cfg.homeHalfWidthDeg;
    homed = true;
  }
  sensorWas = active;
}

// ---------------------------------------------------------------- idle

// What a resting head actually does: mostly nothing, with an occasional
// glance. Small shifts are common, a proper look is rarer, and there is a
// standing bias back toward straight ahead that gets stronger the further
// out the head is, so it never ends up parked sideways for a minute.
void Swivel::chooseGlance(unsigned long now) {
  float m = cfg.maxDeg;
  float t;

  if (awayFromHome) {
    // The matching return. Absolute zero, so it undoes the excursion exactly
    // however big that was - and if anything moved the head while it was
    // out, this still lands on centre rather than on a remembered offset.
    t = 0.0f;
    awayFromHome = false;
  } else {
    // A fresh excursion, the opposite way to the last one. No coin: the only
    // randomness is how far and how long, never which way, so the two
    // directions come out exactly equal in count.
    int8_t dir = (lastLookSide != 0) ? (int8_t)(-lastLookSide)
                                     : (int8_t)(random(0, 2) ? 1 : -1);
    float mag = (float)random(GLANCE_MIN_PCT, GLANCE_MAX_PCT + 1) / 100.0f * m;
    t = (float)dir * mag;
    lastLookSide = dir;
    awayFromHome = true;
  }

  if (t >  m) t =  m;
  if (t < -m) t = -m;

  if (fabs(t - posDeg) < 3.0f) {          // already there; keep the state
    nextGlance = now + (unsigned long)random(1500, 4000);
    return;
  }

  intentDeg     = t;
  glancePending = true;
  glanceAt      = now + eyeLeadMs + (unsigned long)random(0, 80);
  glanceFlag    = true;

  // The pause: a hold at the far end before coming back, then a longer rest
  // at centre before going the other way. Without the first one the head
  // swings straight through and the pairing is invisible.
  nextGlance = now + (unsigned long)((t == 0.0f)
                     ? random(homeRestMin, homeRestMax)
                     : random(lookHoldMin, lookHoldMax));
}

// ---------------------------------------------------------------- frame

void Swivel::update() {
  unsigned long now = millis();
  float dt = (float)(now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  if (dt > 1.0f) dt = 1.0f;

  // 1. Dead-reckon. The pulse that has been on since the last frame is what
  //    moved the head, so integrate it BEFORE planning the next one, and use
  //    the real dt: if loop() stalled for half a second the servo ran for
  //    half a second, and pretending otherwise is how the estimate drifts.
  posDeg += vPhys * dt;

  // 2. Home sensor may correct that estimate.
  pollHome(now);

  // 3. Idle glances. The glance is chosen first and the move starts a beat
  //    later, so the eyes can get there ahead of the head.
  if (wanderOn && !glancePending && (long)(now - nextGlance) >= 0) chooseGlance(now);
  if (glancePending && (long)(now - glanceAt) >= 0) {
    glancePending = false;
    targetDeg = intentDeg;
    mode = POSITION;
  }

  // 4. A rate nobody has refreshed is a rate nobody wants.
  if (mode == RATE && (long)(now - rateUntil) >= 0) rateTarget = 0.0f;

  // 5. Plan the speed for the coming frame.
  float dtPlan = (dt > 0.1f) ? 0.1f : dt;
  float cap = wanderOn ? idleSpeed : drivenSpeed;
  // Floor on the SLOWER of the two directions, so a cap that is fine one
  // way cannot fall under the other way's minimum and refuse to move.
  float slowest = cfg.slowDegPerS;
  if (cfg.slowDegPerSNeg > 0.01f && cfg.slowDegPerSNeg > slowest) slowest = cfg.slowDegPerSNeg;
  if (cap < slowest) cap = slowest;

  float want = 0.0f;
  bool arrived = false;

  if (mode == POSITION) {
    float d  = targetDeg - posDeg;
    float ad = fabs(d);
    if (ad <= settleDeg) {
      arrived = true;
    } else {
      // Fastest we can go and still stop at the target decelerating at
      // 'accel'. Far out this exceeds the cap, so we cruise; close in it
      // falls below, so we brake on the same curve every time.
      float brake = sqrt(2.0f * accel * ad);
      want = sgn(d) * ((brake < cap) ? brake : cap);
    }
  } else {
    want = rateTarget;
    if (want != 0.0f) {
      // Same braking curve, aimed at the software limit instead.
      float room = (want > 0.0f) ? (cfg.maxDeg - posDeg) : (posDeg + cfg.maxDeg);
      if (room <= settleDeg) {
        want = 0.0f;
      } else {
        float brake = sqrt(2.0f * accel * room);
        if (fabs(want) > brake) want = sgn(want) * brake;
      }
    }
  }

  if (arrived) {
    // The final leg is at floor speed and a head this size has no momentum
    // to speak of at that pace: cut the pulse rather than ramp it.
    vCmd = 0.0f;
  } else {
    float dv = want - vCmd;
    float mx = accel * dtPlan;
    if (dv >  mx) dv =  mx;
    if (dv < -mx) dv = -mx;
    vCmd += dv;
  }

  // 6. Out to the servo.
  drive(vCmd);
}

// ---------------------------------------------------------------- blocking

void Swivel::runFor(unsigned long ms) {
  unsigned long until = millis() + ms;
  while ((long)(millis() - until) < 0) { update(); delay(5); }
}

// One slow sweep, watching the sensor. SIDE: stops when the reading
// changes from startState. WINDOW: stops when the reading goes active.
bool Swivel::homeSweep(int8_t direction, float maxDistanceDeg, bool startState) {
  unsigned long limitMs = (unsigned long)(maxDistanceDeg / cfg.slowDegPerS * 1000.0f);
  unsigned long start = millis(), last = start;
  bool found = false;

  drive((float)direction * cfg.slowDegPerS);
  while (millis() - start < limitMs) {
    delay(2);
    unsigned long now = millis();
    posDeg += vPhys * (float)(now - last) / 1000.0f;
    last = now;
    bool active = sensorActive();
    if (cfg.homeKind == HOME_SIDE ? (active != startState) : active) {
      posDeg = (cfg.homeKind == HOME_SIDE) ? 0.0f
                                           : -(float)direction * cfg.homeHalfWidthDeg;
      homed = true;
      sensorWas = active;
      found = true;
      break;
    }
  }
  drive(0.0f);
  delay(150);
  return found;
}

// Blocking. SIDE sensor: one read says which side of centre we are on,
// so the head turns the right way and stops on the edge. WINDOW sensor:
// no such luck - look left up to the limit, then right across the whole
// range. If the head was inside its limits at power-off the sensor is in
// that sweep somewhere, but on the wrong side the search can carry the
// head past the limit first, which is why SIDE is the one to build.
// Not found: the head returns to where it booted and that is treated as
// centre, exactly as if there were no sensor.
bool Swivel::homeNow() {
  if (cfg.homePin < 0 || cfg.homeKind == HOME_NONE) return false;
  stopNow();

  bool found;
  float reach = cfg.maxDeg + HOME_MARGIN_DEG;
  bool active = sensorActive();

  if (cfg.homeKind == HOME_SIDE) {
    // LOW on the lowSide half: if it reads low we are on that side, so go
    // the other way; if not, go toward it.
    int8_t side = active ? cfg.homeLowSide : (int8_t)-cfg.homeLowSide;
    found = homeSweep((int8_t)-side, reach, active);
  } else if (active) {
    posDeg = 0.0f;
    homed = found = true;
    sensorWas = true;
  } else {
    found = homeSweep(-1, reach, false) || homeSweep(+1, 2.0f * reach, false);
  }

  stopNow();
  targetDeg = intentDeg = 0.0f;     // found: on to true centre. not: back to boot spot
  mode = POSITION;
  lastUpdate = millis();
  return found;
}

// Blocking direction check. Watch the HEAD: a positive yaw must turn it
// to ITS right. If it goes left, flip SWIVEL_DIR - nothing else.
void Swivel::demo() {
  bool w = wanderOn;
  wanderOn = false;

  Serial.println(F("centre"));       setYaw(0);   runFor(1500);
  Serial.println(F("right 40%"));    setYaw(40);  runFor(2500);
  Serial.println(F("centre"));       setYaw(0);   runFor(2500);
  Serial.println(F("left 40%"));     setYaw(-40); runFor(2500);
  Serial.println(F("centre"));       setYaw(0);   runFor(2500);

  wanderOn = w;
  lastUpdate = millis();
}

// Blocking. Timed spins at the slow and fast pulse offsets so the two
// speeds in the config can be MEASURED instead of guessed. Everything the
// dead-reckoning does rests on these two numbers.
void Swivel::calibrate() {
  wanderOn = false;
  stopNow();

  Serial.println(F("--- swivel calibration ---"));
  Serial.println(F("put a pencil mark across the ring gear and the plate."));
  Serial.println(F("each spin is timed; degrees moved / seconds = deg/s."));
  Serial.println(F("do this BEFORE the harness is threaded through, or with slack."));
  Serial.println();
  Serial.println(F("1. stop pulse, 3 s. if the head creeps, SWIVEL_STOP_US is off."));
  Serial.println(F("   NOT creeping only means the pulse is somewhere INSIDE"));
  Serial.println(F("   the deadband, not that it is centred in it. Step 1b."));
  delay(1500);
  bank.setPulseUs(cfg.channel, cfg.stopUs);
  delay(3000);
  bank.setPulseUs(cfg.channel, 0.0f);
  delay(1500);

  // 1b. Both edges of the deadband. The midpoint is the TRUE stop, and
  // how far it sits from stopUs is exactly what makes one direction
  // stronger than the other.
  Serial.println(F("1b. deadband edges. watch the pinion; note the FIRST"));
  Serial.println(F("    printed pulse at which it starts to creep, each way."));
  for (int8_t direction = 1; direction >= -1; direction -= 2) {
    Serial.println(direction > 0 ? F("   -- going + --") : F("   -- going - --"));
    delay(1500);
    for (int off = 0; off <= 90; off += 5) {
      float us = cfg.stopUs + (float)direction * (float)cfg.dir * (float)off;
      Serial.print(F("    "));
      Serial.println(us, 0);
      bank.setPulseUs(cfg.channel, us);
      delay(700);
    }
    bank.setPulseUs(cfg.channel, 0.0f);
    delay(2000);
  }
  Serial.println(F("    true stop = midpoint of the two. SWIVEL_NEG_US_BOOST"));
  Serial.println(F("    = twice the gap between it and SWIVEL_STOP_US."));
  delay(1500);

  for (int i = 0; i < 2; i++) {
    float offset = (i == 0) ? cfg.slowUs : cfg.fastUs;
    unsigned long ms = (i == 0) ? CAL_SLOW_MS : CAL_FAST_MS;
    for (int8_t direction = 1; direction >= -1; direction -= 2) {
      Serial.print(i == 0 ? F("2. SLOW ") : F("3. FAST "));
      Serial.print(direction > 0 ? F("+ (should be RIGHT) ") : F("- (should be LEFT) "));
      Serial.print(offset, 0);
      Serial.print(F(" us for "));
      Serial.print(ms);
      Serial.println(F(" ms"));
      delay(2000);
      bank.setPulseUs(cfg.channel, cfg.stopUs + (float)direction * (float)cfg.dir * offset);
      delay(ms);
      bank.setPulseUs(cfg.channel, 0.0f);
      Serial.println(F("   stopped - note degrees moved"));
      delay(3000);
    }
  }

  Serial.println(F("--- done ---"));
  Serial.println(F("RECORD ALL FOUR - the two directions are not the same:"));
  Serial.println(F("  slow + -> SWIVEL_SLOW_DEG_S      fast + -> SWIVEL_FAST_DEG_S"));
  Serial.println(F("  slow - -> SWIVEL_SLOW_DEG_S_NEG  fast - -> SWIVEL_FAST_DEG_S_NEG"));
  Serial.println(F("a direction that turns SHORT is one whose speed is set too high."));
  Serial.println(F("if + went LEFT, flip SWIVEL_DIR. then set SWIVEL_CAL_MODE false."));
  lastUpdate = millis();
}