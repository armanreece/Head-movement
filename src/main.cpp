#include <Arduino.h>
#include <Wire.h>
#include "Joints.h"
#include "Jaw.h"
#include "Neck.h"
#include "Swivel.h"
#include "Eyes.h"
#include "SerialCommands.h"

JointBank bank(0x40);
SerialCommands comms;

// ===================== CALIBRATION =====================
// Everything that describes the physical machine lives here.
// Nothing below this block should need editing to retune the head.

// ---- Jaw ----
// { channel, limitA, limitB, reversed, trim }
const int JAW_SHUT = 105;   // mouth closed
const int JAW_GAPE = 90;    // mouth fully open

Joint jawLeft  = { 0, JAW_SHUT, JAW_GAPE, false, 0 };
Joint jawRight = { 1, JAW_SHUT, JAW_GAPE, true,  0 };
Jaw jaw(bank, jawLeft, jawRight, JAW_SHUT, JAW_GAPE);

// ---- Neck ----  *** REBUILT 3 Sep 2026: two servos, string loops ***
//
// One standard servo per axis, each turning a pulley with a string loop
// over it. The head sits on a two-axis pivot: turning a pulley pulls one
// side of the platform down and pays the other side out, so one servo does
// both directions of its axis. Nothing to keep taut, nothing to keep slack,
// and there is one rest position per axis. See neck.h.
//
//   ch 4  tilt   anticlockwise = head FORWARD, clockwise = head BACK
//   ch 5  pan    anticlockwise = head RIGHT,   clockwise = head LEFT
//                (a sideways LEAN - turning to look is the swivel's job now)
//
// THE NUMBERS HERE ARE REAL UNITS: a pulse width in MICROSECONDS for
// "upright", and HORN DEGREES for travel. They are NOT on the 0-180 scale
// the eyes and jaw use. *** Fixed 4 Sep 2026: *** the neck used to be given
// rest 90 / travel 45 on that scale, and that scale is not servo degrees -
// its "90" is a 1830 us pulse, 30-60 degrees past a standard servo's centre,
// and its "45" is 50-100 real degrees. Both servos were driven hard into
// their strings at power-up: tilt went fully back and stalled, and the
// jammed platform hid whatever pan was doing. A stalled 20 kg servo pulls
// 2-3 A and cooks itself, so if that ever happens again, POWER OFF FIRST
// and ask questions afterwards.
//
// NECK_US_PER_DEG is the servos' own scale. Most 20 kg servos (DS3218 and
// its clones) are 500-2500 us over 180 degrees = 11.1 us/deg. A 270-degree
// variant is 7.4; a 1000-2000 us/180 unit is 5.6. Check the label. If it is
// wrong the head still centres correctly, but a "45 degree" command turns
// the horn some other amount: with RUN_DEMO = true the horn should turn the
// number of degrees it prints - measure one move with a protractor and
// scale this constant by actual/printed.
//
// REST is the pulse at which the head is upright with both sides of that
// axis's string evenly taut. 1500 us is every servo's nominal centre, but
// only a MEASUREMENT makes it true of your servo: the horn's angle at a
// given pulse depends on how it was pushed onto the spline, so a horn
// seated while something else was driving the servo leaves the head
// permanently leaning at "rest". NECK_CAL_CH in the Behaviour block below
// finds the real number - use it before trusting either rest value here.
// The mechanical version of the same fix: power up with the loop off, let
// the horn settle at 1500, then fit the loop with the head upright. Fine
// trim after that is numerical: type N values in the serial
// monitor until the head is upright (say N-8,12) and fold them in as
//   rest_us += travel_deg * NECK_US_PER_DEG * value / 100
//   e.g. tilt: 1500 + 45 * 11.1 * 0.12 = 1560    pan: 1500 + (-45)(11.1)(-0.08) = 1540
// The PCA9685's oscillator is only nominal, so "1500" may not be exactly
// 1500 us at the servo - that is fine, the trim absorbs it.
//
// TRAVEL is the horn angle at a full-scale (+100) command. 45 is the STRING
// LIMIT: past it the loop winds bar-tight and the servo stalls against it.
// Do not raise it without first slackening the strings.
//
// SIGNS ARE A STARTING GUESS. A standard servo turns one way as the pulse
// gets longer, and "clockwise" depends where you were standing when you
// decided. With RUN_DEMO = true it names each direction before moving. If
// FORWARD goes back, negate TILT_TRAVEL_DEG. If RIGHT goes left, negate
// PAN_TRAVEL_DEG. Rest stays put either way.
//
// DO NOT run CALIBRATE_MODE on channel 4 or 5. It sweeps set()'s 0-180,
// which is 730-2930 us - most of that is past where a string goes bar-tight.
// REST OFFSET. Rest is 1500 us - the servo's electrical centre - plus an
// offset in HORN DEGREES, because that is the unit the problem arrives in:
// you look at the servo and see the arm is a quarter turn from where it
// should be. *** 4 Sep 2026: pan (ch 5) is 90 degrees out. *** Its horn was
// seated while something else was driving the servo, so the mechanism's
// neutral and the servo's centre are a quarter turn apart, and the head sat
// permanently leaning at "rest".
//
// SIGN. Negative is anticlockwise on most hobby servos viewed from the horn
// side, but that depends on the servo and on which side you are standing.
// If the arm ends up 90 degrees the WRONG way, change -90 to +90.
//
// READ THE STARTUP PRINT. setup() prints each axis's rest pulse and the
// pulses at full travel either side, and says CLAMPED if an end falls
// outside the servo's 500-2500 us range. A rest 90 degrees off centre is
// 501 or 2499 us - hard against an end - so on a 180-degree servo one
// direction of lean has nowhere to go and WILL print CLAMPED. That is the
// mechanism telling you the horn still wants refitting: this offset buys an
// upright head today, not a finished axis. On a 270-degree servo (7.4
// us/deg) the same 90 degrees is only 667 us and both directions fit.
const uint8_t TILT_CH = 4, PAN_CH = 5;
const float   NECK_US_PER_DEG  = 11.1f;   // 500-2500 us / 180 deg. 270-deg servo: 7.4

const float   TILT_REST_OFFSET_DEG = 30;   // horn degrees from 1500 us
const float   PAN_REST_OFFSET_DEG  = -70;  // measured at the bench 4 Sep 2026
// The -90 that was here bought an upright head and cost every bit of travel
// on one side, which is arithmetic, not bad luck. A servo lives in
// 500-2500 us. Rest 90 degrees anticlockwise of centre is 501 us, w hich
// leaves 1 us of room below it - the servo is sitting ON its end stop, and
// no travel setting recovers that side. Nor does negating the travel: that
// only swaps which direction is the dead one.
//
// The offset stays available because it is the right tool for a small
// seating error. At +/-45 degrees of travel, anything from -45 to +45 keeps
// both directions alive; past about -50 one side starts to clamp. The boot
// print says which you have got.

const float   TILT_REST_US     = 1500.0f + TILT_REST_OFFSET_DEG * NECK_US_PER_DEG;
const float   TILT_TRAVEL_DEG  = +45;     // +100 = head BACK = horn +45 deg
const float   PAN_REST_US      = 1500.0f + PAN_REST_OFFSET_DEG  * NECK_US_PER_DEG;
const float   PAN_TRAVEL_DEG   = -60;     // +100 = lean RIGHT = horn -60 deg
// *** Raised from 45 to 60 on 4 Sep 2026 to give the clockwise side more to
// work with. The servo has room: rest is 723 us and clockwise runs UP the
// pulse range, so full travel is 723 + 60x11.1 = 1389 us against a 2500 us
// ceiling - about 160 degrees are physically available that way. What is NOT
// known from here is what the STRING will take. 45 was inherited from the
// old cable neck and may never have been tested on the pulley loops, so
// watch and listen on the first idle cycle: if the loop winds bar-tight or
// the servo strains at full lean, bring this back down. Nothing is at risk
// anticlockwise - that side hits the 500 us floor at 20 degrees either way.
//
// Note this also changes what the Pi means by N: -100 is now a 60 degree
// lean rather than 45. Nothing needs editing on the Pi, but the head will
// lean further for the same command.

// { channel, restUs, travelDeg, usPerDeg }. The Neck class clamps every
// pulse to rest +/- |travel|, so nothing upstream can drive either servo
// past the string limit however it is asked.
NeckAxis neckPan  = { PAN_CH,  PAN_REST_US,  PAN_TRAVEL_DEG,  NECK_US_PER_DEG };
NeckAxis neckTilt = { TILT_CH, TILT_REST_US, TILT_TRAVEL_DEG, NECK_US_PER_DEG };

Neck neck(bank, neckPan, neckTilt);

// ---- WHICH SWIVEL MOTOR? ----  *** 11 Sep 2026: new motor on channel 6 ***
//
// Set SWIVEL_MOTOR to match the servo now on channel 6. Until you do, the
// swivel is OFF: channel 6 gets no pulse at all and Y commands are ignored.
//
//   SWIVEL_POSITIONAL - an ordinary 180 or 270 degree servo. The pulse sets an
//                       ANGLE, so it can crawl smoothly, and where it points is
//                       exact: no drift, nothing to re-zero. Uses TURN_ below.
//   SWIVEL_CONTINUOUS - a "360" / continuous-rotation servo. The pulse sets a
//                       SPEED and where the head points is only an estimate.
//                       Uses the SWIVEL_ numbers further down and needs
//                       SWIVEL_CAL_MODE first. It cannot truly crawl: below
//                       SWIVEL_SLOW_DEG_S it can only move in short bursts.
//
// NOT SURE WHICH? The label or listing says 180 / 270 (positional) or 360 /
// continuous. Or set NECK_CAL_CH = 6 in the Behaviour block and jog it from
// 1500 in 10 us steps: a positional servo moves a little and HOLDS there, a
// continuous one keeps TURNING for as long as the pulse is off its stop.
#define SWIVEL_NONE       0
#define SWIVEL_POSITIONAL 1
#define SWIVEL_CONTINUOUS 2
#define SWIVEL_MOTOR      SWIVEL_NONE

const bool    TURN_ENABLED   = (SWIVEL_MOTOR == SWIVEL_POSITIONAL);
const bool    SWIVEL_ENABLED = (SWIVEL_MOTOR == SWIVEL_CONTINUOUS);
const uint8_t SWIVEL_CH      = 6;

// ---- Positional swivel (SWIVEL_POSITIONAL) ----
// pulse = TURN_CENTRE_US + (Y / 100) * TURN_TRAVEL_US, eased very slowly.
// MEASURE both with the jog tool (NECK_CAL_CH = 6), head fitted:
//   TURN_CENTRE_US - the pulse where the head faces straight ahead
//   TURN_TRAVEL_US - how far either side of that is safe for the cables, in us,
//                    minus a margin; the SMALLER side if they differ. SIGNED:
//                    flip it if Y+ turns the head the opposite way to E+.
// A positional servo jumps to its pulse at full speed the moment it powers
// up, so centre the head by hand before switching on.
const float TURN_CENTRE_US    = 1500;  // MEASURE
const float TURN_TRAVEL_US    = 150;   // MEASURE - deliberately small until you have
const float TURN_SPEED_PCT_S  = 12;    // top speed, percent of travel per second: very slow
const float TURN_ACCEL_PCT_S2 = 20;    // gentle start and stop

// ---- Continuous swivel (SWIVEL_CONTINUOUS) ----  *** REBUILT 3 Sep 2026 ***
//
// A continuous-rotation servo on channel 6 (was 2), pinion glued to its horn,
// driving the ring gear the head's base bolts to at about 4:1. The gear
// ratio itself never appears in the code: both speeds below are measured
// at the HEAD, so whatever the teeth count is, it is already in them.
//
// READ THE HEADER OF swivel.h BEFORE TOUCHING THIS. A continuous servo is
// commanded in SPEED, not position. Where the head is pointing is only ever
// an estimate, integrated from what it was told to do, and every number in
// this block is a statement about the physical servo that the estimate
// depends on. Guessed numbers here mean a head that ends up pointing
// somewhere other than where it thinks it is - and the harness through the
// middle of the ring gear is what pays for that.
//
// CALIBRATE, DO NOT GUESS: set SWIVEL_CAL_MODE = true below. It holds the
// stop pulse (creep check), then runs timed spins at the slow and fast
// pulse offsets in both directions. Mark the ring and the plate, read off
// the degrees each spin moved, divide by the seconds printed. Do this with
// the harness NOT yet threaded through the ring, or with plenty of slack.
//
// The previous crank-driven swivel (28 Aug) is gone: it ran out of ring
// rotation for geometric reasons. This mechanism has no over-centre limit,
// so the travel is now a software choice - SWIVEL_MAX_DEG - not a mechanical
// accident.
const int8_t  SWIVEL_DIR     = +1;     // flip to -1 if Y+ turns the head LEFT
const float   SWIVEL_STOP_US = 1500;   // nominal. Cal mode step 1 checks it.
const float   SWIVEL_SLOW_US = 60;     // offset that just reliably moves the head
const float   SWIVEL_FAST_US = 200;    // offset for full speed (most saturate ~200)

// EXTRA PUSH ANTICLOCKWISE, in microseconds. *** Added 4 Sep 2026, for a head
// that turned visibly further clockwise than anticlockwise. ***
//
// The cause is almost always that SWIVEL_STOP_US is not centred in the
// servo's deadband. The true stop is wherever the trim pot was left, and the
// deadband is 30-90 us wide, so "no creep at 1500" only proves the pulse is
// somewhere INSIDE the band - not in the middle of it. If the true stop is
// 20 us below 1500, then at 1500 the servo is already being nudged
// clockwise, every clockwise pulse lands 20 us harder and every anticlockwise
// one 20 us softer, and the head turns further clockwise for the same
// command. Exactly the symptom.
//
// The error is ADDITIVE, so the correction adds microseconds rather than
// scaling them - a slow offset of 60 and a fast of 200 are both short by the
// same 20, not by the same percentage.
//
// TO TUNE BY EYE: raise in steps of 10 until the two directions look equal.
// TO MEASURE IT: SWIVEL_CAL_MODE step 1b walks the pulse out from stop in 5 us
// steps each way and prints each one; note the first that moves the head. The
// midpoint of the two is the true stop, and this boost is TWICE the gap
// between that and SWIVEL_STOP_US.
//
// Adding 2x the stop error here is exactly equivalent to correcting
// SWIVEL_STOP_US, but leaves the direction that already works alone. If you
// find yourself needing more than about 60, correct SWIVEL_STOP_US instead -
// SWIVEL_FAST_US + boost should stay near 200, since most continuous servos
// saturate around there and anything past it buys nothing.
const float   SWIVEL_NEG_US_BOOST = 30;

const float   SWIVEL_SLOW_US_NEG = SWIVEL_SLOW_US + SWIVEL_NEG_US_BOOST;
const float   SWIVEL_FAST_US_NEG = SWIVEL_FAST_US + SWIVEL_NEG_US_BOOST;
const float   SWIVEL_SLOW_DEG_S = 20;  // MEASURE: head deg/s at SLOW_US, turning +
const float   SWIVEL_FAST_DEG_S = 100; // MEASURE: head deg/s at FAST_US, turning +

// The same two, measured turning the OTHER way. *** Added 4 Sep 2026,
// because one direction was turning short. *** A continuous servo's stop
// point is a trim pot set by hand and its response either side of the
// deadband is not symmetric, so the same pulse offset gives different head
// speeds each way - 10 to 20 percent apart is ordinary. With one speed model
// for both, the slower direction's dead-reckoned estimate runs ahead of the
// head, the planner believes it has arrived, and that direction consistently
// stops short of where it was asked to go.
//
// A direction that turns SHORT has its speed set too HIGH: the estimate is
// counting degrees the head is not delivering. Scale the number by
// actual/asked - if a 40 degree turn only made 30, multiply by 0.75.
//
// Leave both 0 to fall back to the + numbers and behave exactly as before.
// SWIVEL_CAL_MODE now prints which of the four each spin belongs to.
// ONE KNOB for the direction imbalance, which is the whole of the swivel's
// one-way creep. The idle wander is now PAIRED - out by some angle, pause,
// back by exactly that angle - so after every return the head should be back
// where it started. Whatever it has slipped by instead is this number being
// wrong, and it shows up once per cycle where you can see it rather than
// accumulating invisibly.
//
// 1.00 means "the two directions run at the same speed". Watch four or five
// full cycles:
//   head creeps ANTICLOCKWISE over the cycles  ->  RAISE this
//   head creeps CLOCKWISE                      ->  LOWER it
// Move it in steps of 0.05. The reason that works: creeping a direction means
// the head is delivering more degrees than the model counts, so the model's
// speed for that direction is set too low.
//
// Setting this is worth far more than any behaviour tuning - untrimmed, a
// simulated 10 minutes of idling left the head 166 degrees round with a
// perfectly balanced command pattern.
const float   SWIVEL_NEG_SCALE = 1.00f;

const float   SWIVEL_SLOW_DEG_S_NEG = SWIVEL_SLOW_DEG_S * SWIVEL_NEG_SCALE;
const float   SWIVEL_FAST_DEG_S_NEG = SWIVEL_FAST_DEG_S * SWIVEL_NEG_SCALE;
const float   SWIVEL_MAX_DEG    = 60;  // software limit each side of centre.
                                       // Human neck yaw is ~70-80; 60 is
                                       // plenty and protects the harness.

// Home sensor, wired between this pin and GND (the pin's internal pull-up
// does the rest, so it reads LOW when the switch is closed). -1 = not
// fitted, and the head then assumes it is centred at power-up. With it
// fitted the head finds centre by itself at boot and every pass through
// centre corrects the estimate. It is the ONLY defence against drift.
//
//   HOME_SIDE   (build this one) - a microswitch whose lever rides on a
//                half-moon cam printed on the ring gear, closed for the
//                whole LEFT half of travel. The edge of the cam is centre.
//   HOME_WINDOW - a Hall sensor and a magnet at centre. Works, but the
//                head has to search blind at boot. See swivel.h.
const int8_t   SWIVEL_HOME_PIN      = -1;         // e.g. 2. 0/1 are serial, A4/A5 are I2C.
const HomeKind SWIVEL_HOME_KIND     = HOME_SIDE;
const int8_t   SWIVEL_HOME_LOW_SIDE = -1;         // SIDE: -1 = closed when head is LEFT of centre
const float    SWIVEL_HOME_HALF_WIDTH_DEG = 3.0f; // WINDOW: half the arc it reads over

// How it moves. Speeds in head deg/s; accel shared by everything.
// Idle glances are deliberately unhurried - a resting person does not whip
// their head round. Driven turns (tracking a face) can be brisker.
const float SWIVEL_IDLE_SPEED   = 35;
const float SWIVEL_DRIVEN_SPEED = 20;   // following a face: as slow as the servo allows
// Idle glances when no Pi is connected. OFF for a new, uncalibrated motor:
// with it off the head just returns to centre when the Pi stops driving.
const bool  SWIVEL_IDLE_GLANCES = false;
const float SWIVEL_ACCEL        = 110;  // deg/s^2. Lower = gentler ease in/out
const unsigned int SWIVEL_EYE_LEAD_MS = 180;  // eyes go first, head follows this much later

SwivelConfig swivelCfg = {
  SWIVEL_CH, SWIVEL_DIR,
  SWIVEL_STOP_US, SWIVEL_SLOW_US, SWIVEL_FAST_US,
  SWIVEL_SLOW_US_NEG, SWIVEL_FAST_US_NEG,
  SWIVEL_SLOW_DEG_S, SWIVEL_FAST_DEG_S,
  SWIVEL_SLOW_DEG_S_NEG, SWIVEL_FAST_DEG_S_NEG,
  SWIVEL_MAX_DEG,
  SWIVEL_HOME_PIN, SWIVEL_HOME_KIND, SWIVEL_HOME_LOW_SIDE, SWIVEL_HOME_HALF_WIDTH_DEG
};
Swivel swivel(bank, swivelCfg);

// Positional swivel: a target in percent, eased towards at a very slow top
// speed with a gentle start and stop. Only used when TURN_ENABLED.
class HeadTurn {
public:
  void begin()               { last = millis(); write(); }
  void setYaw(float percent) { target = constrain(percent, -100.0f, 100.0f); }
  void update() {
    unsigned long now = millis();
    float dt = (float)(now - last) / 1000.0f;
    last = now;
    if (dt > 0.1f) dt = 0.1f;
    float err = target - pos;
    // The fastest speed it can still stop from before the target, capped.
    float want = sqrt(2.0f * TURN_ACCEL_PCT_S2 * fabs(err));
    if (want > TURN_SPEED_PCT_S) want = TURN_SPEED_PCT_S;
    if (err < 0.0f) want = -want;
    float dv = want - vel;
    float maxDv = TURN_ACCEL_PCT_S2 * dt;
    if (dv >  maxDv) dv =  maxDv;
    if (dv < -maxDv) dv = -maxDv;
    vel += dv;
    float step = vel * dt;
    if (fabs(err) < 0.05f || (err > 0.0f && step >= err) || (err < 0.0f && step <= err)) {
      pos = target;             // arrive exactly, never overshoot
      vel = 0.0f;
    } else {
      pos += step;
    }
    write();
  }
private:
  void write() { bank.setPulseUs(SWIVEL_CH, TURN_CENTRE_US + pos / 100.0f * TURN_TRAVEL_US); }
  float pos = 0.0f, vel = 0.0f, target = 0.0f;
  unsigned long last = 0;
};
HeadTurn headTurn;

// ---- Eyes ----
// Three servos per eye, driving each frame through wire pushrods. One servo
// in each bank is a 270-degree unit, so commanded degrees are ~1.5x real
// degrees there - treat every number below as units found by testing, not
// as protractor angles.
//
// EVERYTHING IS REST +eyes OFFSET. Each servo has ONE rest angle - the position
// its mechanism was assembled in - and a signed travel from there. At rest
// every servo is commanded to exactly its rest angle, and every movement
// returns to it. There are no absolute target angles anywhere.
//
// THE TWO BANKS ARE MIRROR IMAGES, so the right eye's servos run backwards
// relative to the left. Rather than a reversed flag, each side carries its
// own rest angles and its own signed travels. A perfect mirror would be
// rest_right = 180 - rest_left with the travel negated, and that is what
// the right-hand numbers below start as - but horns are never seated
// identically, so expect to tune the right side separately. That is the
// whole reason each side has its own values.
//
// LEFT EYE - calibrated and working (1 Sep 2026)
const int L_PAN_CH   = 8,  L_TILT_CH  = 9,  L_LID_CH  = 10;
const int L_PAN_REST = 45,  L_PAN_TRAVEL  = 40;   // +ve = eyes right
const int L_TILT_REST = 50, L_TILT_TRAVEL = -35; // +ve = eyes up
const int L_LID_REST = 115, L_LID_TRAVEL  = -78;  // rest = open

// RIGHT EYE - STARTING GUESSES, NOT CALIBRATED.
// Channels are assumed to be 11, 12, 13. CHECK THE ACTUAL WIRING and change
// them if not. The angles are the mathematical mirror of the left side
// (180 - rest, travel negated); calibrate them the same way the left side
// was done, one axis at a time, before trusting them.
const int R_PAN_CH   = 11, R_TILT_CH  = 12, R_LID_CH  = 13;
const int R_PAN_REST = 130, R_PAN_TRAVEL  = 40;
const int R_TILT_REST = -20,   R_TILT_TRAVEL = 35;
// WARNING: 50 anticlockwise from the mirror value of 35 would be -15, which
// does not exist - set() clamps at 0. So this is 35 anticlockwise, not 50,
// and rest now sits at the very bottom of the servo's range with NO room
// below it. Travel is positive-only for that reason: the eye can tilt one
// way from rest and not the other.
//
// If the right eye needs to look both up AND down, this cannot be fixed
// with a number. The horn has to come off the spline and go back on a few
// teeth round so rest lands nearer the middle of the range.
//
// Before adjusting this again by guesswork, run the sweep: set
// CALIBRATE_MODE = true and CALIBRATE_CH = 12 below. It steps the servo
// through every angle and prints each one, so you can read off the true
// rest position in a single upload instead of hunting for it.
const int R_LID_REST = 60,   R_LID_TRAVEL = 96;
// Read as 65 UP from the clamped floor of 0. Moving 65 further in the
// previous direction would have been -65, which the servo cannot reach -
// it was already sitting against the bottom of its range.
//
// 65 + 78 = 143, so the full blink sweep now fits with room to spare, which
// is the first time this axis has not been up against a limit.
//
// If the lids close the wrong way from here, negate the travel: 65 - 78 is
// -13, which clamps, so that would mean the horn needs refitting rather
// than another number.

// Joint limits are rest +/- travel, so nothing upstream can drive a servo
// outside the range these numbers describe.
EyeBank leftEye = {
  {  L_PAN_CH,  L_PAN_REST  - abs(L_PAN_TRAVEL),  L_PAN_REST  + abs(L_PAN_TRAVEL),  false, 0 },
  {  L_TILT_CH, L_TILT_REST - abs(L_TILT_TRAVEL), L_TILT_REST + abs(L_TILT_TRAVEL), false, 0 },
  {  L_LID_CH,  L_LID_REST  - abs(L_LID_TRAVEL),  L_LID_REST  + abs(L_LID_TRAVEL),  false, 0 },
  L_PAN_REST,  L_PAN_TRAVEL,
  L_TILT_REST, L_TILT_TRAVEL,
  L_LID_REST,  L_LID_TRAVEL
};

EyeBank rightEye = {
  {  R_PAN_CH,  R_PAN_REST  - abs(R_PAN_TRAVEL),  R_PAN_REST  + abs(R_PAN_TRAVEL),  false, 0 },
  {  R_TILT_CH, R_TILT_REST - abs(R_TILT_TRAVEL), R_TILT_REST + abs(R_TILT_TRAVEL), false, 0 },
  {  R_LID_CH,  R_LID_REST  - abs(R_LID_TRAVEL),  R_LID_REST  + abs(R_LID_TRAVEL),  false, 0 },
  R_PAN_REST,  R_PAN_TRAVEL,
  R_TILT_REST, R_TILT_TRAVEL,
  R_LID_REST,  R_LID_TRAVEL
};

// Drop the second argument to run the left eye alone while calibrating.
Eyes eyes(bank, leftEye, rightEye);

// ---- Eye-lead on idle head turns ----
// A head does not turn on its own. The eyes jump to the target first, the
// head follows a beat later, and the eyes drift back to centre as the head
// catches up - the vestibulo-ocular reflex, and the single strongest cue
// that a head turn is "looking at something" rather than a motor running.
// Without it the head rotates while the eyes stare fixedly ahead, which is
// the most machine-like thing an animatronic head can do.
//
// PCT_PER_DEG converts degrees the head still has to turn into eye gaze
// percent: at 2.2, a 30-degree glance throws the eyes ~65% over, and they
// recentre as the head arrives. It is cosmetic; tune by eye.
const float EYE_LEAD_PCT_PER_DEG     = 2.2f;
const unsigned long EYE_LEAD_SETTLE_MS = 500;   // eyes stay on target this long after arrival

// ---- Behaviour ----
const bool RUN_DEMO      = false;   // named direction check at startup

// Does the neck move on its own when no Pi is connected? OFF while the neck
// is being set up: with it off the head goes to rest at power-up and then
// only moves when told (N commands), and holds whatever it was last told -
// which is what you want while finding rest and checking directions, since
// the idle poses would otherwise overwrite them a couple of seconds after
// the serial link times out. Set true once rest and signs are confirmed.
// Has no effect while the Pi is driving: there S0/S1 decide, as before.
const bool NECK_IDLE_MOTION = true;

// ---- Neck rest finder ----  *** added 4 Sep 2026 ***
// Set to 4 or 5 to jog that neck servo by hand over the serial monitor and
// find the pulse at which its HORN is parallel to the long side of the servo
// body. 0 = off, normal running.
//
// WHY THIS EXISTS. "Rest" is not something the firmware can see. A servo has
// no position feedback, and the horn is a separate part pressed onto a
// splined shaft in any of about twenty orientations - so where the arm
// points at a given pulse is decided by whoever pushed it on, not by any
// number in this file. 1500 us is the servo's ELECTRICAL centre and the
// right thing to assume until measured, but if the horn was seated while
// something else was driving the servo, the mechanism's neutral and the
// servo's centre are different places, and the head sits leaning at "rest"
// for ever. Measure it once, put the number in REST_US, and it is fixed.
//
// TAKE THE STRING LOOP OFF THAT PULLEY FIRST, or leave it properly slack.
// This walks the servo across most of its range, which is far past where a
// fitted loop goes bar-tight.
//
// Every channel is still switched off when this runs, so only the servo
// being measured moves. Type into the serial monitor:
//   1500      go to this pulse, in microseconds
//   + / -     step 10 us       ++ / --   step 100 us
// Stop when the arm is parallel to the body. That number is this axis's
// REST_US. Then set NECK_CAL_CH back to 0 and re-upload.
//
// WHAT THE ANSWER MEANS. Travel needs room either side of rest: +/-45 horn
// degrees is +/-500 us on a 180-degree servo, +/-333 on a 270.
//   ~1230-2000 us  ->  usable. Put it in REST_US and carry on.
//   outside that   ->  the horn is on the spline in the wrong place and no
//                      number will fix it - one side of travel would run
//                      off the end of the servo. Type 1500, then pull the
//                      horn and push it back on parallel while the servo
//                      holds that pulse. Leave REST_US at 1500.
// A quarter-turn error is the common one, and 1500 +/- 90 degrees is 500 or
// 2500 us - both ends of the range - so a horn that is 90 degrees out is
// always the refit case, never the number case.
// Holds BOTH neck servos at their rest pulse and does nothing else, for as
// long as it is powered. This is the jig for re-seating the string loops:
// the servo defines upright, and you fit the loop to match it, rather than
// fitting the loop and then hunting for a number that agrees with it.
//
// A string can only PULL. The loop works both ways because turning the
// pulley pulls one side while paying the other out - but only while BOTH
// sides are taut. A loop with slack in it pulls one way and pays out into
// the slack the other, which looks exactly like a dead direction or a wrong
// sign, and no amount of editing signs will touch it.
//
// Use it: set true, upload, wait for the servos to settle, then unhook each
// loop, hold the head upright, and re-seat the loop taut on both sides
// against the pulley where it now sits. Set false, re-upload.
const bool  NECK_FIT_REST     = false;

const int   NECK_CAL_CH       = 0;      // 0 = off, 4 = tilt, 5 = pan, 6 = swivel
const float NECK_CAL_START_US = 1500;
const float NECK_CAL_MIN_US   = 800;    // safe either side of any viable rest
const float NECK_CAL_MAX_US   = 2200;

// Find any servo's true rest angle. A servo has no position feedback - it
// does not know where it is, it goes where it is told - so the only thing
// that defines "rest" is the number written above. This sweeps ONE channel
// across its whole range, pausing at each step and printing the angle, so
// you can watch the mechanism and read off the right number rather than
// guessing at it in 60-degree jumps.
//
// Set CALIBRATE_CH to whichever servo you are working on:
//    8 / 9 / 10  left eye pan / tilt / lid
//   11 / 12 / 13  right eye pan / tilt / lid
// Note the angle where the mechanism sits at REST with nothing straining,
// and the angle at full deflection - the gap between them is that axis's
// real travel.
//
// DO NOT point this at channel 2. The swivel servo is continuous: a sweep
// through 0-180 would spin it hard one way, stop briefly around 63 (which
// is where 1500 us falls in this map), then spin it hard the other way.
// Use SWIVEL_CAL_MODE for that channel.
const bool CALIBRATE_MODE = false;
const int  CALIBRATE_CH   = 13;

// Timed spins for the continuous swivel servo. See the swivel block above.
const bool SWIVEL_CAL_MODE = false;

// ---- Neck pace ----
// Percent of travel per second, and per second squared. 100 percent is the
// full |TRAVEL_DEG| (45 horn degrees), so 180 %/s is a full-range move in a
// little over half a second including the ease in and out. Idle poses are
// deliberately slower. Accel is what limits the shock on the strings:
// lower it if a move starts with a visible tug, raise it if the head feels
// treacly. Reach is how far the idle poses go, as a percent of travel.
const float NECK_IDLE_SPEED   = 70;
const float NECK_DRIVEN_SPEED = 180;
const float NECK_ACCEL        = 600;
// Idle reach in HORN DEGREES, converted to percent of travel where it is
// handed to the Neck class. Degrees because that is the unit you judge it
// in while watching the head, and because a percent silently changes
// meaning the moment TRAVEL_DEG changes. *** All three gained 10 degrees on
// 4 Sep 2026 *** - the old values were 20 / 16 / 9 degrees, and anything
// under about 5 degrees vanishes into the slack in a string loop before it
// reaches the head.
//
// Keep the sum of a reach and the gestures in neck.cpp under 45 degrees.
// 45 is the string limit, and a pose plus a gesture plus breathing drift
// that adds past it just sits on the clamp with the string at its tightest.
// One per axis PER ROTATION DIRECTION at the horn, because the two
// directions of an axis are not interchangeable here.
//
// Clockwise is deliberately the bigger of each pair. On pan that is not
// only taste: with rest at 723 us the anticlockwise side has just 20 horn
// degrees before it hits the servo's 500 us floor, while clockwise has the
// full 45. Asking anticlockwise for more than about 20 does not produce
// more movement, it produces a servo sitting on the clamp - so ACW is held
// at 18 and the reach the head actually uses is spent on the side that has
// somewhere to go. Tilt has the full 45 either way, so its split is taste.
//
// Which rotation a POSE SIGN corresponds to is worked out in setup() from
// the sign of TRAVEL_DEG, so these four stay readable if a travel sign is
// ever flipped.
const float NECK_IDLE_TILT_CW_DEG  = 34;   // clockwise     = head BACK
const float NECK_IDLE_TILT_ACW_DEG = 20;   // anticlockwise = head FORWARD
const float NECK_IDLE_PAN_CW_DEG   = 52;   // clockwise     = lean LEFT
const float NECK_IDLE_PAN_ACW_DEG  = 18;   // anticlockwise = lean RIGHT
                                           // (all the mechanism has, see above)
const int  FRAME_MS      = 20;      // PCA9685 runs at 50 Hz
const int  SPEAK_GAP_MIN = 4000;
const int  SPEAK_GAP_MAX = 9000;

// ---- Serial link ----
// How long the Pi may go quiet before the head decides it is on its own
// again. Long enough to ride out a slow frame, short enough that a crash
// mid-sentence does not leave the mouth hanging open.
const unsigned long LINK_TIMEOUT_MS = 2000;

// =======================================================

unsigned long nextSpeech = 0;
unsigned long nextFrame  = 0;

// AUTONOMOUS: no Pi. Random jaw babble, neck wander, occasional glances.
// DRIVEN:     Pi is talking to us. It owns every joint; the head's own
//             behaviours stand down so they cannot fight it.
bool driven = false;

// Eye-lead state
bool          eyesLeading    = false;
unsigned long eyeLeadRelease = 0;

void enterDriven() {
  driven = true;
  neck.setIdle(NECK_STILL);
  neck.look(0, 0);              // come to attention: upright, eased. Holding a
                                // lean for a whole conversation would load the
                                // neck servos the entire time for no reason.
  eyes.setWander(false);        // gaze stands down; auto-blink keeps going
  eyesLeading = false;
  eyeLeadRelease = 0;
  if (SWIVEL_ENABLED) {
    swivel.setWander(false);
    swivel.hold();              // finish any glance smoothly; the Pi owns it now
  }
  jaw.setGape(0);
  Serial.println(F("# driven"));
}

void enterAutonomous() {
  driven = false;
  neck.setIdle(NECK_IDLE_MOTION ? NECK_REST : NECK_STILL);
  eyes.setWander(true);
  eyesLeading = false;
  eyeLeadRelease = 0;
  if (SWIVEL_ENABLED) {
    swivel.hold();              // kills any rate command the Pi left running
    if (SWIVEL_IDLE_GLANCES) {
      swivel.setWander(true);
    } else {
      swivel.setWander(false);
      swivel.setYaw(0);         // no glances: just come back to centre
    }
  }
  if (TURN_ENABLED) headTurn.setYaw(0);   // Pi gone: ease back to centre
  jaw.setGape(0);
  nextSpeech = millis() + 1000;
  Serial.println(F("# autonomous"));
}

// Eyes first, head second, eyes back to centre as the head arrives.
// Autonomous mode only: when the Pi is driving it owns both, and can do
// the same trick itself from the camera.
void serviceEyeLead(unsigned long now) {
  if (swivel.glanceStarted()) {
    eyesLeading = true;
    eyeLeadRelease = 0;
    eyes.setWander(false);      // no saccades until this turn is done
  }
  if (!eyesLeading) return;

  // Keep the eyes on the target: as the head turns, the remaining angle
  // shrinks and so does the gaze offset. Vertical gaze is left alone.
  float lead = constrain(swivel.remainingDeg() * EYE_LEAD_PCT_PER_DEG, -85.0f, 85.0f);
  eyes.look(lead, eyes.gazeY());

  if (swivel.atTarget() && !swivel.isMoving()) {
    if (eyeLeadRelease == 0) eyeLeadRelease = now + EYE_LEAD_SETTLE_MS;
    if ((long)(now - eyeLeadRelease) >= 0) {
      eyesLeading = false;
      eyeLeadRelease = 0;
      eyes.setWander(true);
    }
  } else {
    eyeLeadRelease = 0;
  }
}

// Print what each neck axis will actually do, and say so plainly when an end
// of travel falls outside the servo's range. A clamped end is the failure
// that looks like nothing at all: the axis simply stops short, with no error
// anywhere, and it reads as a dead servo or a bad sign rather than as a rest
// position sitting too near one end.
void reportNeckAxis(float restUs, float travelDeg) {
  float span = travelDeg * NECK_US_PER_DEG;
  float lo   = restUs - fabs(span);
  float hi   = restUs + fabs(span);
  Serial.print(F("rest "));
  Serial.print(restUs, 0);
  Serial.print(F(" us, travel "));
  Serial.print(lo, 0);
  Serial.print(F(".."));
  Serial.print(hi, 0);
  Serial.print(F(" us"));
  if (lo < SERVO_PULSE_MIN_US || hi > SERVO_PULSE_MAX_US) {
    Serial.print(F("  *** CLAMPED at "));
    Serial.print((lo < SERVO_PULSE_MIN_US) ? SERVO_PULSE_MIN_US : SERVO_PULSE_MAX_US, 0);
    Serial.println(F(" us - that side has no travel, refit the horn ***"));
  } else {
    Serial.println(F("  ok"));
  }
}

void setup() {
  // Opens the port at 115200. The Pi will not be understood at any other
  // rate, and neither will you: set monitor_speed = 115200 in
  // platformio.ini or the serial monitor prints garbage.
  comms.begin(115200);

  Wire.begin();
  // Default I2C is 100 kHz. Servo writes per frame took roughly 3 ms of a
  // 20 ms budget at that speed; 400 kHz cuts it to well under 1 ms, so
  // frame timing stays even.
  Wire.setClock(400000);

  bank.begin();
  randomSeed(analogRead(A0));

  // Before any subsystem begins, so every other channel is still switched
  // off and nothing but the neck can move.
  if (NECK_FIT_REST) {
    bank.setPulseUs(TILT_CH, TILT_REST_US);
    bank.setPulseUs(PAN_CH,  PAN_REST_US);
    Serial.println(F("--- neck loop fitting ---"));
    Serial.print(F("tilt ch 4 held at "));
    Serial.print(TILT_REST_US, 0);
    Serial.print(F(" us, pan ch 5 held at "));
    Serial.print(PAN_REST_US, 0);
    Serial.println(F(" us"));
    Serial.println(F("unhook each loop, hold the head UPRIGHT, and re-seat"));
    Serial.println(F("the loop taut on BOTH sides. slack on one side reads"));
    Serial.println(F("as a dead direction. then set NECK_FIT_REST = false."));
    while (true) { delay(1000); }
  }

  if (NECK_CAL_CH == TILT_CH || NECK_CAL_CH == PAN_CH || NECK_CAL_CH == SWIVEL_CH) {
    float us = NECK_CAL_START_US;
    Serial.println(F("--- rest finder ---"));
    Serial.print(F("channel "));
    Serial.println(NECK_CAL_CH);
    Serial.println(F("type: a pulse in us, or + - (10 us), ++ -- (100 us)"));
    if (NECK_CAL_CH == SWIVEL_CH) {
      Serial.println(F("swivel. KEEP STEPS SMALL - mind the cables."));
      Serial.println(F("moves then HOLDS: positional. note the pulse facing"));
      Serial.println(F("straight ahead (TURN_CENTRE_US) and the safe limit"));
      Serial.println(F("each way (TURN_TRAVEL_US = smaller distance)."));
      Serial.println(F("keeps TURNING: continuous. type the pulse that stops it."));
    } else {
      Serial.println(F("TAKE THE STRING LOOP OFF THIS PULLEY FIRST."));
      Serial.println(F("stop when the horn is PARALLEL to the long side of"));
      Serial.println(F("the servo body. that pulse is this axis's REST_US."));
    }
    bank.setPulseUs((uint8_t)NECK_CAL_CH, us);
    Serial.print(F("  "));
    Serial.println(us, 0);

    char line[12];
    uint8_t n = 0;
    while (true) {
      while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') {
          if (n < sizeof(line) - 1) line[n++] = c;
          continue;
        }
        if (n == 0) continue;
        line[n] = '\0';
        n = 0;

        if      (line[0] == '+') us += (line[1] == '+') ? 100.0f : 10.0f;
        else if (line[0] == '-') us -= (line[1] == '-') ? 100.0f : 10.0f;
        else if (line[0] >= '0' && line[0] <= '9') us = (float)atoi(line);
        else { Serial.println(F("  ? number, or + - ++ --")); continue; }

        if (us < NECK_CAL_MIN_US) us = NECK_CAL_MIN_US;
        if (us > NECK_CAL_MAX_US) us = NECK_CAL_MAX_US;
        bank.setPulseUs((uint8_t)NECK_CAL_CH, us);
        Serial.print(F("  "));
        Serial.println(us, 0);
      }
    }
  }

  neck.begin();
  neck.setIdleSpeed(NECK_IDLE_SPEED);
  neck.setDrivenSpeed(NECK_DRIVEN_SPEED);
  neck.setAccel(NECK_ACCEL);
  // A +100 pose turns the horn by TRAVEL_DEG, so a positive pose is
  // clockwise exactly when TRAVEL_DEG is positive. Flip a travel sign and
  // these follow it; the four constants above keep meaning what they say.
  {
    float panPos  = (PAN_TRAVEL_DEG  > 0) ? NECK_IDLE_PAN_CW_DEG   : NECK_IDLE_PAN_ACW_DEG;
    float panNeg  = (PAN_TRAVEL_DEG  > 0) ? NECK_IDLE_PAN_ACW_DEG  : NECK_IDLE_PAN_CW_DEG;
    float tiltPos = (TILT_TRAVEL_DEG > 0) ? NECK_IDLE_TILT_CW_DEG  : NECK_IDLE_TILT_ACW_DEG;
    float tiltNeg = (TILT_TRAVEL_DEG > 0) ? NECK_IDLE_TILT_ACW_DEG : NECK_IDLE_TILT_CW_DEG;
    neck.setIdleReach(panPos  / fabs(PAN_TRAVEL_DEG)  * 100.0f,
                      panNeg  / fabs(PAN_TRAVEL_DEG)  * 100.0f,
                      tiltPos / fabs(TILT_TRAVEL_DEG) * 100.0f,
                      tiltNeg / fabs(TILT_TRAVEL_DEG) * 100.0f);
  }
  eyes.begin();
  if (SWIVEL_ENABLED) {
    swivel.begin();             // channel off, wherever the head is = centre
    swivel.setIdleSpeed(SWIVEL_IDLE_SPEED);
    swivel.setDrivenSpeed(SWIVEL_DRIVEN_SPEED);
    swivel.setAccel(SWIVEL_ACCEL);
    swivel.setEyeLead(SWIVEL_EYE_LEAD_MS);
  }
  if (TURN_ENABLED) headTurn.begin();
  jaw.begin();

  Serial.print(F("# neck tilt: "));
  reportNeckAxis(TILT_REST_US, TILT_TRAVEL_DEG);
  Serial.print(F("# neck pan : "));
  reportNeckAxis(PAN_REST_US, PAN_TRAVEL_DEG);

  Serial.println(F("ready"));
  delay(1500);

  if (CALIBRATE_MODE) {
    // Deliberately bypasses the Eyes class and the calibrated Joint limits,
    // because the whole point is to explore beyond what we currently think
    // the range is. Nothing else moves while this runs.
    Joint probe = { (uint8_t)CALIBRATE_CH, 0, 180, false, 0 };
    Serial.print(F("--- calibrating channel "));
    Serial.print(CALIBRATE_CH);
    Serial.println(F(" ---"));
    Serial.println(F("watch the mechanism. note the REST angle (nothing"));
    Serial.println(F("straining) and the angle at full deflection."));
    for (int a = 0; a <= 180; a += 5) {
      bank.set(probe, (float)a);
      Serial.print(F("  angle "));
      Serial.println(a);
      delay(800);
    }
    Serial.println(F("--- done ---"));
    Serial.println(F("put the rest angle in that axis's _REST constant,"));
    Serial.println(F("then set CALIBRATE_MODE back to false."));
    bank.set(probe, 90.0f);   // park mid-range; harmless wherever it is
    while (true) { delay(1000); }   // stop here; nothing else should run
  }

  if (SWIVEL_CAL_MODE && SWIVEL_ENABLED) {
    swivel.calibrate();
    while (true) { delay(1000); }
  }

  if (SWIVEL_ENABLED) {
    if (SWIVEL_HOME_PIN >= 0) {
      Serial.println(F("# swivel: homing"));
      Serial.println(swivel.homeNow() ? F("# swivel: homed")
                                      : F("# swivel: sensor NOT found, assuming centred"));
    } else {
      Serial.println(F("# swivel: no home sensor, assuming head is straight"));
    }
  } else if (TURN_ENABLED) {
    Serial.print(F("# swivel: positional, ch "));
    Serial.print(SWIVEL_CH);
    Serial.print(F(", centre "));
    Serial.print(TURN_CENTRE_US, 0);
    Serial.print(F(" us, travel "));
    Serial.print(TURN_TRAVEL_US, 0);
    Serial.println(F(" us"));
  } else {
    Serial.println(F("# swivel: OFF - set SWIVEL_MOTOR in main.cpp"));
  }

  if (RUN_DEMO) {
    Serial.println(F("--- neck direction check ---"));
    neck.demo();
    Serial.println(F("--- eye direction check ---"));
    eyes.demo();
    if (SWIVEL_ENABLED) {
      Serial.println(F("--- swivel direction check ---"));
      swivel.demo();
    }
    Serial.println(F("--- done ---"));
  }

  // Put the head into its idle behaviours explicitly. Without this the
  // swivel never wanders: enterAutonomous() otherwise only fires on a
  // link STATE CHANGE, and at boot the state already matches, so nothing
  // would ever switch it on.
  enterAutonomous();

  unsigned long now = millis();
  nextSpeech = now + 3000;
  nextFrame  = now;
}

void loop() {
  // Drain the serial buffer every pass, NOT once per frame. The Uno's
  // hardware buffer is only 64 bytes; at 115200 baud a 20 ms frame is
  // long enough to overrun it if the Pi ever bursts. This call is cheap
  // when there is nothing waiting.
  comms.update();

  // Mode follows the link. Falling back to autonomous on a timeout means
  // a Pi crash leaves a head that idles rather than one frozen mid-word.
  bool linkUp = comms.linkAlive(LINK_TIMEOUT_MS);
  if (linkUp != driven) {
    linkUp ? enterDriven() : enterAutonomous();
  }

  unsigned long now = millis();

  // Fixed-interval frame. The old delay(FRAME_MS) made the real period
  // 20 ms PLUS however long the work took, and that varied - uneven
  // frames read as stutter however smooth the maths is.
  // Signed comparison of the difference is rollover-safe.
  if ((long)(now - nextFrame) < 0) return;
  nextFrame += FRAME_MS;
  if ((long)(millis() - nextFrame) > 0) nextFrame = millis() + FRAME_MS;  // caught behind, resync

  if (driven) {
    // Only write on change. Writing every frame regardless would be
    // harmless but pointless, and it makes the serial traffic harder to
    // read when debugging.
    if (comms.jawUpdated())  jaw.setGape(comms.jawPercent());
    if (comms.neckUpdated()) neck.look(comms.neckPan(), comms.neckTilt());
    // While the Pi is SPEAKING (S1) the head nods and shifts a little on
    // top of whatever pose the Pi set. While it is LISTENING (S0) the neck
    // is dead still: the mic is inches from two servos whose whine sits in
    // the speech band. The Pi's N commands still work in both states.
    neck.setIdle(comms.isSpeaking() ? NECK_TALK : NECK_STILL);
    if (SWIVEL_ENABLED) {
      if (comms.zeroRequested()) swivel.zeroHere();
      if (comms.yawUpdated())    swivel.setYaw(comms.yawPercent());
      if (comms.rateUpdated())   swivel.setRate(comms.ratePercent());
    }
    if (TURN_ENABLED && comms.yawUpdated()) headTurn.setYaw(comms.yawPercent());
    if (comms.eyesUpdated())     eyes.look(comms.eyeX(), comms.eyeY());
    if (comms.lidUpdated())      eyes.setLids(comms.lidPercent());
    if (comms.blinkRequested())  eyes.blink();
  } else {
    if (!jaw.isSpeaking() && (long)(now - nextSpeech) >= 0) {
      jaw.speak();
      nextSpeech = now + random(SPEAK_GAP_MIN, SPEAK_GAP_MAX);
    }
    if (SWIVEL_ENABLED) serviceEyeLead(now);
  }

  neck.update();
  eyes.update();
  if (SWIVEL_ENABLED) swivel.update();
  if (TURN_ENABLED)   headTurn.update();
  jaw.update();
}