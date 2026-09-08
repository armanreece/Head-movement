#pragma once
#include <Arduino.h>

/*
 * SerialCommands.h — lib/Comms/SerialCommands.h
 *
 * Line parser for commands arriving from the Raspberry Pi over USB serial.
 * Reads only whatever is already in the hardware buffer per call and never
 * blocks, so it can run every pass through loop() without disturbing frame
 * timing.
 *
 * This is the ONLY place the Uno knows anything about the Pi. Everything
 * above it keeps working unchanged when the Pi is unplugged, and any future
 * student can drive the head by typing these same lines into a serial
 * monitor by hand.
 *
 * Protocol — one ASCII command per line, '\n' terminated, 115200 baud:
 *
 *   J<0..100>        jaw gape percent      -> Jaw::setGape()
 *   N<pan>,<tilt>    both -100..+100       -> Neck::look()
 *                    pan:  -100 = lean left, +100 = lean right
 *                    tilt: -100 = forward,   +100 = back
 *                    Since 3 Sep 2026 the neck TILTS only - pan is a
 *                    sideways lean. To TURN the head, send Y.
 *   Y<-100..100>     head swivel POSITION  -> Swivel::setYaw()
 *                    -100 = full left, 0 = ahead, +100 = full right
 *   V<-100..100>     head swivel RATE      -> Swivel::setRate()
 *                    -100 = full speed left, 0 = stop, +100 = full speed right.
 *                    For closing a loop around the camera: send the rate
 *                    every frame while a face is off-centre, V0 when it is
 *                    centred. A rate not refreshed within 600 ms lapses to
 *                    a stop, so a stalled Pi cannot drive the head into
 *                    its limit.
 *   Z                re-zero the swivel    -> Swivel::zeroHere()
 *                    "the head is straight ahead RIGHT NOW". The swivel is
 *                    dead-reckoned and drifts; this is how you correct it
 *                    by eye when there is no home sensor fitted.
 *   E<x>,<y>         eye gaze              -> Eyes::look()
 *                    x: -100 = left, +100 = right; y: -100 = down, +100 = up
 *   B                blink once            -> Eyes::blink()
 *   L<0..100>        eyelid base, 0 = open, 100 = shut -> Eyes::setLids()
 *   S0 / S1          0 = idle/listening, 1 = speaking
 *   P                ping; replies "OK"
 *
 * Percentages rather than degrees throughout: the mechanical limits live on
 * the Uno, in one place, so changing the hardware never means changing the
 * Pi code.
 *
 * Uno RAM is 2 KB, so the buffer is deliberately small. Overlong lines are
 * discarded and the parser resyncs on the next newline rather than
 * overflowing.
 */

class SerialCommands {
public:
  void begin(long baud = 115200) {
    Serial.begin(baud);
    _len = 0;
  }

  // Call once per loop(). Returns true if at least one command completed.
  bool update() {
    bool got = false;
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (_len > 0) {
          _buf[_len] = '\0';
          got |= parse(_buf);
          _len = 0;
        }
        continue;
      }

      if (_len < sizeof(_buf) - 1) {
        _buf[_len++] = c;
      } else {
        _len = 0;            // too long — drop and resync at the next newline
        _overflow = true;
      }
    }
    return got;
  }

  // ---- Values -------------------------------------------------------------
  int  jawPercent()  const { return _jaw; }     // 0..100
  int  neckPan()     const { return _pan; }     // -100..+100
  int  neckTilt()    const { return _tilt; }    // -100..+100
  int  yawPercent()  const { return _yaw; }     // -100..+100
  int  ratePercent() const { return _rate; }    // -100..+100
  int  eyeX()        const { return _ex; }      // -100..+100
  int  eyeY()        const { return _ey; }      // -100..+100
  int  lidPercent()  const { return _lid; }     // 0..100
  bool isSpeaking()  const { return _speaking; }

  // True once per newly arrived value; reading clears the flag, so a caller
  // only writes to a joint when something has actually changed.
  bool jawUpdated()  { bool f = _jawNew;  _jawNew  = false; return f; }
  bool neckUpdated() { bool f = _neckNew; _neckNew = false; return f; }
  bool yawUpdated()  { bool f = _yawNew;  _yawNew  = false; return f; }
  bool rateUpdated() { bool f = _rateNew; _rateNew = false; return f; }
  bool eyesUpdated() { bool f = _eyeNew;  _eyeNew  = false; return f; }
  bool lidUpdated()  { bool f = _lidNew;  _lidNew  = false; return f; }
  bool blinkRequested() { bool f = _blink; _blink = false; return f; }
  bool zeroRequested()  { bool f = _zero;  _zero  = false; return f; }

  // ---- Link state ---------------------------------------------------------
  // Is the Pi there and talking? False before the first command ever arrives,
  // which matters at boot — otherwise millis() starting near zero would look
  // like a command that had just been received.
  bool linkAlive(unsigned long timeoutMs) const {
    return _seen && (millis() - _lastRx) < timeoutMs;
  }

  unsigned long sinceLastCommand() const {
    return _seen ? (millis() - _lastRx) : 0xFFFFFFFFUL;
  }

  // A malformed or overlong line was seen. Worth logging while debugging —
  // it usually means a baud mismatch or a flaky cable.
  bool overflowed() { bool f = _overflow; _overflow = false; return f; }

private:
  bool parse(const char *s) {
    switch (s[0]) {
      case 'J': {
        _jaw = constrain(atoi(s + 1), 0, 100);
        _jawNew = true;
        mark();
        return true;
      }
      case 'N': {
        const char *comma = strchr(s, ',');
        if (!comma) return false;
        _pan  = constrain(atoi(s + 1),     -100, 100);
        _tilt = constrain(atoi(comma + 1), -100, 100);
        _neckNew = true;
        mark();
        return true;
      }
      case 'Y': {
        _yaw = constrain(atoi(s + 1), -100, 100);
        _yawNew = true;
        mark();
        return true;
      }
      case 'V': {
        _rate = constrain(atoi(s + 1), -100, 100);
        _rateNew = true;
        mark();
        return true;
      }
      case 'Z': {
        _zero = true;
        mark();
        return true;
      }
      case 'E': {
        const char *comma = strchr(s, ',');
        if (!comma) return false;
        _ex = constrain(atoi(s + 1),     -100, 100);
        _ey = constrain(atoi(comma + 1), -100, 100);
        _eyeNew = true;
        mark();
        return true;
      }
      case 'L': {
        _lid = constrain(atoi(s + 1), 0, 100);
        _lidNew = true;
        mark();
        return true;
      }
      case 'B': {
        _blink = true;
        mark();
        return true;
      }
      case 'S': {
        _speaking = (s[1] == '1');
        mark();
        return true;
      }
      case 'P': {
        Serial.println(F("OK"));
        mark();
        return true;
      }
      default:
        return false;
    }
  }

  void mark() {
    _lastRx = millis();
    _seen = true;
  }

  char _buf[24];
  uint8_t _len = 0;

  int  _jaw = 0, _pan = 0, _tilt = 0, _yaw = 0, _rate = 0;
  int  _ex = 0, _ey = 0, _lid = 0;
  bool _jawNew = false, _neckNew = false, _yawNew = false, _rateNew = false;
  bool _eyeNew = false, _lidNew = false, _blink = false, _zero = false;
  bool _speaking = false, _overflow = false;
  bool _seen = false;
  unsigned long _lastRx = 0;
};
