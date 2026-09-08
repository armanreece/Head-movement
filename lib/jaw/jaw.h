#pragma once
#include "Joints.h"

// The jaw is ONE logical joint driven by TWO servos. Callers ask for
// a gape percentage; they never see channels, mirroring or trim.
//
// NON-BLOCKING: call update() every pass through loop(). Nothing here
// waits around, so the neck can move at the same time.
class Jaw {
public:
  Jaw(JointBank &bank, Joint left, Joint right, int shutAngle, int gapeAngle);

  void begin();
  void update();                 // call every loop
  void speak();                  // start a phrase
  bool isSpeaking() const { return speaking; }

  void setGape(float percent);   // 0 = shut, 100 = fully open

private:
  struct Gesture { uint8_t gape; uint16_t ms; };
  static const int MAX_GESTURES = 24;

  float gapeToAngle(float percent) const;
  void  buildPhrase();
  void  startGesture(int i);

  JointBank &bank;
  Joint left, right;
  int   shutAngle, gapeAngle;
  float currentPercent;

  Gesture gestures[MAX_GESTURES];
  int   count, index;
  float fromPercent, toPercent;
  int   durationMs;
  unsigned long gestureStart;
  bool  speaking;
};
