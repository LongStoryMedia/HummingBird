#include "Pid.h"

// there are 3 constants: proportional, integral and derivative.
// we'll discard derivative for now to avoid windup.
// the first is the most important. it tells us by what value
// the error should be multiplied in order to adjust the given throttle
// to correct toward our set point. the last will usually by 0
// because the last error will almost always be greater than the current error.
// mathmatically, this will return negative, but we only care if it is greater
// than 0, so for our purposes, it will return 0. if it returns a value greater than 0,
// something has gone wrong and we need to correct it quickly.

// integral is more confusing in my opinion. because the way it is written,
// it looks like it would cause the output to always increase. my guess though is that
// it is supposed to prevent the output from increasing too quickly because the
// accumulated error will increase on each tick, but the error
// multiplied by our proportional contant will decrease by a greater ammount
// and so it will cause a nice, smooth transition.

/* motor layout
        -
      pitch
     |1| |2|
       \ /
     + roll -
       / \
     |3| |4|
      pitch
        +
*/

void Pid::processTick(int16_t pitch, int16_t roll, int16_t pitchTarget, int16_t rollTarget, uint16_t thrust)
{
  deltaTime = millis() - time;      // milliseconds since last tick
  time = millis();                  // milliseconds since start
  rollError = roll - rollTarget;    // current roll error
  pitchError = pitch - pitchTarget; // current pitch error

  // (rollError * Kp) + (accumulatedRollError * Ki)
  float rollTerm = (rollError * Kp) + ((rollError * (deltaTime / 1000)) * Ki);
  // (pitchError * Kp) + (accumulatedPitchError * Ki)
  float pitchTerm = (pitchError * Kp) + ((pitchError * (deltaTime / 1000)) * Ki);

  // r1 should be inverse to roll and aligned with pitch
  r1 = thrust + pitchTerm - rollTerm;
  // r2 should be inverse to both
  r2 = thrust - pitchTerm - rollTerm;
  // r3 should be aligned with both
  r3 = thrust + pitchTerm + rollTerm;
  // r4 should be aligned with roll and inverse to pitch
  r4 = thrust - pitchTerm + rollTerm;
}

void Pid::setCoefficients(float proportionalCoefficient, float integralCoefficient)
{
  Kp = proportionalCoefficient;
  Ki = integralCoefficient;
}