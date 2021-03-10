#include "Pid.h"

// there are 3 constants: proportional, integral and derivative.
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

// however, if the error times the proportional constant
// reaches 0 (which is the goal), there would remain
// an accumulated error, which when multiplied
// by the integral constant will produce an output,
// and cause the error to be greater than 0 on the next tick.
// then the derivative portion of the output equation will add value
// along with the accumulated (integral) value causing further drift?
// or perhaps correcting for it. but even if it was correcting for it,
// it would mean that as long as the integral/accumulated error continues to grow
// the derivative will be correcting for greater and greater values,
// which in turn would make the accumulated error grow at a faster rate.
// in practice, i imagine this would look like a top wobbling more and more
// until it finally crashes and stops. but our goal is the opposite of that.
// maybe i just need to see an example, or get some starting values for these
// constants.

// anyway... that's why there's a resetTimer() function defined
// I wonder if the whole thing just needs to be reset once
// the error reaches 0

/* motor layout
        +
      pitch
     |3| |1|
       \ /
     - roll +
       / \
     |4| |2|
      pitch
        -
*/

// scenario - pitchError is 1, rollError is 1, accumulatedPitchError is 1, and accumulatedRollError is 1
// for simplicity, Kp is 1 and Ki is .1.

void Pid::processTick(int16_t pitchTarget, int16_t rollTarget, uint16_t thrust)
{
  deltaTime = millis() - time;      // milliseconds since last tick
  time = millis();                  // milliseconds since start
  rollError = roll - rollTarget;    // current roll error
  pitchError = pitch - pitchTarget; // current pitch error

  accumulatedRollError += rollError * deltaTime;
  accumulatedPitchError += pitchError * deltaTime;

  // multiply with respective coefficients
  float proportionalRollTerm = rollError * Kp;
  float proportionalPitchTerm = pitchError * Kp;
  float integralRollTerm = accumulatedRollError * Ki;
  float integralPitchTerm = accumulatedPitchError * Ki;

  // r1 should be directly inverse the error of both pitch and roll
  // take the total sum and SUBTRACT it from control1. and if control1 is at 10,
  // this would set it to 7.8
  r1 = thrust + (-proportionalPitchTerm - proportionalRollTerm) + (-integralPitchTerm - integralRollTerm);
  // r2 should be inverse to rollError, but directly related to pitchError
  // so SUBTRACT roll related multipliers, and ADD pitch related multipliers.
  // using the example from above and assuming all controls are 10,
  // this would set control2 to 10 (no change)
  r2 = thrust + (proportionalRollTerm - proportionalPitchTerm) + (integralRollTerm - integralPitchTerm);
  // r3 is the opposite of r2.
  // our example sets this also to 10 (no change)
  r3 = thrust + (proportionalPitchTerm - proportionalRollTerm) + (integralPitchTerm - integralRollTerm);
  // r4 is the opposite of r1
  // our example sets this to 12.2
  r4 = thrust + (proportionalRollTerm + proportionalPitchTerm) + (integralPitchTerm + integralRollTerm);
}

void Pid::setPitchAndRoll(int16_t pitch, int16_t roll)
{
  this->pitch = pitch;
  this->roll = roll;
}

void Pid::setCoefficients(float proportionalCoefficient, float integralCoefficient)
{
  Kp = proportionalCoefficient;
  Ki = integralCoefficient;
}