#include "config.h"

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

void Pid::processTick(int16_t pitch, int16_t roll)
{
#if ESC_TEST
  r1 = thrustTarget;
  r2 = thrustTarget;
  r3 = thrustTarget;
  r4 = thrustTarget;
#else

  deltaTime = millis() - time; // milliseconds since last tick
  time = millis();             // milliseconds since start

  if (thrustTarget > 10)
  {
    rollError = roll - rollTarget;    // current roll error
    pitchError = pitch - pitchTarget; // current pitch error
    integralRollError = constrain((rollError * deltaTime) + integralRollError, -25, 25);
    integralPitchError = constrain((pitchError * deltaTime) + integralPitchError, -25, 25);

    float rollTerm = (rollError * Kp) + (integralRollError * Ki);
    float pitchTerm = (pitchError * Kp) + (integralPitchError * Ki);

    // float derivedRoll = constrain(abs(derivativeRollError * Kd), 0, 20);
    // float derivedPitch = constrain(abs(derivativePitchError * Kd), 0, 20);

    // r1 should be inverse to both
    r1 = thrustTarget - pitchTerm - rollTerm;
    // r2 should be aligned with roll and inverse to pitch
    r2 = thrustTarget - pitchTerm + rollTerm;
    // r3 should be inverse to roll and aligned with pitch
    r3 = thrustTarget + pitchTerm - rollTerm;
    // r4 should be aligned with both
    r4 = thrustTarget + pitchTerm + rollTerm;

    //   if (derivativePitchError > 0)
    //   {
    //     r1 += derivedPitch;
    //     r2 += derivedPitch;
    //     r3 -= derivedPitch;
    //     r4 -= derivedPitch;
    //   }
    //   else
    //   {
    //     r1 -= derivedPitch;
    //     r2 -= derivedPitch;
    //     r3 += derivedPitch;
    //     r4 += derivedPitch;
    //   }
    //   if (derivativeRollError > 0)
    //   {
    //     r1 += derivedRoll;
    //     r2 -= derivedRoll;
    //     r3 += derivedRoll;
    //     r4 -= derivedRoll;
    //   }
    //   else
    //   {
    //     r1 -= derivedRoll;
    //     r2 += derivedRoll;
    //     r3 -= derivedRoll;
    //     r4 += derivedRoll;
    //   }
  }
  else
  {
    r1 = 0;
    r2 = 0;
    r3 = 0;
    r4 = 0;
  }
#endif
}

void Pid::setCoefficients(float proportionalCoefficient, float integralCoefficient, float derivativeCoefficient)
{
  Kp = proportionalCoefficient;
  Ki = integralCoefficient;
  Kd = derivativeCoefficient;
}

void Pid::setTargets(int16_t pitch, int16_t roll, uint16_t thrust)
{
  pitchTarget = pitch;
  rollTarget = roll;
  thrustTarget = thrust;
}

void Pid::setDerivatives(float rollDerivative, float pitchDerivative)
{
  derivativePitchError = pitchDerivative;
  derivativeRollError = rollDerivative;
}