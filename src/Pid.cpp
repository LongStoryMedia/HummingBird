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
     |1| |3|
       \ /
     + roll -
       / \
     |2| |4|
      pitch
        +
*/

void Pid::processTick(float yaw, float pitch, float roll, float gx, float gy, float gz, uint32_t dt)
{
#if ESC_TEST
  r1 = thrustTarget;
  r2 = thrustTarget;
  r3 = thrustTarget;
  r4 = thrustTarget;
#else
  if (thrustTarget > 30)
  {
    // scale input
    rollError = (rollTarget * 0.85) - roll;    // current roll error
    pitchError = (pitchTarget * 0.85) - pitch; // current pitch error
    yawError = (yawTarget * 2) - gz;

    integralRollError = (0.01 * rollError * dt) + (0.01 * previousIntegralRollError);
    integralPitchError = (0.01 * pitchError * dt) + (0.01 * previousIntegralPitchError);
    integralYawError = (0.01 * yawError * dt) + (0.01 * previousIntegralYawError);
    if (thrustTarget < 35)
    {
      integralRollError = 0;
      integralPitchError = 0;
      integralYawError = 0;
    }
    integralRollError = constrain(integralRollError, -integratorLimit, integratorLimit);
    integralPitchError = constrain(integralPitchError, -integratorLimit, integratorLimit);
    integralYawError = constrain(integralYawError, -integratorLimit, integratorLimit);

    float derivativeYaw = (yawError - previousYawError) / dt;

    float rollTerm = ((rollError * KpR) + (integralRollError * KiR) - (gx * KdR)) * 0.1;
    float pitchTerm = ((pitchError * KpP) + (integralPitchError * KiP) - (gy * KdP)) * 0.1;
    float yawTerm = ((yawError * KpY) + (integralYawError * KiY) + (derivativeYaw * KdY)) * 0.1;

    r1 = constrain(thrustTarget + pitchTerm - rollTerm - yawTerm, 0, 100);
    r2 = constrain(thrustTarget - pitchTerm - rollTerm + yawTerm, 0, 100);
    r3 = constrain(thrustTarget + pitchTerm + rollTerm + yawTerm, 0, 100);
    r4 = constrain(thrustTarget - pitchTerm + rollTerm - yawTerm, 0, 100);

    previousIntegralYawError = integralYawError;
    previousIntegralPitchError = integralPitchError;
    previousIntegralRollError = integralRollError;
    previousYawError = yawError;
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

void Pid::setCoefficients(float KpRoll, float KpPitch, float KpYaw, float KiRoll, float KiPitch, float KiYaw, float KdRoll, float KdPitch, float KdYaw, float iLimit)
{
  KpR = KpRoll;
  KiR = KiRoll;
  KdR = KdRoll;
  KpP = KpPitch;
  KiP = KiPitch;
  KdP = KdPitch;
  KpY = KpYaw;
  KiY = KiYaw;
  KdY = KdYaw;
  integratorLimit = iLimit;
}

void Pid::setTargets(int16_t yaw, int16_t pitch, int16_t roll, uint16_t thrust)
{
  yawTarget = yaw;
  pitchTarget = pitch;
  rollTarget = roll; // imu is backwards I guess
  thrustTarget = thrust;
}