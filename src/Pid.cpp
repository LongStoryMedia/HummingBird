#include "Pid.h"

void Pid::processTick(float pitchTarget, float rollTarget)
{
    deltaTime = millis() - time;      // milliseconds since last tick
    time = millis();                  // milliseconds since start
    rollError = roll - rollTarget;    // current roll error
    pitchError = pitch - pitchTarget; // current pitch error

    accumulatedRollError += rollError * deltaTime;   // for KI (accumulatedError * Ki)
                                                     // should represent a curve fluxuating
                                                     // around the set point
    accumulatedPitchError += pitchError * deltaTime; // for KI (accumulatedError * Ki)
                                                     // should represent a curve fluxuating
                                                     // around the set point
    proportionalTerm = (rollError * Kp) + (pitchError * Kp);
    integralTerm = (accumulatedPitchError * Ki) + (accumulatedRollError * Ki);
    r1 = r1correct();
    r2 = r2correct();
    r3 = r3correct();
    r4 = r4correct();
}

uint16_t Pid::r1correct()
{
}

uint16_t Pid::r2correct()
{
}

uint16_t Pid::r3correct()
{
}

uint16_t Pid::r4correct()
{
}
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