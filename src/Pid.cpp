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
     |4| |3|
      pitch
        +
*/

void Pid::init()
{
  switch (IMU_ORIENTATION)
  {
  case imuOrientation::upForward:
    propConfig.p1.xAxis = propConfig.p1.negative;
    propConfig.p1.yAxis = propConfig.p1.positive;
    propConfig.p2.xAxis = propConfig.p2.positive;
    propConfig.p2.yAxis = propConfig.p2.positive;
    propConfig.p3.xAxis = propConfig.p3.positive;
    propConfig.p3.yAxis = propConfig.p3.negative;
    propConfig.p4.xAxis = propConfig.p1.negative;
    propConfig.p4.yAxis = propConfig.p1.negative;
    break;

  case imuOrientation::upBackward:
    propConfig.p1.xAxis = propConfig.p1.positive;
    propConfig.p1.yAxis = propConfig.p1.negative;
    propConfig.p2.xAxis = propConfig.p2.negative;
    propConfig.p2.yAxis = propConfig.p2.negative;
    propConfig.p3.xAxis = propConfig.p3.negative;
    propConfig.p3.yAxis = propConfig.p3.positive;
    propConfig.p4.xAxis = propConfig.p1.positive;
    propConfig.p4.yAxis = propConfig.p1.positive;
    break;

  default:
    propConfig.p1.xAxis = propConfig.p1.negative;
    propConfig.p1.yAxis = propConfig.p1.positive;
    propConfig.p2.xAxis = propConfig.p2.positive;
    propConfig.p2.yAxis = propConfig.p2.positive;
    propConfig.p3.xAxis = propConfig.p3.positive;
    propConfig.p3.yAxis = propConfig.p3.negative;
    propConfig.p4.xAxis = propConfig.p1.negative;
    propConfig.p4.yAxis = propConfig.p1.negative;
    break;
  }

  integratorLimit = I_LIMIT;
  kRate.roll.Kp = KP_ROLL_RATE;
  kRate.roll.Ki = KI_ROLL_RATE;
  kRate.roll.Kd = KD_ROLL_RATE;
  kRate.pitch.Kp = KP_PITCH_RATE;
  kRate.pitch.Ki = KI_PITCH_RATE;
  kRate.pitch.Kd = KD_PITCH_RATE;
  kAngle.roll.Kp = KP_ROLL_ANGLE;
  kAngle.roll.Ki = KI_ROLL_ANGLE;
  kAngle.roll.Kd = KD_ROLL_ANGLE;
  kAngle.pitch.Kp = KP_PITCH_ANGLE;
  kAngle.pitch.Ki = KI_PITCH_ANGLE;
  kAngle.pitch.Kd = KD_PITCH_ANGLE;
  kAngle.yaw.Kp = KP_YAW;
  kAngle.yaw.Ki = KI_YAW;
  kAngle.yaw.Kd = KD_YAW;
  kRate.yaw.Kp = KP_YAW;
  kRate.yaw.Ki = KI_YAW;
  kRate.yaw.Kd = KD_YAW;
}

void Pid::setDesiredState(Input packet)
{
#if defined(USE_ALT)
  integrateAlt(packet);
#else
  state.thrust = constrain(packet.thrust / 1000.0, 0.0, 1.0);
#endif

  state.roll = constrain(packet.roll / 500.0, -1.0, 1.0) * MAX_ROLL;
  state.pitch = constrain(packet.pitch / 500.0, -1.0, 1.0) * MAX_PITCH;
  state.yaw = constrain(-packet.yaw / 500.0, -1.0, 1.0) * MAX_YAW;

#if IMU_ORIENTATION == 0
  state.roll *= -1.0;
  state.pitch *= -1.0;
#endif
}

void Pid::integrateAlt(Input packet)
{
#if defined(USE_ALT)
  if (alt.altLocked != (Alt::lockState)packet.lockAlt)
  {
    alt.altLocked = (Alt::lockState)packet.lockAlt;
    if (packet.lockAlt == Alt::lockState::locked)
    {
      alt.lockedAlt = alt.getAlt();
      alt.lockedThrust = packet.thrust;
      lockedDesiredThrust = packet.thrust / 1000.0;
      state.thrust = lockedDesiredThrust;
    }
  }

  if (alt.altLocked == Alt::lockState::locked)
  {
    errorAlt = alt.lockedAlt - alt.alt;
    integralAlt = prevIntegralAlt + errorAlt * fcTimer.delta;
    integralAlt = constrain(integralAlt, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
    state.thrust = state.thrust + (KP_ALT * errorAlt) + (KI_ALT * integralAlt);
    // limit increment/decrement
    float lowerLimit = state.thrust - ALT_CONSTRAINT_INC;
    float upperLimit = state.thrust + ALT_CONSTRAINT_INC;
    state.thrust = constrain(state.thrust, lowerLimit, upperLimit);
    // absolute throttle limit
    state.thrust = constrain(state.thrust, lockedDesiredThrust - ALT_CONSTRAINT_ABS, lockedDesiredThrust + ALT_CONSTRAINT_ABS);
    prevIntegralAlt = integralAlt;
  }
  else
  {
    errorAlt = 0;
    integralAlt = 0;
    prevIntegralAlt = 0;
    state.thrust = constrain(packet.thrust / 1000.0, 0.0, 1.0);
  }
  // Serial.println(state.thrust);
#endif
}

float Pid::mix(Prop prop)
{
  return ((float)prop.xAxis * out.roll) + ((float)prop.yAxis * out.pitch) + ((float)prop.rotation * out.yaw);
}

bool Pid::isPreTakeoff()
{
  return state.thrust < (I_TH / 1000.0);
}

void Pid::angleLoop(const AccelGyro &imu)
{
  // Outer loop - PID on angle
  // Roll
  error.roll = state.roll - imu.accel.roll;
  integralOl.roll = prevIntegralOl.roll + error.roll * fcTimer.delta;
  if (isPreTakeoff())
  { // don't let integrator build if throttle is too low
    integralOl.roll = 0;
  }
  integralOl.roll = constrain(integralOl.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = (imu.accel.roll - prevImu.accel.roll) / fcTimer.delta;
  ol.roll = kAngle.roll.Kp * error.roll + kAngle.roll.Ki * integralOl.roll - kAngle.roll.Kd * derivative.roll;

  // Pitch
  error.pitch = state.pitch - imu.accel.pitch;
  integralOl.pitch = prevIntegralOl.pitch + error.pitch * fcTimer.delta;
  if (isPreTakeoff())
  { // don't let integrator build if throttle is too low
    integralOl.pitch = 0;
  }
  integralOl.pitch = constrain(integralOl.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (imu.accel.pitch - prevImu.accel.pitch) / fcTimer.delta;
  ol.pitch = kAngle.pitch.Kp * error.pitch + kAngle.pitch.Ki * integralOl.pitch - kAngle.pitch.Kd * derivative.pitch;
}

void Pid::rateLoop(const AccelGyro &imu)
{
  // Inner loop - PID on rate
  // Roll
  error.roll = ol.roll - imu.gyro.roll;
  integral.roll = prevIntegral.roll + error.roll * fcTimer.delta;
  if (isPreTakeoff())
  { // don't let integrator build if throttle is too low
    integral.roll = 0;
  }
  integral.roll = constrain(integral.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = (error.roll - prevError.roll) / fcTimer.delta;
  out.roll = .01 * (kRate.roll.Kp * error.roll + kRate.roll.Ki * integral.roll + kRate.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = ol.pitch - imu.gyro.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * fcTimer.delta;
  if (isPreTakeoff())
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (error.pitch - prevError.pitch) / fcTimer.delta;
  out.pitch = .01 * (kRate.pitch.Kp * error.pitch + kRate.pitch.Ki * integral.pitch + kRate.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range
}

Commands Pid::control(const AccelGyro &imu)
{
  angleLoop(imu);
  // Apply loop gain, constrain, and LP filter for artificial damping
  ol.roll *= KL;
  ol.pitch *= KL;
  ol.roll = constrain(ol.roll, -240.0, 240.0);
  ol.pitch = constrain(ol.pitch, -240.0, 240.0);
  ol.roll = (1.0 - B_LOOP_ROLL) * prevState.roll + B_LOOP_ROLL * ol.roll;
  ol.pitch = (1.0 - B_LOOP_PITCH) * prevState.pitch + B_LOOP_PITCH * ol.pitch;
  rateLoop(imu);
  // Yaw
  error.yaw = state.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * fcTimer.delta;
  if (isPreTakeoff())
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / fcTimer.delta;
  out.yaw = .01 * (kRate.yaw.Kp * error.yaw + kRate.yaw.Ki * integral.yaw + kRate.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

  // Serial.print("roll: ");
  // Serial.print(out.roll);
  // Serial.print("\tpitch: ");
  // Serial.print(out.pitch);
  // Serial.print("\tyaw: ");
  // Serial.println(out.yaw);
  // Update roll variables
  prevError = error;
  prevIntegralOl = integralOl;
  prevIntegral = integral;
  prevImu = imu;
  prevState.roll = ol.roll;
  prevState.pitch = ol.pitch;

  float m1 = state.thrust + mix(propConfig.p1);
  float m2 = state.thrust + mix(propConfig.p2);
  float m3 = state.thrust + mix(propConfig.p3);
  float m4 = state.thrust + mix(propConfig.p4);
  // scale to protocol
  float m1Scaled = map(m1, 0.0F, 1.0F, (float)COMMANDS_LOW, (float)COMMANDS_HIGH);
  float m2Scaled = map(m2, 0.0F, 1.0F, (float)COMMANDS_LOW, (float)COMMANDS_HIGH);
  float m3Scaled = map(m3, 0.0F, 1.0F, (float)COMMANDS_LOW, (float)COMMANDS_HIGH);
  float m4Scaled = map(m4, 0.0F, 1.0F, (float)COMMANDS_LOW, (float)COMMANDS_HIGH);
  // Constrain commands to motors within bounds
  commands.m1 = constrain(m1Scaled, COMMANDS_LOW, COMMANDS_HIGH);
  commands.m2 = constrain(m2Scaled, COMMANDS_LOW, COMMANDS_HIGH);
  commands.m3 = constrain(m3Scaled, COMMANDS_LOW, COMMANDS_HIGH);
  commands.m4 = constrain(m4Scaled, COMMANDS_LOW, COMMANDS_HIGH);

  return commands;
}