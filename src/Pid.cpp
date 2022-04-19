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
  integratorThreashold = I_TH;

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
  rollRate.Kp = KP_ROLL_RATE;
  rollRate.Ki = KI_ROLL_RATE;
  rollRate.Kd = KD_ROLL_RATE;
  pitchRate.Kp = KP_PITCH_RATE;
  pitchRate.Ki = KI_PITCH_RATE;
  pitchRate.Kd = KD_PITCH_RATE;
  rollAngle.Kp = KP_ROLL_ANGLE;
  rollAngle.Ki = KI_ROLL_ANGLE;
  rollAngle.Kd = KD_ROLL_ANGLE;
  pitchAngle.Kp = KP_PITCH_ANGLE;
  pitchAngle.Ki = KI_PITCH_ANGLE;
  pitchAngle.Kd = KD_PITCH_ANGLE;
  yawAngle.Kp = KP_YAW;
  yawAngle.Ki = KI_YAW;
  yawAngle.Kd = KD_YAW;
  yawRate.Kp = KP_YAW;
  yawRate.Ki = KI_YAW;
  yawRate.Kd = KD_YAW;
  kAngle.roll = rollAngle;
  kAngle.pitch = pitchAngle;
  kAngle.yaw = yawAngle;
  kRate.roll = rollRate;
  kRate.pitch = pitchRate;
  kRate.yaw = yawRate;
}

void Pid::setDesiredState(State packet)
{
#if defined(USE_ALT)
  integrateAlt(packet);
#else
  desiredState.thrust = constrain(packet.thrust / 1000.0, 0.0, 1.0);
#endif

  desiredState.roll = constrain(packet.roll / 500.0, -1.0, 1.0) * MAX_ROLL;
  desiredState.pitch = constrain(packet.pitch / 500.0, -1.0, 1.0) * MAX_PITCH;
  desiredState.yaw = constrain(-packet.yaw / 500.0, -1.0, 1.0) * MAX_YAW;

#if IMU_ORIENTATION == 0
  desiredState.roll *= -1.0;
  desiredState.pitch *= -1.0;
#endif
}

void Pid::integrateAlt(State packet)
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
      desiredState.thrust = lockedDesiredThrust;
    }
  }

  if (alt.altLocked == Alt::lockState::locked)
  {
    errorAlt = alt.lockedAlt - alt.alt;
    integralAlt = prevIntegralAlt + errorAlt * timer.delta;
    integralAlt = constrain(integralAlt, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
    desiredState.thrust = desiredState.thrust + (KP_ALT * errorAlt) + (KI_ALT * integralAlt);
    // limit increment/decrement
    float lowerLimit = desiredState.thrust - ALT_CONSTRAINT_INC;
    float upperLimit = desiredState.thrust + ALT_CONSTRAINT_INC;
    desiredState.thrust = constrain(desiredState.thrust, lowerLimit, upperLimit);
    // absolute throttle limit
    desiredState.thrust = constrain(desiredState.thrust, lockedDesiredThrust - ALT_CONSTRAINT_ABS, lockedDesiredThrust + ALT_CONSTRAINT_ABS);
    prevIntegralAlt = integralAlt;
  }
  else
  {
    errorAlt = 0;
    integralAlt = 0;
    prevIntegralAlt = 0;
    desiredState.thrust = constrain(packet.thrust / 1000.0, 0.0, 1.0);
  }
  // Serial.println(desiredState.thrust);
#endif
}

void Pid::simpleAngle(AccelGyro imu)
{
  bool isPreTakeoff = desiredState.thrust < (integratorThreashold / 1000.0);

  // Roll
  error.roll = desiredState.roll - imu.accel.roll;
  integral.roll = prevIntegral.roll + error.roll * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.roll = 0;
  }
  integral.roll = constrain(integral.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = imu.gyro.roll;
  out.roll = 0.01 * (kAngle.roll.Kp * error.roll + kAngle.roll.Ki * integral.roll - kAngle.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = desiredState.pitch - imu.accel.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = imu.gyro.pitch;
  out.pitch = 0.01 * (kAngle.pitch.Kp * error.pitch + kAngle.pitch.Ki * integral.pitch - kAngle.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from imu.gyro.yaw
  error.yaw = desiredState.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / timer.delta;
  out.yaw = 0.01 * (kAngle.yaw.Kp * error.yaw + kAngle.yaw.Ki * integral.yaw + kAngle.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  prevIntegral.roll = integral.roll;
  // Update pitch variables
  prevIntegral.pitch = integral.pitch;
  // Update yaw variables
  error.yaw = error.yaw;
  prevIntegral.yaw = integral.yaw;
}

void Pid::cascadingAngle(AccelGyro imu)
{
  bool isPreTakeoff = desiredState.thrust < (integratorThreashold / 1000.0);
  // Outer loop - PID on angle
  // Roll
  error.roll = desiredState.roll - imu.accel.roll;
  integralOl.roll = prevIntegralOl.roll + error.roll * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integralOl.roll = 0;
  }
  integralOl.roll = constrain(integralOl.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = (imu.accel.roll - prevImu.accel.roll) / timer.delta;
  ol.roll = kAngle.roll.Kp * error.roll + kAngle.roll.Ki * integralOl.roll - kAngle.roll.Kd * derivative.roll;

  // Pitch
  error.pitch = desiredState.pitch - imu.accel.pitch;
  integralOl.pitch = prevIntegralOl.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integralOl.pitch = 0;
  }
  integralOl.pitch = constrain(integralOl.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (imu.accel.pitch - prevImu.accel.pitch) / timer.delta;
  ol.pitch = kAngle.pitch.Kp * error.pitch + kAngle.pitch.Ki * integralOl.pitch - kAngle.pitch.Kd * derivative.pitch;

  // Apply loop gain, constrain, and LP filter for artificial damping
  ol.roll *= KL;
  ol.pitch *= KL;
  ol.roll = constrain(ol.roll, -240.0, 240.0);
  ol.pitch = constrain(ol.pitch, -240.0, 240.0);
  ol.roll = (1.0 - B_LOOP_ROLL) * prevDesiredState.roll + B_LOOP_ROLL * ol.roll;
  ol.pitch = (1.0 - B_LOOP_PITCH) * prevDesiredState.pitch + B_LOOP_PITCH * ol.pitch;

  // Inner loop - PID on rate
  // Roll
  error.roll = ol.roll - imu.gyro.roll;
  integral.roll = prevIntegral.roll + error.roll * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.roll = 0;
  }
  integral.roll = constrain(integral.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = (error.roll - prevError.roll) / timer.delta;
  out.roll = .01 * (kRate.roll.Kp * error.roll + kRate.roll.Ki * integral.roll + kRate.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = ol.pitch - imu.gyro.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (error.pitch - prevError.pitch) / timer.delta;
  out.pitch = .01 * (kRate.pitch.Kp * error.pitch + kRate.pitch.Ki * integral.pitch + kRate.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range

  // Yaw
  error.yaw = desiredState.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / timer.delta;
  out.yaw = .01 * (kRate.yaw.Kp * error.yaw + kRate.yaw.Ki * integral.yaw + kRate.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

  Serial.print("roll: ");
  Serial.print(out.roll);
  Serial.print("\tpitch: ");
  Serial.print(out.pitch);
  Serial.print("\tyaw: ");
  Serial.println(out.yaw);
  // Update roll variables
  prevError = error;
  prevIntegralOl = integralOl;
  prevIntegral = integral;
  prevImu = imu;
  prevDesiredState.roll = ol.roll;
  prevDesiredState.pitch = ol.pitch;
}

void Pid::simpleRate(AccelGyro imu)
{
  bool isPreTakeoff = desiredState.thrust < (integratorThreashold / 1000.0);
  // Roll
  error.roll = desiredState.roll - imu.gyro.roll;
  integral.roll = prevIntegral.roll + error.roll * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.roll = 0;
  }
  integral.roll = constrain(integral.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = (error.roll - prevError.roll) / timer.delta;
  out.roll = .01 * (kRate.roll.Kp * error.roll + kRate.roll.Ki * integral.roll + kRate.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = desiredState.pitch - imu.gyro.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (error.pitch - prevError.pitch) / timer.delta;
  out.pitch = .01 * (kRate.pitch.Kp * error.pitch + kRate.pitch.Ki * integral.pitch + kRate.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from imu.gyro.yaw
  error.yaw = desiredState.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / timer.delta;
  out.yaw = .01 * (kRate.yaw.Kp * error.yaw + kRate.yaw.Ki * integral.yaw + kRate.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  prevError.roll = error.roll;
  prevIntegral.roll = integral.roll;
  prevImu.gyro.roll = imu.gyro.roll;
  // Update pitch variables
  prevError.pitch = error.pitch;
  prevIntegral.pitch = integral.pitch;
  prevImu.gyro.pitch = imu.gyro.pitch;
  // Update yaw variables
  prevError.yaw = error.yaw;
  prevIntegral.yaw = integral.yaw;
}

float Pid::mix(Prop prop)
{
  return ((float)prop.xAxis * out.roll) + ((float)prop.yAxis * out.pitch) + ((float)prop.rotation * out.yaw);
}

Commands Pid::control(AccelGyro imu)
{
  switch (PID_MODE)
  {
  case simpleAngleMode:
    simpleAngle(imu);
    break;

  case cascadingMode:
    cascadingAngle(imu);
    break;

  case simpleRateMode:
    simpleRate(imu);
    break;

  default:
    simpleAngle(imu);
    break;
  }
  int low = COMMANDS_LOW;
  int high = COMMANDS_HIGH;

  float m1 = desiredState.thrust + mix(propConfig.p1);
  float m2 = desiredState.thrust + mix(propConfig.p2);
  float m3 = desiredState.thrust + mix(propConfig.p3);
  float m4 = desiredState.thrust + mix(propConfig.p4);
  // scale to protocol
  float m1Scaled = map(m1, 0.0F, 1.0F, (float)low, (float)high);
  float m2Scaled = map(m2, 0.0F, 1.0F, (float)low, (float)high);
  float m3Scaled = map(m3, 0.0F, 1.0F, (float)low, (float)high);
  float m4Scaled = map(m4, 0.0F, 1.0F, (float)low, (float)high);
  // Constrain commands to motors within bounds
  commands.m1 = constrain(m1Scaled, low, high);
  commands.m2 = constrain(m2Scaled, low, high);
  commands.m3 = constrain(m3Scaled, low, high);
  commands.m4 = constrain(m4Scaled, low, high);

  return commands;
}