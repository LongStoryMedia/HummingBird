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
  if (PID_MODE == pidMode::simpleRate)
  {
    roll.Kp = KP_ROLL_RATE;
    roll.Ki = KI_ROLL_RATE;
    roll.Kd = KD_ROLL_RATE;
    pitch.Kp = KP_PITCH_RATE;
    pitch.Ki = KI_PITCH_RATE;
    pitch.Kd = KD_PITCH_RATE;
  }
  else
  {
    roll.Kp = KP_ROLL_ANGLE;
    roll.Ki = KI_ROLL_ANGLE;
    roll.Kd = KD_ROLL_ANGLE;
    pitch.Kp = KP_PITCH_ANGLE;
    pitch.Ki = KI_PITCH_ANGLE;
    pitch.Kd = KD_PITCH_ANGLE;
  }
  integratorLimit = I_LIMIT;
  yaw.Kp = KP_YAW;
  yaw.Ki = KI_YAW;
  yaw.Kd = KD_YAW;
  k.roll = roll;
  k.pitch = pitch;
  k.yaw = yaw;
}

void Pid::setDesiredState(State packet)
{
  desiredState.thrust = constrain(packet.thrust / 1000.0, 0.0, 1.0);
  desiredState.roll = constrain(packet.roll / 500.0, -1.0, 1.0) * MAX_ROLL;
  desiredState.pitch = constrain(packet.pitch / 500.0, -1.0, 1.0) * MAX_PITCH;
  desiredState.yaw = constrain(-packet.yaw / 500.0, -1.0, 1.0) * MAX_YAW;
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
  out.roll = 0.01 * (k.roll.Kp * error.roll + k.roll.Ki * integral.roll - k.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = desiredState.pitch - imu.accel.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = imu.gyro.pitch;
  out.pitch = 0.01 * (k.pitch.Kp * error.pitch + k.pitch.Ki * integral.pitch - k.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from imu.gyro.yaw
  error.yaw = desiredState.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / timer.delta;
  out.yaw = 0.01 * (k.yaw.Kp * error.yaw + k.yaw.Ki * integral.yaw + k.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

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
  YPR ol;
  YPR integralOl;
  YPR prevIntegralOl;
  // Roll
  error.roll = desiredState.roll - imu.accel.roll;
  integralOl.roll = prevIntegralOl.roll + error.roll * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integralOl.roll = 0;
  }
  integralOl.roll = constrain(integralOl.roll, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.roll = (imu.accel.roll - prevImu.accel.roll) / timer.delta;
  ol.roll = k.roll.Kp * error.roll + k.roll.Ki * integralOl.roll - k.roll.Kd * derivative.roll;

  // Pitch
  error.pitch = desiredState.pitch - imu.accel.pitch;
  integralOl.pitch = prevIntegralOl.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integralOl.pitch = 0;
  }
  integralOl.pitch = constrain(integralOl.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (imu.accel.pitch - prevImu.accel.pitch) / timer.delta;
  ol.pitch = k.pitch.Kp * error.pitch + k.pitch.Ki * integralOl.pitch - k.pitch.Kd * derivative.pitch;

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  ol.roll = Kl * ol.roll;
  ol.pitch = Kl * ol.pitch;
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
  out.roll = .01 * (k.roll.Kp * error.roll + k.roll.Ki * integral.roll + k.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = ol.pitch - imu.gyro.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (error.pitch - prevError.pitch) / timer.delta;
  out.pitch = .01 * (k.pitch.Kp * error.pitch + k.pitch.Ki * integral.pitch + k.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range

  // Yaw
  error.yaw = desiredState.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / timer.delta;
  out.yaw = .01 * (k.yaw.Kp * error.yaw + k.yaw.Ki * integral.yaw + k.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  prevIntegralOl.roll = integralOl.roll;
  prevIntegral.roll = integral.roll;
  prevError.roll = error.roll;
  prevImu.accel.roll = imu.accel.roll;
  prevDesiredState.roll = ol.roll;
  // Update pitch variables
  prevIntegralOl.pitch = integralOl.pitch;
  prevIntegral.pitch = integral.pitch;
  prevError.pitch = error.pitch;
  prevImu.accel.pitch = imu.accel.pitch;
  prevDesiredState.pitch = ol.pitch;
  // Update yaw variables
  prevError.yaw = error.yaw;
  prevIntegral.yaw = integral.yaw;
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
  out.roll = .01 * (k.roll.Kp * error.roll + k.roll.Ki * integral.roll + k.roll.Kd * derivative.roll); // scaled by .01 to bring within -1 to 1 range

  // Pitch
  error.pitch = desiredState.pitch - imu.gyro.pitch;
  integral.pitch = prevIntegral.pitch + error.pitch * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.pitch = 0;
  }
  integral.pitch = constrain(integral.pitch, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.pitch = (error.pitch - prevError.pitch) / timer.delta;
  out.pitch = .01 * (k.pitch.Kp * error.pitch + k.pitch.Ki * integral.pitch + k.pitch.Kd * derivative.pitch); // scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from imu.gyro.yaw
  error.yaw = desiredState.yaw - imu.gyro.yaw;
  integral.yaw = prevIntegral.yaw + error.yaw * timer.delta;
  if (isPreTakeoff)
  { // don't let integrator build if throttle is too low
    integral.yaw = 0;
  }
  integral.yaw = constrain(integral.yaw, -integratorLimit, integratorLimit); // saturate integrator to prevent unsafe buildup
  derivative.yaw = (error.yaw - prevError.yaw) / timer.delta;
  out.yaw = .01 * (k.yaw.Kp * error.yaw + k.yaw.Ki * integral.yaw + k.yaw.Kd * derivative.yaw); // scaled by .01 to bring within -1 to 1 range

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

Commands Pid::control(AccelGyro imu)
{
  switch (PID_MODE)
  {
  case pidMode::simpleAngle:
    simpleAngle(imu);
    break;

  case pidMode::cascadingAngle:
    cascadingAngle(imu);
    break;

  case pidMode::simpleRate:
    simpleRate(imu);
    break;

  default:
    simpleAngle(imu);
    break;
  }

  float m1 = desiredState.thrust - out.pitch + out.roll - out.yaw;
  float m2 = desiredState.thrust - out.pitch - out.roll + out.yaw;
  float m3 = desiredState.thrust + out.pitch - out.roll - out.yaw;
  float m4 = desiredState.thrust + out.pitch + out.roll + out.yaw;

  int m1Scaled = m1 * 125 + 125;
  int m2Scaled = m2 * 125 + 125;
  int m3Scaled = m3 * 125 + 125;
  int m4Scaled = m4 * 125 + 125;
  // Constrain commands to motors within oneshot125 bounds
  commands.m1 = constrain(m1Scaled, 125, 250);
  commands.m2 = constrain(m2Scaled, 125, 250);
  commands.m3 = constrain(m3Scaled, 125, 250);
  commands.m4 = constrain(m4Scaled, 125, 250);
  // if (timer.now > timer.prev + 2000)
  // {
  //   Serial.print("m1:");
  //   Serial.print(commands.m1);
  //   Serial.print("\tm2:");
  //   Serial.print(commands.m2);
  //   Serial.print("\tm3:");
  //   Serial.print(commands.m3);
  //   Serial.print("\tm4:");
  //   Serial.println(commands.m4);
  // }
  return commands;
}