/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <stdbool.h>
#include "fix.h"
 
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "imu.h"

//Better semantic
#define SATURATE_SINT16(in) ({ __typeof__(in) _in = (in); (_in<-32767)?-32767:((_in>32767)?32767:(int16_t)_in); })

//Fancier version
#define TRUNCATE_SINT16(out, in) ({ (out) = SATURATE_SINT16(in); })

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;

static bool isInit;

static fix_t limitSlew(const fix_t rate, const fix_t maxRate)
{
  if (rate > maxRate) return maxRate;
  if (rate < -maxRate) return -maxRate;
  return rate;
}

void controllerInit(void)
{
  if(isInit)
    return;
  
  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  
  isInit = true;
}

bool controllerTest(void)
{
  return isInit;
}

void controllerCorrectRatePID(
       fix_t rollRateActual, fix_t pitchRateActual, fix_t yawRateActual,
       fix_t rollRateDesired, fix_t pitchRateDesired, fix_t yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  pidUpdate(&pidRollRate, rollRateActual, TRUE);
  TRUNCATE_SINT16(rollOutput, pidGetOutput(&pidRollRate));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pidUpdate(&pidPitchRate, pitchRateActual, TRUE);
  TRUNCATE_SINT16(pitchOutput, pidGetOutput(&pidPitchRate));

  pidSetDesired(&pidYawRate, yawRateDesired);
  pidUpdate(&pidYawRate, yawRateActual, TRUE);
  TRUNCATE_SINT16(yawOutput, pidGetOutput(&pidYawRate));
}

void controllerCorrectAttitudePID(
       fix_t eulerRollActual, fix_t eulerPitchActual, fix_t eulerYawActual,
       fix_t eulerRollDesired, fix_t eulerPitchDesired, fix_t eulerYawDesired,
       fix_t* rollRateDesired, fix_t* pitchRateDesired, fix_t* yawRateDesired)
{
  // FIXME: do we need to limit slew?

  // Update PID for roll axis
  pidSetDesired(&pidRoll, eulerRollDesired);
  pidUpdate360(&pidRoll, eulerRollActual, true);
  *rollRateDesired = limitSlew(pidGetOutput(&pidRoll), PID_ROLL_MAX_SLEW);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  pidUpdate360(&pidPitch, eulerPitchActual, true);
  *pitchRateDesired = limitSlew(pidGetOutput(&pidPitch), PID_PITCH_MAX_SLEW);

  // Update PID for yaw axis
  pidSetDesired(&pidYaw, eulerYawDesired);
  pidUpdate360(&pidYaw, eulerYawActual, true);
  *yawRateDesired = limitSlew(pidGetOutput(&pidYaw), PID_YAW_MAX_SLEW);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FIX, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FIX, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FIX, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FIX, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FIX, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FIX, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FIX, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FIX, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FIX, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FIX, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FIX, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FIX, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FIX, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FIX, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FIX, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FIX, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FIX, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FIX, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)
