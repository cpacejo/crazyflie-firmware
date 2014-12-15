/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 * pid.c - implementation of the PID regulator
 */
#include "stm32f10x_conf.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "led.h"
#include "motors.h"

static fix_t wrapAngle(fix_t angle)
{
  while (angle >= 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}

void pidInit(PidObject* pid, const fix_t desired, const fix_t kp,
             const fix_t ki, const fix_t kd, const fix_t dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
  pid->idt       = 1.0k / dt;
}

void pidUpdate(PidObject* pid, const fix_t measured, const bool updateError)
{
    if (updateError)
    {
        pid->prevError = pid->error;
        pid->error = pid->desired - measured;
    }

    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < -pid->iLimit)
    {
        pid->integ = -pid->iLimit;
    }

    pid->deriv = (sat_fix_t) (pid->error - pid->prevError) * (sat_fix_t) pid->idt;

    pid->outP = (sat_fix_t) pid->kp * (sat_fix_t) pid->error;
    pid->outI = (sat_fix_t) pid->ki * (sat_fix_t) pid->integ;
    pid->outD = (sat_fix_t) pid->kd * (sat_fix_t) pid->deriv;
}

void pidUpdate360(PidObject* pid, const fix_t measured, const bool updateError)
{
    if (updateError)
    {
        pid->prevError = pid->error;
        pid->error = wrapAngle(pid->desired - measured);
    }

    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < -pid->iLimit)
    {
        pid->integ = -pid->iLimit;
    }

    pid->deriv = (sat_fix_t) wrapAngle(pid->error - pid->prevError) * (sat_fix_t) pid->idt;

    pid->outP = (sat_fix_t) pid->kp * (sat_fix_t) pid->error;
    pid->outI = (sat_fix_t) pid->ki * (sat_fix_t) pid->integ;
    pid->outD = (sat_fix_t) pid->kd * (sat_fix_t) pid->deriv;
}

fix_t pidGetOutput(PidObject* pid)
{
    return (sat_fix_t) pid->outP + (sat_fix_t) pid->outI + (sat_fix_t) pid->outD;
}

void pidSetIntegralLimit(PidObject* pid, const fix_t limit) {
    pid->iLimit = limit;
}


void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const fix_t error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const fix_t desired)
{
  pid->desired = desired;
}

fix_t pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = TRUE;

  if (pid->kp < 0.0001k && pid->ki < 0.0001k && pid->kd < 0.0001k)
  {
    isActive = FALSE;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const fix_t kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const fix_t ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const fix_t kd)
{
  pid->kd = kd;
}

void pidSetDt(PidObject* pid, const fix_t dt) {
    pid->dt = dt;
}
