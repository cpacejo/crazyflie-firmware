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
#include "stm32f10x_conf.h"
#include "fix.h"
#include "quaternion.h"

#include "sensfusion6.h"
#include "imu.h"
#include "param.h"

#define KP_DEF  0.4k // proportional gain
#define KI_DEF  0.001k // integral gain

static fix_t kp = KP_DEF;    // proportional gain (Kp)
static fix_t ki = KI_DEF;    // integral gain (Ki)
static fix_t integralFBx = 0.0k;
static fix_t integralFBy = 0.0k;
static fix_t integralFBz = 0.0k;  // integral error terms scaled by Ki


static quaternion_t q = Q_UNIT_INIT;  // quaternion of sensor frame relative to auxiliary frame

static bool isInit;

void sensfusion6Init(void)
{
  if(isInit)
    return;

  isInit = TRUE;
}

bool sensfusion6Test(void)
{
  return isInit;
}


// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
void sensfusion6UpdateQ(fix_t gx, fix_t gy, fix_t gz, fix_t ax, fix_t ay, fix_t az, fix_t dt)
{
  gx *= M_PI_FIX / 180.0k;
  gy *= M_PI_FIX / 180.0k;
  gz *= M_PI_FIX / 180.0k;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!(fabsfix(ax) < 0.01k && fabsfix(ay) < 0.01k && fabsfix(az) < 0.01k))
  {
    // Normalise accelerometer measurement
    const fix_t recipNorm = invsqrtfix(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    fix_t vx, vy, vz;
    quaternion_t q_conj;
    qConj(&q, &q_conj);  // why the conjugation here?
    qRot(0.0k, 0.0k, 1.0k, &q_conj, &vx, &vy, &vz);

    // Error is cross product between estimated and measured direction of gravity
    // TODO: should this be the other way around?
    const fix_t ex = ay * vz - az * vy;
    const fix_t ey = az * vx - ax * vz;
    const fix_t ez = ax * vy - ay * vx;

    // Compute and apply integral feedback
    integralFBx += ki * ex * dt;  // integral error scaled by Ki
    integralFBy += ki * ey * dt;
    integralFBz += ki * ez * dt;
    gx += integralFBx;  // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;

    // Apply proportional feedback
    gx += kp * ex;
    gy += kp * ey;
    gz += kp * ez;
  }

  // Integrate rate of change of quaternion
  quaternion_t gq, q_new;
  qFromRPYRate(gy, gx, gz, dt, &gq);
  qMul(&q, &gq, &q_new);  // FIXME: This should be other way around

  // Normalise quaternion
  qUnitize(&q_new, &q);
}

#include <math.h>
void sensfusion6GetEulerRPY(fix_t* roll, fix_t* pitch, fix_t* yaw)
{
  fix_t gx, gy, gz; // estimated gravity direction

  gx = 2.0k * (q.i*q.k - q.r*q.j);
  gy = 2.0k * (q.r*q.i + q.j*q.k);
  gz = q.r*q.r - q.i*q.i - q.j*q.j + q.k*q.k;

  if (gx>1.0k) gx=1.0k;
  if (gx<-1.0k) gx=-1.0k;

  // FIXME
  *yaw = atan2f(2.0k*(q.r*q.k + q.i*q.j), q.r*q.r + q.i*q.i - q.j*q.j - q.k*q.k) * (float) (180 / M_PI);
  *pitch = asinf(gx) * (float) (180 / M_PI); //Pitch seems to be inverted
  *roll = atan2f(gy, gz) * (float) (180 / M_PI);
}

fix_t sensfusion6GetAccZWithoutGravity(const fix_t ax, const fix_t ay, const fix_t az)
{
  fix_t gx, gy, gz; // estimated gravity direction

  gx = 2.0k * (q.i*q.k - q.r*q.j);
  gy = 2.0k * (q.r*q.i + q.j*q.k);
  gz = q.r*q.r - q.i*q.i - q.j*q.j + q.k*q.k;

  // return vertical acceleration without gravity
  // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  return ((ax*gx + ay*gy + az*gz) - 1.0k);
}



PARAM_GROUP_START(sensorfusion6)
PARAM_ADD(PARAM_FIX, kp, &kp)
PARAM_ADD(PARAM_FIX, ki, &ki)
PARAM_GROUP_STOP(sensorfusion6)
