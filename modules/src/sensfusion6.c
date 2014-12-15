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
#include "vec3.h"

#include "sensfusion6.h"
#include "imu.h"
#include "param.h"

#define KP_DEF  1.0k // proportional gain
#define KI_DEF  0.001k // integral gain

static fix_t kp = KP_DEF;    // proportional gain (Kp)
static fix_t ki = KI_DEF;    // integral gain (Ki)
static vec3_t integralFB = VEC3_ZERO_INIT;  // integral error terms scaled by Ki


static quaternion_t q = Q_UNIT_INIT;  // quaternion of sensor frame relative to auxiliary frame
static vec3_t qRate = VEC3_ZERO_INIT;  // quaternion rate of change

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
// 12/12/2014 C Pacejo  rewrote using quaternion library
void sensfusion6UpdateQ(const vec3_t *const g, const vec3_t *const a, const fix_t dt)
{
  vec3_t qRateTemp = *g;
  const fix_t aNorm2 = vec3Norm2(a);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (aNorm2 >= 0.01k)
  {
    // Normalise accelerometer measurement
    vec3_t aUnit;
    vec3Scale(invsqrtfix(aNorm2), a, &aUnit);

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    vec3_t v;
    quaternion_t qc;
    qConj(&q, &qc);
    qRotZ(&qc, &v);
    vec3Neg(&v, &v);

    // Compute rate needed to rotate to desired position
    quaternion_t qDiff;
    qVecDiff(&v, &aUnit, &qDiff);
    vec3_t e;
    qDelta0(&qDiff, 1.0k, &e);

    // Compute and apply integral feedback
    vec3ScaleAdd(dt, &e, &integralFB, &integralFB);  // integral error scaled by dt
    vec3ScaleAdd(ki, &integralFB, &qRateTemp, &qRateTemp);  // apply integral feedback

    // Apply proportional feedback
    vec3ScaleAdd(kp, &e, &qRateTemp, &qRateTemp);
  }

  // Integrate rate of change of quaternion
  quaternion_t qTemp;
  qInteg0(&qRateTemp, dt, &qTemp);
  // multiply "backward"; we want to apply rotation *locally*
  qMul(&q, &qTemp, &qTemp);

  // Normalise quaternion
  qUnitize(&qTemp, &q);
  qRate = qRateTemp;
}

void sensfusion6GetQ(quaternion_t *const qOut)
{
  *qOut = q;
}

void sensfusion6GetQRate(vec3_t *const qRateOut)
{
  *qRateOut = qRate;
}

fix_t sensfusion6GetAccZWithoutGravity(const vec3_t *const a)
{
  // rotate acceleration relative to auxiliary frame
  vec3_t g;
  qRot(a, &q, &g);

  // return just z portion, sans gravity
  return g.z + 1.0k;
}



PARAM_GROUP_START(sensorfusion6)
PARAM_ADD(PARAM_FIX, kp, &kp)
PARAM_ADD(PARAM_FIX, ki, &ki)
PARAM_GROUP_STOP(sensorfusion6)
