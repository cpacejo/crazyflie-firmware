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
 * quaternion.c - Quaternion functions
 */

#include "fix.h"
#include "quaternion.h"


// below, note aliasing and restrict!!

void qUnit(quaternion_t *const out)
{
  out->r = 1.0k;
  out->i = 0.0k;
  out->j = 0.0k;
  out->k = 0.0k;
}

// represent a vector as a quaternion
void qVec(const fix_t vx, const fix_t vy, const fix_t vz,
          quaternion_t *const out)
{
  out->r = 0.0k;
  out->i = vx;
  out->j = vy;
  out->k = vz;
}

// represent an Euler rotation as a unit quaternion
void qComp(const fix_t angle, const fix_t ux, const fix_t uy, const fix_t uz,
           quaternion_t *const out)
{
  fix_t s, c;
  sincosfix(0.5k * angle, &s, &c);
  out->r = c;
  out->i = s * ux;
  out->j = s * uy;
  out->k = s * uz;
}

void qNeg(const quaternion_t *const x, quaternion_t *const out)
{
  out->r = -x->r;
  out->i = -x->i;
  out->j = -x->j;
  out->k = -x->k;
}

void qConj(const quaternion_t *const x, quaternion_t *const out)
{
  out->r = x->r;
  out->i = -x->i;
  out->j = -x->j;
  out->k = -x->k;
}

fix_t qNorm2(const quaternion_t *const x)
{
  return x->r * x->r + x->i * x->i + x->j * x->j + x->k * x->k;
}

void qScale(const fix_t x, const quaternion_t *const y,
            quaternion_t *const out)
{
  out->r = x * y->r;
  out->i = x * y->i;
  out->j = x * y->j;
  out->k = x * y->k;
}

// note that qInv(unit) is equivalent to the (faster) qConj(unit)
void qInv(const quaternion_t *const x, quaternion_t *const out)
{
  const fix_t n2 = qNorm2(x);
  qConj(x, out);
  qScale(1.0k / n2, out, out);
}

void qUnitize(const quaternion_t *const x, quaternion_t *const out)
{
  qScale(invsqrtfix(qNorm2(x)), x, out);
}

void qMul(const quaternion_t *restrict const x,
          const quaternion_t *restrict const y,
          quaternion_t *restrict const out)
{
  out->r = x->r * y->r - x->i * y->i - x->j * y->j - x->k * y->k;
  out->i = x->r * y->i + x->i * y->r + x->j * y->k - x->k * y->j;
  out->j = x->r * y->j + x->j * y->r + x->k * y->i - x->i * y->k;
  out->k = x->r * y->k + x->k * y->r + x->i * y->j - x->j * y->i;
}

// rotate a vector by the given unit quaternion
void qRot(const fix_t vx, const fix_t vy, const fix_t vz,
          const quaternion_t *restrict const x,
          fix_t *restrict const out_vx,
          fix_t *restrict const out_vy,
          fix_t *restrict const out_vz)
{
  *out_vx =
    2.0k * (vx * (x->r * x->r + x->i * x->i - 0.5k) +
            vy * (x->i * x->j - x->r * x->k) +
            vz * (x->r * x->j + x->i * x->k));

  *out_vy =
    2.0k * (vy * (x->r * x->r + x->j * x->j - 0.5k) +
            vz * (x->j * x->k - x->r * x->i) +
            vx * (x->r * x->k + x->i * x->j));

  *out_vz =
    2.0k * (vz * (x->r * x->r + x->k * x->k - 0.5k) +
            vx * (x->i * x->k - x->r * x->j) +
            vy * (x->r * x->i + x->j * x->k));
}

#if 0
// decompose as infinitesmal roll, pitch & yaw rates
// equivalent to twice the natural logarithm
// assumes unit quaternion
// TODO: go short way; fix asin
void qToRPYRate(const quaternion_t *restrict const x,
                              const fix_t scale,
                              fix_t *restrict const dRoll,
                              fix_t *restrict const dPitch,
                              fix_t *restrict const dYaw)
{
  const fix_t umag = sqrtf(1.0f - x->r * x->r);

  fix_t asx;
  // the limit of 0.03 keeps the error on the sinc just below 1/2 ULP
  if (fabsf(umag) < 0.03f) {
    // too close to singularity; compute manually to keep stable
    // 3 is 3! (from expansion of sin(x)/x) divided by 2
    asx = 2.0f + (umag * umag) * (1.0f / 3.0f);
  }
  else
    asx = 2.0f * asinf(umag) / umag;

  asx *= scale;

  *dPitch = asx * x->i;
  *dRoll = asx * x->j;
  *dYaw = asx * x->k;
}
#endif

// where would we end up if we kept rotating at this rate for one time unit?
// equivalent to the square root of the exponent
void qFromRPYRate(const fix_t dRoll, const fix_t dPitch,
                  const fix_t dYaw, const fix_t scale,
                  quaternion_t *const out)
{
  const fix_t mag2 = dRoll * dRoll + dPitch * dPitch + dYaw * dYaw;
  const fix_t angle2 = (scale * scale) * mag2;
  fix_t c, sx;  // cos(s*x/2), sin(s*x/2)/x

  // the limit of 0.05^2 keeps the errors below 1/2 ULP
  if (fabsfix(mag2) < 0.0025k) {
    // too close to singularity; compute manually to keep stable
    // (also for speed!)

    // 8 is 2! (from expansion of cos(x)) times 4 (0.5 * angle squared)
    c = 1.0k - angle2 * (1.0k / 8.0k);

    // 48 is 3! (from expansion of sin(x)/x) times 4 (0.5 * angle squared)
    // times 2 (to convert sinc(s*x/2)=sin(s*x/2)/(s*x/2) to sin(s*x/2)/x)
    sx = scale * (0.5k - angle2 * (1.0k / 48.0k));
  }
  else {
    const fix_t invMag = invsqrtfix(mag2);
    const fix_t angle = scale * (mag2 * invMag);
    sincosfix(0.5k * angle, &sx, &c);
    sx *= invMag;
  }

  out->r = c;
  out->i = sx * dPitch;
  out->j = sx * dRoll;
  out->k = sx * dYaw;
}

#undef restrict
