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
#include "vec3.h"
#include "quaternion.h"


// below, note aliasing!!

void qUnit(quaternion_t *const out)
{
  out->r = 1.0k;
  out->ijk = (vec3_t) VEC3_ZERO_INIT;
}

// represent a vector as a quaternion
void qVec(const vec3_t *const v, quaternion_t *const out)
{
  out->r = 0.0k;
  out->ijk = *v;
}

// represent an Euler rotation as a unit quaternion
void qComp(const fix_t angle, const vec3_t *const u, quaternion_t *const out)
{
  fix_t s, c;
  sincosfix(0.5k * angle, &s, &c);
  out->r = c;
  vec3Scale(s, u, &out->ijk);
}

void qNeg(const quaternion_t *const x, quaternion_t *const out)
{
  out->r = -x->r;
  vec3Neg(&x->ijk, &out->ijk);
}

void qConj(const quaternion_t *const x, quaternion_t *const out)
{
  out->r = x->r;
  vec3Neg(&x->ijk, &out->ijk);
}

fix_t qNorm2(const quaternion_t *const x)
{
  return x->r * x->r + vec3Norm2(&x->ijk);
}

void qScale(const fix_t x, const quaternion_t *const y,
            quaternion_t *const out)
{
  out->r = x * y->r;
  vec3Scale(x, &y->ijk, &out->ijk);
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

void qMul(const quaternion_t *x, const quaternion_t *y, quaternion_t *out)
{
  const fix_t out_r = x->r * y->r - x->i * y->i - x->j * y->j - x->k * y->k;
  const fix_t out_i = x->r * y->i + x->i * y->r + x->j * y->k - x->k * y->j;
  const fix_t out_j = x->r * y->j + x->j * y->r + x->k * y->i - x->i * y->k;
  const fix_t out_k = x->r * y->k + x->k * y->r + x->i * y->j - x->j * y->i;

  out->r = out_r;
  out->i = out_i;
  out->j = out_j;
  out->k = out_k;
}

void qRot(const vec3_t *const v, const quaternion_t *const x, vec3_t *const out)
{
  const fix_t out_x =
    2.0k * (v->x * (x->r * x->r + x->i * x->i - 0.5k) +
            v->y * (x->i * x->j - x->r * x->k) +
            v->z * (x->r * x->j + x->i * x->k));

  const fix_t out_y =
    2.0k * (v->y * (x->r * x->r + x->j * x->j - 0.5k) +
            v->z * (x->j * x->k - x->r * x->i) +
            v->x * (x->r * x->k + x->i * x->j));

  const fix_t out_z =
    2.0k * (v->z * (x->r * x->r + x->k * x->k - 0.5k) +
            v->x * (x->i * x->k - x->r * x->j) +
            v->y * (x->r * x->i + x->j * x->k));

  out->x = out_x;
  out->y = out_y;
  out->z = out_z;
}

void qRotX(const quaternion_t *const x, vec3_t *const out)
{
  const fix_t out_x = 2.0k * (x->r * x->r + x->i * x->i - 0.5k);
  const fix_t out_y = 2.0k * (x->r * x->k + x->i * x->j);
  const fix_t out_z = 2.0k * (x->i * x->k - x->r * x->j);

  out->x = out_x;
  out->y = out_y;
  out->z = out_z;
}

void qRotY(const quaternion_t *const x, vec3_t *const out)
{
  const fix_t out_x = 2.0k * (x->i * x->j - x->r * x->k);
  const fix_t out_y = 2.0k * (x->r * x->r + x->j * x->j - 0.5k);
  const fix_t out_z = 2.0k * (x->r * x->i + x->j * x->k);

  out->x = out_x;
  out->y = out_y;
  out->z = out_z;
}

void qRotZ(const quaternion_t *const x, vec3_t *const out)
{
  const fix_t out_x = 2.0k * (x->r * x->j + x->i * x->k);
  const fix_t out_y = 2.0k * (x->j * x->k - x->r * x->i);
  const fix_t out_z = 2.0k * (x->r * x->r + x->k * x->k - 0.5k);

  out->x = out_x;
  out->y = out_y;
  out->z = out_z;
}

fix_t qCosAngleX(const quaternion_t *const x)
{
  return 2.0k * (x->r * x->r + x->i * x->i - 0.5k);
}

fix_t qCosAngleY(const quaternion_t *const x)
{
  return 2.0k * (x->r * x->r + x->j * x->j - 0.5k);
}

fix_t qCosAngleZ(const quaternion_t *const x)
{
  return 2.0k * (x->r * x->r + x->k * x->k - 0.5k);
}

void qDelta0(const quaternion_t *const final, const fix_t dt, vec3_t *const delta)
{
  // find the sine of (half) the angle; let the vector take care of sign
  // note: don't use 1.0k - final->r * final->r; this is inaccurate for small angles
  const fix_t umag2 = final->i * final->i + final->j * final->j + final->k * final->k;
  const fix_t umagInv = invsqrtfix(umag2);
  const fix_t umag = umagInv * umag2;

  fix_t asx;
  // the limit of 0.03 keeps the error on the sinc just below 1/2 ULP
  if (fabsfix(umag) < 0.03k) {
    // too close to singularity; compute manually to keep stable
    // we can just use asin here (the x component is very close to 1
    // and thus has no slope)
    // 3 is 3! (from expansion of sin(x)/x) divided by 2
    asx = dt * (2.0k + umag2 * (1.0k / 3.0k));
  }
  else {
    // use atan2 (to ensure we don't have 0 or inf slope)
    // but we can manually divide out the vector magnitude
    // note: use abs value of R to ensure minimal rotation
    // (negation doesn't change quaternion)
    asx = dt * 2.0k * atan2fix(umag, fabsfix(final->r)) * umagInv;
  }

  // negate the quaternion if necessary to match positive R
  vec3Scale(final->r < 0.0k ? -asx : asx, &final->ijk, delta);
}

void qInteg0(const vec3_t *const delta, const fix_t dt,
             quaternion_t *const final)
{
  const fix_t mag2 = vec3Norm2(delta);
  const fix_t angle2 = (dt * dt) * mag2;
  fix_t c, sx;  // cos(s*x/2), sin(s*x/2)/x

  // the limit of 0.05^2 keeps the errors below 1/2 ULP
  if (fabsfix(mag2) < 0.0025k) {
    // too close to singularity; compute manually to keep stable
    // (also for speed!)

    // 8 is 2! (from expansion of cos(x)) times 4 (0.5 * angle squared)
    c = 1.0k - angle2 * (1.0k / 8.0k);

    // 48 is 3! (from expansion of sin(x)/x) times 4 (0.5 * angle squared)
    // times 2 (to convert sinc(s*x/2)=sin(s*x/2)/(s*x/2) to sin(s*x/2)/x)
    sx = dt * (0.5k - angle2 * (1.0k / 48.0k));
  }
  else {
    const fix_t invMag = invsqrtfix(mag2);
    const fix_t angle = dt * (mag2 * invMag);
    sincosfix(0.5k * angle, &sx, &c);
    sx *= invMag;
  }

  final->r = c;
  vec3Scale(sx, delta, &final->ijk);
}

void qToRPY(const quaternion_t *const x, vec3_t *const rpy)
{
  rpy->x = atan2fix(x->r * x->i + x->j * x->k, 0.5k - (x->i * x->i + x->j * x->j));
  rpy->y = asinfix(2.0k * (x->r * x->j - x->k * x->i));
  rpy->z = atan2fix(x->r * x->k + x->i * x->j, 0.5k - (x->j * x->j + x->k * x->k));
}

void qVecDiff(const vec3_t *const to, const vec3_t *const from, quaternion_t *const x)
{
  quaternion_t temp;
  temp.r = vec3Dot(from, to) + 1.0k;
  vec3Cross(from, to, &temp.ijk);
  qUnitize(&temp, x);
}
