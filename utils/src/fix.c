/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 Bitcraze AB
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
 * fix.c - s16.15 fixed point handling functions
 */

#include <stdfix.h>
#include "fix.h"

#if ACCUM_IBIT < 16
#error This platform does not support enough (>= 16) fixpoint integer bits!
#endif

#if ACCUM_FBIT < 15
#error This platform does not support enough (>= 15) fixpoint fraction bits!
#endif


//
// ARITHMETIC
//

fix_t fabsfix(const fix_t x)
{
  if (x < 0.0k) return -x;
  else return x;
}

fix_t constrainfix(const fix_t x, const fix_t min, const fix_t max)
{
  if (x < min) return min;
  if (x > max) return max;
  return x;
}


//
// ALGEBRAIC
//

fix_t invsqrtfix(const fix_t x)
{
  if (x <= 0.0k) return 0.0k;

  fix_t res;

  // compute res = 2 ** (0.5 * round(log2(x)))
  const int log_2 = floorLog2fix(x * M_SQRT2_FIX);
  if (log_2 < 0)
    res = (((-log_2) & 1) == 0 ? 1.0k : M_SQRT2_FIX) << ((-log_2) >> 1);
  else
    res = ((log_2 & 1) == 0 ? 1.0k : M_SQRT1_2_FIX) >> (log_2 >> 1);

  // Newton's method
  const fix_t half_x = 0.5k * x;
  for (unsigned int i = 0; i < 4; i++) {
    const fix_t old_res = res;
    res *= 1.5k - half_x * res * res;
    if (res == old_res) break;
  }

  return res;
}

fix_t sqrtfix(const fix_t x)
{
  return x * invsqrtfix(x);
}


//
// TRANSCENDENTAL
//

int floorLog2fix(const fix_t x)
{
  // we make a lot of assumptions here, regarding the fix-point format
#if SIZEOF_FIX == __SIZEOF_INT__
  const int lz = __builtin_clz(*(unsigned int *) &x);
#elif SIZEOF_FIX == __SIZEOF_LONG__
  const int lz = __builtin_clzl(*(unsigned int *) &x);
#elif SIZEOF_FIX == __SIZEOF_LONG_LONG__
  const int lz = __builtin_clzll(*(unsigned int *) &x);
#else
#error
#endif

  return ((8 * SIZEOF_FIX) - FIX_FBIT - 1) - lz;
}

// these assume fabsfix(x) <= M_PI_8_FIX
static fix_t sinfix_8(const fix_t x)
{
  // Taylor series order 5
  const fix_t x2 = x * x;
  return x * (1.0k - (x2 * (1.0k - x2 * (1.0k / 20.0k))) * (1.0k / 6.0k));
}

static fix_t cosfix_8(const fix_t x)
{
  // Taylor series order 4
  const fix_t x2 = x * x;
  return 1.0k - (x2 * (1.0k - x2 * (1.0k / 12.0k))) * (1.0k / 2.0k);
}

// sin(x - pi/4)
static fix_t sin4fix_8(const fix_t x)
{
  // Taylor series order 4 about pi/4
  return M_SQRT1_2_FIX * (1.0k + x * (1.0k - (x * (1.0k + (x * (1.0k - x * (1.0k / 4.0k))) * (1.0k / 3.0k))) * (1.0k / 2.0k)));
}

fix_t sinfix(fix_t x)
{
  while (x > M_PI_FIX) x -= M_TWO_PI_FIX;
  while (x < -M_PI_FIX) x += M_TWO_PI_FIX;

  if (x >= -M_PI_8_FIX)
  {
    if (x <= M_PI_8_FIX) return sinfix_8(x);
    else if (x <= M_PI_4_FIX + M_PI_8_FIX) return sin4fix_8(x - M_PI_4_FIX);
    else if (x <= M_PI_2_FIX + M_PI_8_FIX) return cosfix_8(x - M_PI_2_FIX);
    else if (x <= M_PI_FIX - M_PI_8_FIX) return sin4fix_8((M_PI_2_FIX + M_PI_4_FIX) - x);
    else return sinfix_8(M_PI_FIX - x);
  }
  else if (x >= -M_PI_4_FIX - M_PI_8_FIX) return -sin4fix_8(-x - M_PI_4_FIX);
  else if (x >= -M_PI_2_FIX - M_PI_8_FIX) return -cosfix_8(-x - M_PI_2_FIX);
  else if (x >= -M_PI_FIX + M_PI_8_FIX) return -sin4fix_8(x + (M_PI_2_FIX + M_PI_4_FIX));
  else return -sinfix_8(x + M_PI_FIX);
}

fix_t cosfix(fix_t x)
{
  while (x > M_PI_FIX) x -= M_TWO_PI_FIX;
  while (x < -M_PI_FIX) x += M_TWO_PI_FIX;

  if (x >= -M_PI_8_FIX)
  {
    if (x <= M_PI_8_FIX) return cosfix_8(x);
    else if (x <= M_PI_4_FIX + M_PI_8_FIX) return sin4fix_8(M_PI_4_FIX - x);
    else if (x <= M_PI_2_FIX + M_PI_8_FIX) return sinfix_8(M_PI_2_FIX - x);
    else if (x <= M_PI_FIX - M_PI_8_FIX) return -sin4fix_8(x - (M_PI_2_FIX + M_PI_4_FIX));
    else return -cosfix_8(x - M_PI_FIX);
  }
  else if (x >= -M_PI_4_FIX - M_PI_8_FIX) return sin4fix_8(M_PI_4_FIX + x);
  else if (x >= -M_PI_2_FIX - M_PI_8_FIX) return sinfix_8(M_PI_2_FIX + x);
  else if (x >= -M_PI_FIX + M_PI_8_FIX) return -sin4fix_8(-x - (M_PI_2_FIX + M_PI_4_FIX));
  else return -cosfix_8(-x - M_PI_FIX);
}

void sincosfix(fix_t x, fix_t *const restrict s, fix_t *const restrict c)
{
  *s = sinfix(x);
  *c = cosfix(x);
}

// assumes |x| <= 0.5
static fix_t atanfix_aux(const fix_t x)
{
  // Pade' approximation order (3,4)
  const fix_t x2 = x * x;
  return (x * (105.0k + x2 * 55.0k)) / (105.0k + x2 * (90.0k + x2 * 9.0k));
}

// atan(x + 1); assumes 0.5 <= x + 1 <= 2.0
static fix_t atanfix_aux1(const fix_t x)
{
  // Pade' approximation order (3,3) about 1.0k
  return M_PI_4_FIX +
    (x * (60.0k + x * (60.0k + x * 19.0k))) /
    (120.0k + x * (180.0k + x * (108.0k + x * 24.0k)));
}

fix_t atan2fix(const fix_t y, const fix_t x)
{
  fix_t res;

  if (fabsfix(y) <= fabsfix(0.5k * x))
    res = atanfix_aux(y / x);
  else if (fabsfix(x) <= fabsfix(0.5k * y))
  {
    const fix_t x_y = x / y;
    if (x_y >= 0.0k)
      res = M_PI_2_FIX - atanfix_aux(x_y);
    else
      res = -M_PI_2_FIX - atanfix_aux(x_y);
  }
  else
  {
    const fix_t y_x = y / x;
    if (y_x >= 0.0k)
      res = atanfix_aux1(y_x - 1.0k);
    else
      // use -1.0k * instead of - to avoid stupid GCC internal error
      res = -1.0k * atanfix_aux1(-y_x - 1.0k);
  }

  if (x < 0.0k)
  {
    if (y < 0.0k)
      return res - M_PI_FIX;
    else
      return res + M_PI_FIX;
  }
  else return res;
}

// asin(x - 1); valid for 0 < x < 0.25
static fix_t asinfix_aux(const fix_t x)
{
  // Puiseux series order 4 about -1, plus pi/2
  return sqrtfix(x) * (1.0k + x * (1.0k + x * (1.0k + x * (1.0k + x *
      (49.0k / 144.0k)) * (25.0k / 84.0k)) * (9.0k / 40.0k)) * (1.0k / 12.0k)) * M_SQRT2_FIX;
}

fix_t asinfix(const fix_t x)
{
  // FIXME: this is not perfectly accurate over 0.25 < |x| < 0.75
  // (we'd probably need Pade' approximation); but this is currently
  // only used for display purposes

  if (x < -0.5k)
    return asinfix_aux(x + 1.0k) - M_PI_2_FIX;
  else if (x < 0.5k)
  {
    // Taylor series order 5; valid for |x| < 0.25
    const fix_t x2 = x * x;
    return x * (1.0k + x2 * (1.0k + x2 * (9.0k / 20.0k)) * (1.0k / 6.0k));
  }
  else
  {
    return M_PI_2_FIX + -1.0k * asinfix_aux(1.0k - x);
  }
}

fix_t acosfix(const fix_t x)
{
  return M_PI_2_FIX + -1.0k * asinfix(x);
}
