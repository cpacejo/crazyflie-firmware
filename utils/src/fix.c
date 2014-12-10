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
  const fix_t x2 = x * x;
  return x * (1.0k - (x2 * (1.0k - x2 * (1.0k / 20.0k))) * (1.0k / 6.0k));
}

static fix_t cosfix_8(const fix_t x)
{
  const fix_t x2 = x * x;
  return 1.0k - (x2 * (1.0k - x2 * (1.0k / 12.0k))) * (1.0k / 2.0k);
}

// sin(x - pi/4)
static fix_t sin4fix_8(const fix_t x)
{
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
