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
 * fix.h - s16.15 fixed point handling functions
 */

#ifndef __FIX_H
#define __FIX_H

#include <stdfix.h>
#include <math.h>  // for constants

typedef accum fix_t;  // suffix is k
typedef sat accum sat_fix_t;

#define FIX_MIN (-0xFFFF.FFFFk)
#define FIX_MAX 0xFFFF.FFFFk
#define FIX_IBIT 16
#define FIX_FBIT 15
#define SIZEOF_FIX 4

// saturation conversion to fixpoint
#define FIX(x) ((fix_t) (sat accum) (x))

#define M_SQRT2_FIX ((fix_t) M_SQRT2)
#define M_SQRT1_2_FIX ((fix_t) M_SQRT1_2)

#define M_TWO_PI_FIX ((fix_t) (2.0 * M_PI))
#define M_PI_FIX ((fix_t) M_PI)
#define M_PI_2_FIX ((fix_t) M_PI_2)
#define M_PI_3_FIX ((fix_t) (M_PI / 3.0))
#define M_PI_4_FIX ((fix_t) M_PI_4)
#define M_PI_6_FIX ((fix_t) (M_PI_2 / 3.0))
#define M_PI_8_FIX ((fix_t) (M_PI_4 / 2.0))

fix_t fabsfix(fix_t x);
fix_t constrainfix(fix_t x, fix_t min, fix_t max);

fix_t sqrtfix(fix_t x);
fix_t invsqrtfix(fix_t x);

int floorLog2fix(fix_t x);
fix_t sinfix(fix_t x);
fix_t cosfix(fix_t x);
void sincosfix(fix_t x, fix_t *restrict s, fix_t *restrict c);

fix_t asinfix(fix_t x);
fix_t acosfix(fix_t x);
fix_t atan2fix(fix_t y, fix_t x);

#endif
