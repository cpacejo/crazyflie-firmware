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
 * quaternion.h - Quaternion functions
 */
#ifndef QUATERNION_H_
#define QUATERNION_H_
#include "fix.h"

typedef struct {
  fix_t r, i, j, k;
} quaternion_t;

#define Q_UNIT_INIT { .r = 1.0k, .i = 0.0k, .j = 0.0k, .k = 0.0k }

// below, note aliasing and restrict!!

void qUnit(quaternion_t *out);
// represent a vector as a quaternion
void qVec(fix_t vx, fix_t vy, fix_t vz, quaternion_t *out);
// represent an Euler rotation as a unit quaternion
void qComp(fix_t angle, fix_t ux, fix_t uy, fix_t uz, quaternion_t *out);
void qNeg(const quaternion_t *x, quaternion_t *out);
void qConj(const quaternion_t *x, quaternion_t *out);

fix_t qNorm2(const quaternion_t *x);
void qScale(fix_t x, const quaternion_t *y, quaternion_t *out);

// note that qInv(unit) is equivalent to the (faster) qConj(unit)
void qInv(const quaternion_t *x, quaternion_t *out);
void qUnitize(const quaternion_t *x, quaternion_t *out);
void qMul(const quaternion_t *restrict x, const quaternion_t *restrict y,
          quaternion_t *restrict out);
// rotate a vector by the given unit quaternion
void qRot(fix_t vx, fix_t vy, fix_t vz,
          const quaternion_t *restrict x, fix_t *restrict out_vx,
          fix_t *restrict out_vy, fix_t *restrict out_vz);

// decompose as infinitesmal roll, pitch & yaw rates
// equivalent to twice the natural logarithm
// assumes unit quaternion
void qToRPYRate(const quaternion_t *restrict x,
                fix_t scale, fix_t *restrict dRoll,
                fix_t *restrict dPitch, fix_t *restrict dYaw);

// where would we end up if we kept rotating at this rate for one time unit?
// equivalent to the square root of the exponent
void qFromRPYRate(fix_t dRoll, fix_t dPitch,
                  fix_t dYaw, fix_t scale,
                  quaternion_t *out);

#endif //QUATERNION_H_
