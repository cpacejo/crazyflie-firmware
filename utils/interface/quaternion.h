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
#include "vec3.h"

typedef struct {
  fix_t r, i, j, k;
} quaternion_t;

#define Q_UNIT_INIT { .r = 1.0k, .i = 0.0k, .j = 0.0k, .k = 0.0k }

void qUnit(quaternion_t *out);
// represent a vector as a quaternion
void qVec(const vec3_t *v, quaternion_t *out);
// represent an Euler rotation as a unit quaternion
// equivalent to qInteg0 for unit vectors
void qComp(fix_t angle, const vec3_t *u, quaternion_t *out);
void qNeg(const quaternion_t *x, quaternion_t *out);
void qConj(const quaternion_t *x, quaternion_t *out);

fix_t qNorm2(const quaternion_t *x);
void qScale(fix_t x, const quaternion_t *y, quaternion_t *out);

// note that qInv(unit) is equivalent to the (faster) qConj(unit)
void qInv(const quaternion_t *x, quaternion_t *out);
void qUnitize(const quaternion_t *x, quaternion_t *out);
void qMul(const quaternion_t *global, const quaternion_t *local, quaternion_t *out);
// rotate a vector by the given unit quaternion
void qRot(const vec3_t *v, const quaternion_t *x, vec3_t *out);
void qRotX(const quaternion_t *x, vec3_t *out);
void qRotY(const quaternion_t *x, vec3_t *out);
void qRotZ(const quaternion_t *x, vec3_t *out);
// what is the angle by which the given vector will be rotated?
// equivalent to vec3Dot(v, qRot(v, x))
fix_t qCosAngleX(const quaternion_t *x);
fix_t qCosAngleY(const quaternion_t *x);
fix_t qCosAngleZ(const quaternion_t *x);

// by what rate should we rotate to reach final from initial in a time of dt?
// equivalent to twice the natural logarithm
// assumes unit quaternion
void qDelta(const quaternion_t *final, const quaternion_t *initial,
            fix_t dt, vec3_t *delta);

void qDelta0(const quaternion_t *final, fix_t dt, vec3_t *delta);

// where would we end up if we kept rotating at this rate for a time of dt?
// equivalent to the square root of the exponent
void qInteg(const quaternion_t *initial,
            const vec3_t *rate, fix_t dt,
            quaternion_t *final);

void qInteg0(const vec3_t *rate, fix_t dt, quaternion_t *final);

#endif //QUATERNION_H_
