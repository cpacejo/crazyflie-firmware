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
 * vec3.h - 3-vectors
 */

#ifndef __VEC3_H
#define __VEC3_H

#include "fix.h"
#include "imu_types.h"  // temporary

typedef Axis3f vec3_t;  // temporary

#define VEC3_ZERO_INIT { .x = 0.0k, .y = 0.0k, .z = 0.0k }

#define VEC3_UNIT_X_INIT { .x = 1.0k, .y = 0.0k, .z = 0.0k }
#define VEC3_UNIT_Y_INIT { .x = 0.0k, .y = 1.0k, .z = 0.0k }
#define VEC3_UNIT_Z_INIT { .x = 0.0k, .y = 0.0k, .z = 1.0k }

#define VEC3_UNIT_NEGX_INIT { .x = -1.0k, .y = 0.0k, .z = 0.0k }
#define VEC3_UNIT_NEGY_INIT { .x = 0.0k, .y = -1.0k, .z = 0.0k }
#define VEC3_UNIT_NEGZ_INIT { .x = 0.0k, .y = 0.0k, .z = -1.0k }

fix_t vec3Norm2(const vec3_t *x);
void vec3Neg(const vec3_t *x, vec3_t *out);
void vec3Scale(fix_t x, const vec3_t *y, vec3_t *out);
void vec3Add(const vec3_t *x, const vec3_t *y, vec3_t *out);
void vec3ScaleAdd(fix_t x, const vec3_t *y, const vec3_t *z, vec3_t *out);
fix_t vec3Dot(const vec3_t *x, const vec3_t *y);
void vec3Cross(const vec3_t *x, const vec3_t *y, vec3_t *out);

#endif
