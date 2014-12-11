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
 * vec3.c - 3-vectors
 */

#include "fix.h"
#include "vec3.h"

fix_t vec3Norm2(const vec3_t *const x)
{
  return x->x * x->x + x->y * x->y + x->z * x->z;
}

void vec3Neg(const vec3_t *const x, vec3_t *const out)
{
  out->x = -x->x;
  out->y = -x->y;
  out->z = -x->z;
}

void vec3Scale(const fix_t x, const vec3_t *const y, vec3_t *const out)
{
  out->x = x * y->x;
  out->y = x * y->y;
  out->z = x * y->z;
}

fix_t vec3Dot(const vec3_t *const x, const vec3_t *const y)
{
  return x->x * y->x + x->y * y->y + x->z * y->z;
}

void vec3Cross(const vec3_t *const x, const vec3_t *const y, vec3_t *const out)
{
  const fix_t out_x = x->y * y->z - x->z * y->x;
  const fix_t out_y = x->z * y->x - x->x * y->z;
  const fix_t out_z = x->x * y->y - x->y * y->x;

  out->x = out_x;
  out->y = out_y;
  out->z = out_z;
}
