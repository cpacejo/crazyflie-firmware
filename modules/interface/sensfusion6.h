/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
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
 */

#ifndef SENSORFUSION6_H_
#define SENSORFUSION6_H_
#include <stdbool.h>
#include "fix.h"
#include "vec3.h"
#include "quaternion.h"

void sensfusion6Init(void);
bool sensfusion6Test(void);

// g: angular velocity in radians; a: acceleration vector
void sensfusion6UpdateQ(const vec3_t *g, const vec3_t *a, fix_t dt);
void sensfusion6GetQ(quaternion_t *qOut);
void sensfusion6GetQRate(vec3_t *qRateOut);
fix_t sensfusion6GetAccZWithoutGravity(const vec3_t *a);


#endif /* SENSORFUSION6_H_ */
