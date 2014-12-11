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

void sensfusion6Init(void);
bool sensfusion6Test(void);

void sensfusion6UpdateQ(fix_t gx, fix_t gy, fix_t gz, fix_t ax, fix_t ay, fix_t az, fix_t dt);
void sensfusion6GetEulerRPY(fix_t* roll, fix_t* pitch, fix_t* yaw);
fix_t sensfusion6GetAccZWithoutGravity(const fix_t ax, const fix_t ay, const fix_t az);


#endif /* SENSORFUSION6_H_ */
