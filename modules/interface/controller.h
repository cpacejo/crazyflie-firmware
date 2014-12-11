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

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdbool.h>
#include "commander.h"
#include "fix.h"


void controllerInit(void);
bool controllerTest(void);

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 */
void controllerCorrectAttitudePID(
       fix_t eulerRollActual, fix_t eulerPitchActual, fix_t eulerYawActual,
       fix_t eulerRollDesired, fix_t eulerPitchDesired, fix_t eulerYawDesired,
       fix_t* rollRateDesired, fix_t* pitchRateDesired, fix_t* yawRateDesired);

/**
 * Make the controller run an update of the rate PID. The output is
 * the actuator force.
 */
void controllerCorrectRatePID(
       fix_t rollRateActual, fix_t pitchRateActual, fix_t yawRateActual,
       fix_t rollRateDesired, fix_t pitchRateDesired, fix_t yawRateDesired);

/**
 * Reset controller roll, pitch and yaw PID's.
 */
void controllerResetAllPID(void);

/**
 * Get the actuator output.
 */
void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);


#endif /* CONTROLLER_H_ */
