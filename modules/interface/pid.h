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
 *
 * pid.h - implementation of the PID regulator
 */
#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include "fix.h"

#define PID_ROLL_RATE_KP  70.0k
#define PID_ROLL_RATE_KI  0.0k
#define PID_ROLL_RATE_KD  0.0k
#define PID_ROLL_RATE_INTEGRATION_LIMIT    100.0k

#define PID_PITCH_RATE_KP  70.0k
#define PID_PITCH_RATE_KI  0.0k
#define PID_PITCH_RATE_KD  0.0k
#define PID_PITCH_RATE_INTEGRATION_LIMIT   100.0k

#define PID_YAW_RATE_KP  50.0k
#define PID_YAW_RATE_KI  0.0k
#define PID_YAW_RATE_KD  0.0k
#define PID_YAW_RATE_INTEGRATION_LIMIT     500.0k

#define PID_ROLL_KP  3.5k
#define PID_ROLL_KI  0.0k
#define PID_ROLL_KD  0.0k
#define PID_ROLL_INTEGRATION_LIMIT    20.0k
#define PID_ROLL_MAX_SLEW  300.0k

#define PID_PITCH_KP  3.5k
#define PID_PITCH_KI  0.0k
#define PID_PITCH_KD  0.0k
#define PID_PITCH_INTEGRATION_LIMIT   20.0k
#define PID_PITCH_MAX_SLEW  300.0k

#define PID_YAW_KP  12.0k
#define PID_YAW_KI  0.0k
#define PID_YAW_KD  0.0k
#define PID_YAW_INTEGRATION_LIMIT     360.0k
#define PID_YAW_MAX_SLEW  1000.0k


#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0k

typedef struct
{
  fix_t desired;     //< set point
  fix_t error;        //< error
  fix_t prevError;    //< previous error
  fix_t integ;        //< integral
  fix_t deriv;        //< derivative
  fix_t kp;           //< proportional gain
  fix_t ki;           //< integral gain
  fix_t kd;           //< derivative gain
  fix_t outP;         //< proportional output (debugging)
  fix_t outI;         //< integral output (debugging)
  fix_t outD;         //< derivative output (debugging)
  fix_t iLimit;       //< integral limit
  fix_t dt;           //< delta-time dt
  fix_t idt;          //< inverse dt
} PidObject;

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 */
void pidInit(PidObject* pid, const fix_t desired, const fix_t kp,
             const fix_t ki, const fix_t kd, const fix_t dt);

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidSetIntegralLimit(PidObject* pid, const fix_t limit);

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidReset(PidObject* pid);

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 */
void pidUpdate(PidObject* pid, const fix_t measured, const bool updateError);

/**
 * Update the PID parameters for a rotational PV.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured angle
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 */
void pidUpdate360(PidObject* pid, const fix_t measured, const bool updateError);

/**
 * Return the PID output.
 *
 * @param[in] pid         A pointer to the pid object.
 * @return PID algorithm output
 */
fix_t pidGetOutput(PidObject* pid);

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void pidSetDesired(PidObject* pid, const fix_t desired);

/**
 * Set a new set point for the PID to track.
 * @return The set point
 */
fix_t pidGetDesired(PidObject* pid);

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool pidIsActive(PidObject* pid);

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void pidSetError(PidObject* pid, const fix_t error);

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void pidSetKp(PidObject* pid, const fix_t kp);

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void pidSetKi(PidObject* pid, const fix_t ki);

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void pidSetKd(PidObject* pid, const fix_t kd);

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void pidSetDt(PidObject* pid, const fix_t dt);
#endif /* PID_H_ */
