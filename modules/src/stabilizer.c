/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 */
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"
#include "fix.h"
#include "vec3.h"
#include "quaternion.h"

#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
#include "ms5611.h"

#undef max
#define max(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#undef min
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#undef abs
#define abs(x) ({ __typeof__ (x) _x = (x); _x < 0 ? -_x : _x; })

static fix_t wrapAngle(fix_t angle)
{
  while (angle >= 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (fix_t)(1.0k / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

#define ACTUATOR_THRUST_SCALE 65535.0k

static vec3_t gyro; // Gyro axis data in rad/s
static vec3_t acc;  // Accelerometer axis data in mG
static vec3_t mag;  // Magnetometer axis data in tesla

static quaternion_t qActual = Q_UNIT_INIT;

static fix_t rollDesired = 0.0k;
static fix_t pitchDesired = 0.0k;
static fix_t yawDesired = 0.0k;
static quaternion_t qDesired = Q_UNIT_INIT;

static vec3_t qRateDesired = VEC3_ZERO_INIT;
static vec3_t qRateActual = VEC3_ZERO_INIT;

static vec3_t rpyActual = VEC3_ZERO_INIT;

static fix_t thrustDesired = 0.0k;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
static PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
static bool altHold = false;          // Currently in altitude hold mode
static bool setAltHold = false;      // Hover mode has just been activated
static fix_t accWZ     = 0.0k;
static fix_t accMAG    = 0.0k;
static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;                    // Output of the PID controller
static float altHoldErr;                       // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.91; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust

static RPYType rollType;
static RPYType pitchType;
static RPYType yawType;

static uint16_t actuatorThrust;
static int16_t  actuatorRoll;
static int16_t  actuatorPitch;
static int16_t  actuatorYaw;

static int32_t motorPowerM1;
static int32_t motorPowerM2;
static int32_t motorPowerM3;
static int32_t motorPowerM4;

static bool isInit;

static void stabilizerAltHoldUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static fix_t deadband(fix_t value, const fix_t threshold);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit(IMU_UPDATE_DT, FUSION_UPDATE_DT);

  xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

  isInit = TRUE;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // convention is the "nautical" one: x forward, y right, z down

    // Magnetometer not yet used more than for logging.
    vec3_t gyroTemp, accTemp;
    imu9Read(&gyroTemp, &accTemp, &mag);
    vec3Scale(M_PI_FIX / 180.0k, &gyroTemp, &gyroTemp);
    gyro.x = gyroTemp.x;
    gyro.y = -gyroTemp.y;
    gyro.z = -gyroTemp.z;
    acc.x = accTemp.x;
    acc.y = -accTemp.y;
    acc.z = -accTemp.z;

    if (imu6IsCalibrated())
    {
      fix_t tempRollDesired, tempPitchDesired, tempYawDesired;
      quaternion_t tempQDesired = Q_UNIT_INIT;

      commanderGetRPY(&tempRollDesired, &tempPitchDesired, &tempYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // FIXME: hardcoded as roll/pitch ANGLE and yaw CRATE
      // generalizing this is hard with quaternions!
      rollDesired = tempRollDesired;
      pitchDesired = tempPitchDesired;

      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        yawDesired = wrapAngle(yawDesired + tempYawDesired * FUSION_UPDATE_DT);
      }

      // first tilt to the desired (Cartesian) pitch & roll
      qInteg0(&(vec3_t) {
        .x = rollDesired * M_PI_FIX / 180.0k,
        .y = pitchDesired * M_PI_FIX / 180.0k,
        .z = 0.0k }, 1.0k, &tempQDesired);

      // now rotate to the desired yaw
      quaternion_t yawQ;
      qComp(yawDesired * M_PI_FIX / 180.0k, &(vec3_t) VEC3_UNIT_Z_INIT, &yawQ);
      qMul(&yawQ, &tempQDesired, &tempQDesired);

      qDesired = tempQDesired;

      if (attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(&gyro, &acc, FUSION_UPDATE_DT);
        sensfusion6GetQ(&qActual);
        sensfusion6GetQRate(&qRateActual);
        vec3_t rpyActualRad;
        qToRPY(&qActual, &rpyActualRad);
        vec3Scale(180.0k / M_PI_FIX, &rpyActualRad, &rpyActual);

        accWZ = sensfusion6GetAccZWithoutGravity(&acc);
        accMAG = vec3Norm2(&acc);
        // Estimate speed from acc (drifts)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

        controllerCorrectAttitudePID(&qActual, &qDesired, &qRateDesired);
        attitudeCounter = 0;
      }

      // 100HZ
      #if 0
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
      {
        stabilizerAltHoldUpdate();
        altHoldCounter = 0;
      }
      #endif

      controllerCorrectRatePID(&qRateActual, &qRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      if (!altHold || !imuHasBarometer())
      {
        // Use thrust from controller if not in altitude hold mode
        commanderGetThrust(&thrustDesired);
#if REFERENCE_THRUST_TO_VERTICAL
        const fix_t cosTiltAngle = qCosAngleZ(&qActual);

        if (cosTiltAngle < 0.0k) {
          // oops, we're upside down.
          actuatorThrust = 0;
        }
        else if (cosTiltAngle < 0.5k) {
          // cap at 2x thrust input
          actuatorThrust = (uint16_t) (min(1.0k, thrustDesired * 2.0k) * ACTUATOR_THRUST_SCALE);
        }
        else {
          // compensate for tilt
          actuatorThrust = (uint16_t) (min(1.0k, thrustDesired / cosTiltAngle) * ACTUATOR_THRUST_SCALE);
        }
#else
        actuatorThrust = (uint16_t) (min(1.0k, thrustDesired) * ACTUATOR_THRUST_SCALE);
#endif
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
      }

      if (thrustDesired > 0.0k)
      {
#if defined(TUNE_ROLL)
        distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
        distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
        distributePower(actuatorThrust, 0, 0, actuatorYaw);
#else
        distributePower(actuatorThrust, actuatorRoll, actuatorPitch, actuatorYaw);
#endif
      }
      else
      {
        distributePower(0, 0, 0, 0);
        controllerResetAllPID();
        if (rollType == CRATE)
          rollDesired = 0.0k;
        if (pitchType == CRATE)
          pitchDesired = 0.0k;
        if (yawType == CRATE)
          yawDesired = rpyActual.z;
      }
    }
  }
}

static void stabilizerAltHoldUpdate(void)
{
  // Get altitude hold commands from pilot
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);

  // Get barometer height estimates
  //TODO do the smoothing within getData
  ms5611GetData(&pressure, &temperature, &aslRaw);
  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
  vSpeedAcc = vSpeed;

  // Reset Integral gain of PID controller if being charged
  if (!pmIsDischarging())
  {
    altHoldPID.integ = 0.0f;
  }

  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Cache last integral term for reuse after pid init
    const float pre_integral = altHoldPID.integ;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);

    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    pidUpdate(&altHoldPID, asl, false);
    altHoldPIDVal = pidGetOutput(&altHoldPID);
  }

  // In altitude hold mode
  if (altHold)
  {
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??
    pidUpdate(&altHoldPID, asl, false);
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidGetOutput(&altHoldPID));

    // compute new thrust
    actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));

    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0f;
    altHoldErr = 0.0f;
    altHoldPIDVal = 0.0f;
  }
}

static void distributePower(uint16_t thrust, int16_t roll,
                            int16_t pitch, int16_t yaw)
{
  // smart clipping: if we just clipped individual motor outputs,
  // a "pitch" output could result in yaw, etc.
  // so instead, clip along the controlled axes

  // prioritize yaw; pitch/roll is useless if our yaw is out of control
  // limit pitch & roll (equally); at the expense of thrust
  // work only with absolute value; not only does it simplify,
  // it ensures we don't stochastically favor one direction
  const int32_t abs_yaw = abs((int32_t) yaw);
  const int32_t max_abs_pitch_roll = max(abs((int32_t) pitch), abs((int32_t) roll));
  const int32_t half_thrust = UINT16_MAX / 2;  // half rounded down
  if (abs_yaw + max_abs_pitch_roll > half_thrust)
  {
    pitch = ((int32_t) pitch * (half_thrust - abs_yaw)) / max_abs_pitch_roll;
    roll = ((int32_t) roll * (half_thrust - abs_yaw)) / max_abs_pitch_roll;
  }
  else
  {
    if (abs_yaw + max_abs_pitch_roll + thrust > UINT16_MAX)
      thrust = UINT16_MAX - (abs_yaw + max_abs_pitch_roll);
    else if (abs_yaw + max_abs_pitch_roll > thrust)
      thrust = abs_yaw + max_abs_pitch_roll;
  }

  // now, still, limitThrust() in case of float rounding errors
  motorPowerM1 = limitThrust((int32_t) thrust + pitch + yaw);
  motorPowerM2 = limitThrust((int32_t) thrust - roll - yaw);
  motorPowerM3 = limitThrust((int32_t) thrust - pitch + yaw);
  motorPowerM4 = limitThrust((int32_t) thrust + roll - yaw);

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(const float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static fix_t deadband(fix_t value, const fix_t threshold)
{
  if (fabsfix(value) <= threshold)
  {
    value = 0.0k;
  }
  else if (value > 0.0k)
  {
    value -= threshold;
  }
  else if (value < 0.0k)
  {
    value += threshold;
  }
  return value;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FIX, roll, &rpyActual.x)
LOG_ADD(LOG_FIX, pitch, &rpyActual.y)
LOG_ADD(LOG_FIX, yaw, &rpyActual.z)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FIX, x, &acc.x)
LOG_ADD(LOG_FIX, y, &acc.y)
LOG_ADD(LOG_FIX, z, &acc.z)
LOG_ADD(LOG_FIX, zw, &accWZ)
LOG_ADD(LOG_FIX, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FIX, x, &gyro.x)
LOG_ADD(LOG_FIX, y, &gyro.y)
LOG_ADD(LOG_FIX, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FIX, x, &mag.x)
LOG_ADD(LOG_FIX, y, &mag.y)
LOG_ADD(LOG_FIX, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)
