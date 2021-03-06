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

#include "commander.h"
#include "crtp.h"
#include "configblock.h"
#include "param.h"

#include "fix.h"

#define MIN_THRUST  (10000.0k >> 16)
#define MAX_THRUST  (60000.0k >> 16)

struct CommanderCrtpValues
{
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
} __attribute__((packed));

static struct CommanderCrtpValues targetVal[2];
static bool isInit;
static int side=0;
static uint32_t lastUpdate;
static bool isInactive;
static bool altHoldMode = FALSE;
static bool altHoldModeOld = FALSE;

static void commanderCrtpCB(CRTPPacket* pk);
static void commanderWatchdogReset(void);

void commanderInit(void)
{
  if(isInit)
    return;


  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;
}

bool commanderTest(void)
{
  crtpTest();
  return isInit;
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  targetVal[!side] = *((struct CommanderCrtpValues*)pk->data);
  side = !side;
  commanderWatchdogReset();
}

void commanderWatchdog(void)
{
  int usedSide = side;
  uint32_t ticktimeSinceUpdate;

  ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_STABALIZE)
  {
    targetVal[usedSide].roll = 0;
    targetVal[usedSide].pitch = 0;
    targetVal[usedSide].yaw = 0;
  }
  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    targetVal[usedSide].thrust = 0;
    altHoldMode = FALSE; // do we need this? It would reset the target altitude upon reconnect if still hovering
    isInactive = TRUE;
  }
  else
  {
    isInactive = FALSE;
  }
}

static void commanderWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

void commanderGetRPY(fix_t* eulerRollDesired, fix_t* eulerPitchDesired, fix_t* eulerYawDesired)
{
  int usedSide = side;

  *eulerRollDesired  = FIX(targetVal[usedSide].roll);
  *eulerPitchDesired = FIX(targetVal[usedSide].pitch);
  *eulerYawDesired   = FIX(targetVal[usedSide].yaw);
}

void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange)
{
  *altHold = altHoldMode; // Still in altitude hold mode
  *setAltHold = !altHoldModeOld && altHoldMode; // Hover just activated
  *altHoldChange = altHoldMode ? ((float) targetVal[side].thrust - 32767.f) / 32767.f : 0.0f; // Amount to change altitude hold target
  altHoldModeOld = altHoldMode;
}


void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = ANGLE;
  *pitchType = ANGLE;
  *yawType   = CRATE;
}

// scale thrust down so we don't overflow fix_t
void commanderGetThrust(fix_t* thrustOut)
{
  // for some reason (fix_t) (uint16_t) doesn't work?
  // acts as (fix_t) (int16_t) (uint16_t)
  const fix_t thrust = (fix_t) (targetVal[side].thrust >> 1) >> 15;
  if (thrust < MIN_THRUST) *thrustOut = 0.0k;
  else if (thrust > MAX_THRUST) *thrustOut = MAX_THRUST;
  else *thrustOut = thrust;
  commanderWatchdog();
}

// Params for flight modes
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_GROUP_STOP(flightmode)
