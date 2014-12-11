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
 * pidctrl.c - Used to receive/answer requests from client and to receive updated PID values from client
 */
 
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "pidctrl.h"
#include "pid.h"
#include "fix.h"

typedef enum {
  pidCtrlValues = 0x00,
} PIDCrtlNbr;

void pidCrtlTask(void *param);

void pidCtrlInit(void)
{
  xTaskCreate(pidCrtlTask, (const signed char * const)"PIDCrtl",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  crtpInitTaskQueue(6);
}

void pidCrtlTask(void *param)
{
  CRTPPacket p;
  extern PidObject pidRollRate;
  extern PidObject pidPitchRate;
  extern PidObject pidYawRate;
  extern PidObject pidRoll;
  extern PidObject pidPitch;
  extern PidObject pidYaw;
  struct pidValues
  {
    uint16_t rateKpRP;
    uint16_t rateKiRP;
    uint16_t rateKdRP;
    uint16_t attKpRP;
    uint16_t attKiRP;
    uint16_t attKdRP;
    uint16_t rateKpY;
    uint16_t rateKiY;
    uint16_t rateKdY;
    uint16_t attKpY;
    uint16_t attKiY;
    uint16_t attKdY;
  }  __attribute__((packed));
  struct pidValues *pPid;

  while (TRUE)
  {
    if (crtpReceivePacketBlock(6, &p) == pdTRUE)
    {
      PIDCrtlNbr pidNbr = p.channel;
      
      switch (pidNbr)
      {
        case pidCtrlValues:
          pPid = (struct pidValues *)p.data;
          {
            pidSetKp(&pidRollRate, (fix_t)pPid->rateKpRP*0.01k);
            pidSetKi(&pidRollRate, (fix_t)pPid->rateKiRP*0.01k);
            pidSetKd(&pidRollRate, (fix_t)pPid->rateKdRP*0.01k);
            pidSetKp(&pidRoll, (fix_t)pPid->attKpRP*0.01k);
            pidSetKi(&pidRoll, (fix_t)pPid->attKiRP*0.01k);
            pidSetKd(&pidRoll, (fix_t)pPid->attKdRP*0.01k);
            pidSetKp(&pidPitchRate, (fix_t)pPid->rateKpRP*0.01k);
            pidSetKi(&pidPitchRate, (fix_t)pPid->rateKiRP*0.01k);
            pidSetKd(&pidPitchRate, (fix_t)pPid->rateKdRP*0.01k);
            pidSetKp(&pidPitch, (fix_t)pPid->attKpRP*0.01k);
            pidSetKi(&pidPitch, (fix_t)pPid->attKiRP*0.01k);
            pidSetKd(&pidPitch, (fix_t)pPid->attKdRP*0.01k);
            pidSetKp(&pidYawRate, (fix_t)pPid->rateKpY*0.01k);
            pidSetKi(&pidYawRate, (fix_t)pPid->rateKiY*0.01k);
            pidSetKd(&pidYawRate, (fix_t)pPid->rateKdY*0.01k);
            pidSetKp(&pidYaw, (fix_t)pPid->attKpY*0.01k);
            pidSetKi(&pidYaw, (fix_t)pPid->attKiY*0.01k);
            pidSetKd(&pidYaw, (fix_t)pPid->attKdY*0.01k);
          }
          break;
        default:
          break;
      } 
    }
  }
}

