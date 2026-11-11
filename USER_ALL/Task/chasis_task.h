#ifndef _CHASIS_H
#define _CHASIS_H

#include "main.h"

#define E_P 2.5f

extern void Chassis_Task(void *argument);
extern void Chassis_TaskInit(void);
extern void YawDM_Gimbal_ControlOfIMU(float real_angle ,float Yaw_target_angle);

#endif
