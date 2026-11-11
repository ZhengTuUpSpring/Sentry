#ifndef Judge_Sys_H
#define Judge_Sys_H

#include "main.h"
#define SEND_HEAT_DATA_ID 0X500
extern void JudgSys_Task(void *argument);
extern void CAN_send_shoot_Heatdata_task(void *argument);
extern void RS_SendToHighCore(uint8_t progress, uint16_t current_hp, uint8_t robot_status, uint16_t shoot_heat, uint16_t shoot_heat_limit);
#endif
