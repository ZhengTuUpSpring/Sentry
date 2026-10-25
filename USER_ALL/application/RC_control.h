#ifndef _RC_CON_H
#define _RC_CON_H
#include "main.h"
#include "bsp_dji.h"

extern RC_t rc;
extern void RC_CAN_parse_to_RC_struct(uint8_t *chassis_can_recv_data, RC_t *rc_struct);
extern int16_t apply_deadzone(int16_t value);
#endif
