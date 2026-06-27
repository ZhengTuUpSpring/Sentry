#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "can.h"

extern void can_filter_init(void);
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
#endif
