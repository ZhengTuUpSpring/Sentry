#include "main.h"
#include "bsp_dji.h"
#include "stdlib.h"
// 摇杆死区，防止轻微抖动
#define JOYSTICK_DEAD_ZONE 100

RC_t rc;

void RC_CAN_parse_to_RC_struct(uint8_t *chassis_can_recv_data, RC_t *rc_struct)
{
    if (chassis_can_recv_data == NULL || rc_struct == NULL)
    {
        return; // 空指针保护
    }
  #define SIGN_EXTEND_12BIT(x) ((x) & 0x800 ? (x) | 0xF000 : (x))  // 12位符号扩展

    // 解析ch0
    uint16_t ch0_12bit = ((uint16_t)chassis_can_recv_data[0] << 4) | 
                        ((uint16_t)(chassis_can_recv_data[1] >> 4) & 0x000F);
    rc_struct->ch0 = (int16_t)SIGN_EXTEND_12BIT(ch0_12bit);

    // 解析ch1
    uint16_t ch1_12bit = ((uint16_t)(chassis_can_recv_data[1] & 0x0F) << 8) | 
                        (uint16_t)chassis_can_recv_data[2];
    rc_struct->ch1 = (int16_t)SIGN_EXTEND_12BIT(ch1_12bit);

    // 解析roller
    uint16_t roll_12bit = ((uint16_t)chassis_can_recv_data[3] << 4) | 
                         ((uint16_t)(chassis_can_recv_data[4] >> 4) & 0x000F);
    rc_struct->roller = (int16_t)SIGN_EXTEND_12BIT(roll_12bit);

    // 解析ch2
    uint16_t ch2_12bit = ((uint16_t)(chassis_can_recv_data[4] & 0x0F) << 8) | 
                        (uint16_t)chassis_can_recv_data[5];
    rc_struct->ch2 = (int16_t)SIGN_EXTEND_12BIT(ch2_12bit);

    // 解析s1/s2
    rc_struct->s1 = (int16_t)((chassis_can_recv_data[6] >> 4) & 0x03);
    rc_struct->s2 = (int16_t)(chassis_can_recv_data[6] & 0x03);

    #undef SIGN_EXTEND_12BIT
}
int16_t apply_deadzone(int16_t value) {
    if(abs(value) < JOYSTICK_DEAD_ZONE) return 0;
    return value;
}
