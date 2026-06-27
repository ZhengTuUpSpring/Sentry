#ifndef __4310APPLY_H
#define __4310APPLY_H
#include <stdbool.h>
#include "driver_4310.h"
extern motor_t DM_4310;
typedef enum
{
    DM_MIT_MODE    = 0,
    DM_POS_SPD_MODE = 1,
    DM_SPD_MODE     = 2,
} dm4310_mode_e;
/* 初始化 */
void mit_motor_init(void);
void mit_motor_enable(void);
void dm4310_init(motor_t *motor, dm4310_mode_e mode);
void pos_motor_init(void);
/* 使能 */
void dm4310_enable(CAN_HandleTypeDef *hcan, motor_t *motor);
void pos_motor_enable(void);
/* 统一参数设置 */
void dm4310_set(motor_t *motor,
                float pos, float vel,
                float kp,  float kd,
                float tor);

/* 统一发送 */
bool motormit_init(motor_t *motor1);
void dm4310_send(CAN_HandleTypeDef *hcan, motor_t *motor);
bool motor1_init(motor_t *motor1);								
void mit_motor_gravity_compensation_only(float compensation_torque);
void mit_motor_position_with_gravity_deg(float target_deg, float compensation_torque);
void pitch_mit_control(float target_deg);								
void pitch_spepos_control(float target_deg);								
#endif
				