#include "4310_apply.h"
#include "driver_4310.h"
#include "gimbalControl.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"
#include <stdbool.h>
motor_t DM_4310;

void mit_motor_init(void)
{
    /* 清空电机结构体 */
    memset(&DM_4310, 0, sizeof(motor_t));

    /* 基本参数 */
    DM_4310.id = 0x01;             
    DM_4310.ctrl.mode = 0;        // 0 = MIT 模式
    DM_4310.cmd.mode  = 0;

    /* 默认不启动 */
    DM_4310.start_flag = 0;
}

void pos_motor_init(void)
{
    /* 清空电机结构体 */
    memset(&DM_4310, 0, sizeof(motor_t));

    /* 基本参数 */
    DM_4310.id = 1;             
    DM_4310.ctrl.mode = 1;        
    DM_4310.cmd.mode  = 0;

    /* 默认不启动 */
    DM_4310.start_flag = 0;
}
void mit_motor_enable(void)
{
    DM_4310.start_flag = 1;
    
    enable_motor_mode(&hcan1, DM_4310.id, MIT_MODE);
}

bool motor1_init(motor_t *motor1)
{
    static uint32_t last_enable_time = 0;
    uint32_t now;

    if (!motor1) {
        return false;
    }

    if (motor1->para.state == 0x01) {
        motor1->start_flag = 1;
        return true;
    }

    now = HAL_GetTick();
    if (now - last_enable_time >= 50) {
        last_enable_time = now;
        enable_motor_mode(&hcan1, motor1->id, POS_MODE);
    }

    return false;
}
bool motormit_init(motor_t *motor1)
{
    static uint32_t last_enable_time = 0;
    uint32_t now;

    if (!motor1) {
        return false;
    }

    if (motor1->para.state == 0x01) {
        motor1->start_flag = 1;
        return true;
    }

    now = HAL_GetTick();
    if (now - last_enable_time >= 50) {
        last_enable_time = now;
        enable_motor_mode(&hcan1, motor1->id, MIT_MODE);
    }

    return false;
}
void pos_motor_enable(void)
{
    DM_4310.start_flag = 1;
    
    enable_motor_mode(&hcan1, DM_4310.id, POS_MODE);
	         pitch_spepos_control(0);
}
void dm4310_init(motor_t *motor, dm4310_mode_e mode)
{
	DM_4310.id = 0x01;          // 电机拨码 ID
  DM_4310.start_flag = 0;     // ? 必须
    motor->ctrl.mode = mode;
}

void dm4310_enable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
        case DM_MIT_MODE:
            enable_motor_mode(hcan, motor->id, MIT_MODE);
            break;

        case DM_POS_SPD_MODE:
            enable_motor_mode(hcan, motor->id, POS_MODE);
            break;

        case DM_SPD_MODE:
            enable_motor_mode(hcan, motor->id, SPEED_MODE);
            break;
    }
}

void dm4310_set(motor_t *motor,
                float pos, float vel,
                float kp,  float kd,
                float tor)
{
    /* 所有模式都支持的参数 */
    motor->ctrl.pos_set = pos;
    motor->ctrl.vel_set = vel;

    /* MIT 专用参数 */
    motor->ctrl.kp_set = kp;
    motor->ctrl.kd_set = kd;
    motor->ctrl.tor_set = tor;
}
void dm4310_send(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
        case DM_MIT_MODE:
            mit_ctrl(hcan,
                     motor->id,
                     motor->ctrl.pos_set,
                     motor->ctrl.vel_set,
                     motor->ctrl.kp_set,
                     motor->ctrl.kd_set,
                     motor->ctrl.tor_set);
            break;

        case DM_POS_SPD_MODE:
            pos_speed_ctrl(hcan,
                           motor->id,
                           motor->ctrl.pos_set,
                           motor->ctrl.vel_set);
            break;

        case DM_SPD_MODE:
            speed_ctrl(hcan,
                       motor->id,
                       motor->ctrl.vel_set);
            break;
    }
}
// 1. 纯力矩模式：用于标定和拖动示教
// 此时 kp=0, kd=0，电机不锁定位置，只输出力矩
void mit_motor_gravity_compensation_only(float compensation_torque)
{
	
    // MIT 模式，kp=0, kd=0，只输出力矩
    mit_ctrl(&hcan1, DM_4310.id, 
             0.0f,    // pos 
             0.0f,    // vel 
             0.5f,    // kp = 0，禁用位置刚度
             0.3f,    // kd = 0，禁用阻尼
             compensation_torque);  // tau: 重力补偿力矩
}

// 2. 运行时模式：带重力补偿的位置控制
void mit_motor_position_with_gravity_deg(float target_deg, float compensation_torque)
{
    float target_rad = target_deg * 3.14159f / 180.0f;
	
	if(DM_4310.start_flag == 1){
    mit_ctrl(&hcan1, DM_4310.id, target_rad, 0.0f, 4.0f, 0.3f, compensation_torque);
	}
}
void pitch_mit_control(float target_deg)
{
	    float compensation_torque=Get_GravityTorque();
	    float target_rad = target_deg * 3.14159f / 180.0f;
		  if (DM_4310.start_flag != 1) {
        return;
                                   }

    mit_ctrl(&hcan1, DM_4310.id, target_rad, 2.0f, 4.0f, 0.3f, compensation_torque);

}
void pitch_spepos_control(float target_deg)
{
	 float target_rad = target_deg * 3.14159f / 180.0f;
   if (DM_4310.start_flag != 1) {
        return;
			}
    pos_speed_ctrl(&hcan1, DM_4310.id, target_rad, 10.0f);

}
