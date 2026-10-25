#include "4310_control.h"

#include "apply_4310.h"
#include "bsp_can.h"
#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"
#include <math.h>
#include "pid.h"

#define YAW_CAN hcan1  
#define CAN_YAW_ID 0x005
#define CAN_VELMODE_ID 0x200
//达妙电机数据结构体实例化
motor_t DM_4310;

pid_type_def DM4310_pid;
pid_type_def yaw_pos_pid;
friction_compensation_t YAW_fric;
PID_t YAW_IMU_SPEED_PID;

void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id = (rx_data[0])&0x0F;
    motor->para.state = (rx_data[0])>>4;
    motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
    motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
    motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
        motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}

void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id +mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}
void YAW_4310motor_init(void)
{

    memset(&DM_4310, 0, sizeof(motor_t));
		memset(&yaw_pos_pid, 0, sizeof(pid_type_def));
	
    DM_4310.id = 5;
    DM_4310.ctrl.mode = MOTOR_MODE_MIT;
    DM_4310.cmd.mode  = 0;
    DM_4310.tmp.PMAX = P_MAX;
    DM_4310.tmp.VMAX = V_MAX;
    DM_4310.tmp.TMAX = T_MAX;

	 fp32 DM_POS_PID[3]={
	 YAW_POS_KP,
	 YAW_POS_KI,
   YAW_POS_KD};
	 
	friction_compensation_init(&YAW_fric,
    0.12f,   // Coulomb 摩擦力矩 (N·m)
    0.0f,   // 粘性系数 (N·m/(rad/s))
    80.0f,   // tanh 斜率
    1.0f  // output_per_nm
	 );
	 
	 PID_Init(&YAW_IMU_SPEED_PID, YAW_IMU_KP, YAW_IMU_KI, YAW_IMU_KD, 1000.0f, 1000.0f);
	 PID_init(&yaw_pos_pid,PID_POSITION,DM_POS_PID,1000,1000);
   DM_4310.start_flag = 0;
}


//bool motor1_init(motor_t *motor1)
//{
//    static uint32_t last_enable_time = 0;
//    uint32_t now;
//		uint16_t mode_id;
//	
//    if (!motor1) {
//        return false;
//    }

//    switch (motor1->ctrl.mode)
//    {
//        case MOTOR_MODE_MIT:
//            mode_id = MIT_MODE;
//            break;
//        case MOTOR_MODE_SPEED:
//            mode_id = SPEED_MODE;
//            break;
//        case MOTOR_MODE_POSVEL:
//        default:
//            mode_id = POS_MODE;
//            break;
//    }

//    if (motor1->para.state == 0x01) {
//        motor1->start_flag = 1;
//        return true;
//    }

//    now = HAL_GetTick();
//    if (now - last_enable_time >= 50) {
//        last_enable_time = now;
//        enable_motor_mode(&hcan1, motor1->id, mode_id);
//    }

//    return false;
//}
bool motor1_init(motor_t *motor1)
{
  if(!motor1) return false;
  if(motor1->para.state != 0x01)
  {
    enable_motor_mode(&hcan1, DM_4310.id, MIT_MODE);
    osDelay(50);
  }else return false;
}
void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}
//�ٶ�λ�ÿ���ģʽ
void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}

void mit_ctrl(CAN_HandleTypeDef* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = (MIT_MODE | motor_id);

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}

void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}

void friction_compensation_init(friction_compensation_t *compensation,
                                float coulomb_torque_nm,
                                float viscous_coefficient_nm_per_rad_s,
                                float tanh_slope_s_per_rad,
                                float output_per_nm)
{
    if (compensation == NULL)
    {
        return;
    }

    compensation->coulomb_torque_nm = (coulomb_torque_nm >= 0.0f)
                                       ? coulomb_torque_nm
                                       : -coulomb_torque_nm;
    compensation->viscous_coefficient_nm_per_rad_s = (viscous_coefficient_nm_per_rad_s >= 0.0f)
                                                      ? viscous_coefficient_nm_per_rad_s
                                                      : -viscous_coefficient_nm_per_rad_s;
    compensation->tanh_slope_s_per_rad = (tanh_slope_s_per_rad >= 0.0f)
                                          ? tanh_slope_s_per_rad
                                          : -tanh_slope_s_per_rad;
    compensation->output_per_nm = output_per_nm;
}

float friction_compensation_calc_torque(const friction_compensation_t *compensation,
                                        float angular_velocity_rad_s)
{
    if (compensation == NULL)
    {
        return 0.0f;
    }

    return compensation->coulomb_torque_nm
           * tanhf(compensation->tanh_slope_s_per_rad * angular_velocity_rad_s)
           + compensation->viscous_coefficient_nm_per_rad_s * angular_velocity_rad_s;
}

float friction_compensation_calc_output(const friction_compensation_t *compensation,
                                        float angular_velocity_rad_s)
{
    if (compensation == NULL)
    {
        return 0.0f;
    }

    return friction_compensation_calc_torque(compensation, angular_velocity_rad_s)
           * compensation->output_per_nm;
}
