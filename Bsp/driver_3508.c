#include "driver_3508.h"
#include "can_receive.h"
#include "string.h"
#include "stdbool.h"


inline uint8_t Friction_GetBoxIndex(uint8_t esc_id)
{
    if (esc_id == FRICTION_ESC_ID_L)
        return FRICTION_BOX_IDX_L;
    else if (esc_id == FRICTION_ESC_ID_R)
        return FRICTION_BOX_IDX_R;
    else
        return 0xFF;  // 非法
}
void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode,bool is_friction_wheel)
{
		device->motor_id=motorid;
	//电机基础信息初始化
   memset(&device->basic_data, 0, sizeof(device->basic_data));

	//初始化pid
	fp32 PID_SPEED[3]={0};//速度环pid参数
	
  if (is_friction_wheel && mode == PID_POSITION)
    {
      switch(motorid)//判断电机id
      {
        case FRICTION_ESC_ID_L:
        PID_SPEED[0]=FRICTION_ID2_SPEEDPID_POSITION_KP;
        PID_SPEED[1]=FRICTION_ID2_SPEEDPID_POSITION_KI;
        PID_SPEED[2]=FRICTION_ID2_SPEEDPID_POSITION_KD;
          break;
        case FRICTION_ESC_ID_R:
        PID_SPEED[0]=FRICTION_ID3_SPEEDPID_POSITION_KP;
        PID_SPEED[1]=FRICTION_ID3_SPEEDPID_POSITION_KI;
        PID_SPEED[2]=FRICTION_ID3_SPEEDPID_POSITION_KD;
          break;

        default :
          break;
      }
    }   
  
	PID_init(&(device->speed_control_data), mode, PID_SPEED, 8192,3000);
	
	device->speed_control_data.integral_valve = 300.0f;
	
}
/**
  * @brief  更新 3508 电机基础数据
  * @param  device: M3508 句柄
  * @retval none
  */
/**
  * @brief  更新 3508 摩擦轮电机基础数据
  * @param  device: M3508 句柄
  * @retval none
  */
void M3508_GetBasicData(M3508_HandleTypeDef *device)
{
    if (device == NULL)
        return;

    uint8_t box_index = Friction_GetBoxIndex(device->motor_id);
    if (box_index == 0xFF)
        return;

    motor_measure_t *src = &dji_motorbox[box_index];

    device->basic_data.ecd            = src->ecd;
    device->basic_data.speed_rpm      = src->speed_rpm;
    device->basic_data.given_current  = src->given_current;
    device->basic_data.temperate      = src->temperate;
    device->basic_data.last_ecd       = src->last_ecd;
    device->basic_data.ref_angle      = src->ref_angle;
    device->basic_data.filtered_speed = src->filtered_speed;
}

