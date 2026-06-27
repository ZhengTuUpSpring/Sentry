/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       6020_driver.c/h
  * @brief      
  *             这里是6020的驱动函数，通过pid控制控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "driver_6020.h"
#include "arm_math.h"

/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:6020电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @param[in]      cmdmode:6020的控制模式( CAN_CMD_VOLTAGE：can发送电压控制信号，CAN_CMD_CURRENT：can发送电流控制信号)
  * @retval         none
  */
void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
{
	if(device==NULL)
	{
		return;
	}
	/*初始化电机相关参数*/
	device->motor_id=motorid;
	device->cmd_mode=cmdmode;
	
	device->basic_data.ecd=0;
	device->basic_data.given_current=0;
	device->basic_data.last_ecd=0;
	device->basic_data.speed_rpm=0;
	device->basic_data.temperate=0;
//	device->basic_data.is_online=0;
	/*初始化pid相关参数*/
	fp32 pid_position[3]={0.0f};
	fp32 pid_speed[3]={0.0f};
  fp32 pid_current[3]={0.0f};
	fp32 integralvalve=0;
	if(pidmode==PID_POSITION)
	{
		switch(0x204+motorid)//初始化的时候给指定电机pid参数赋值
		{
			case CAN_YAW_MOTOR_ID:
				
			pid_position[0]=YAWM6020_POSITIONPID_KP;
			pid_position[1]=YAWM6020_POSITIONPID_KI;
			pid_position[2]=YAWM6020_POSITIONPID_KD;
			
			pid_speed[0]=YAWM6020_SPEEDPID_KP;
			pid_speed[1]=YAWM6020_SPEEDPID_KI;
			pid_speed[2]=YAWM6020_SPEEDPID_KD;
			
			pid_current[0]=YAWM6020_CURRENTPID_KP;
			pid_current[1]=YAWM6020_CURRENTPID_KI;
			pid_current[2]=YAWM6020_CURRENTPID_KD;
			
			/*初始化积分分离阀门值*/
			integralvalve=YAW_INTERGRAL_VALVE;
				break;
			default:
				break;
		}
	}
	
	if(cmdmode==CAN_CMD_CURRENT)
	{
		PID_init(&(device->position_control_data),pidmode, pid_position, 350,350);
		PID_init(&(device->speed_control_data),   pidmode, pid_speed,    16384,16384);
	}
	else if(cmdmode==CAN_CMD_VOLTAGE)
	{
		PID_init(&(device->position_control_data),pidmode, pid_position, 350,350);
		PID_initwithkf(&(device->speed_control_data),pidmode,pid_speed,5.0f,25000,25000);
//		PID_init(&(device->speed_control_data),   pidmode, pid_speed,    25000,25000);//速度环输出电压吗
	}
	PID_init(&(device->current_control_data), pidmode, pid_current, 15000,15000);
	device->position_control_data.integral_valve=integralvalve;
}

/**
  * @func			void M6020_GetBasicData(M6020_HandleTypeDef* device)
  * @brief          获取6020电机相关参数
  * @param[in]      device:6020电机句柄
  * @retval         none
  */
void M6020_GetBasicData(M6020_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	/*电机id判断*/
	switch(device->motor_id+0x204) //真电机id：yaw:0x205  pitch:0x206  6020初始化的时候电机id: yaw:0x01  pit:0x02
	{
	  case CAN_YAW_MOTOR_ID:
		device->basic_data.ecd=get_yaw_gimbal_motor_measure_point()->ecd;
		device->basic_data.given_current=get_yaw_gimbal_motor_measure_point()->given_current;
		device->basic_data.last_ecd=get_yaw_gimbal_motor_measure_point()->last_ecd;
		device->basic_data.speed_rpm=get_yaw_gimbal_motor_measure_point()->speed_rpm;
		device->basic_data.temperate=get_yaw_gimbal_motor_measure_point()->temperate;
//		device->basic_data.is_online=get_yaw_gimbal_motor_measure_point()->is_online;
	  break;
		default:
		break;
	}
}
