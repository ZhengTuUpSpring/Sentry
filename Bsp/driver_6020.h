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
#ifndef __6020_DRIVER_H
#define __6020_DRIVER_H

#include "main.h"
#include "can_receive.h"
#include "pid.h"

/*YAW轴pid参数相关宏定义*/		//带裁判系统		  
#define YAWM6020_POSITIONPID_KP 1.2f	    //1.0f
#define YAWM6020_POSITIONPID_KI 0.00f	
#define YAWM6020_POSITIONPID_KD 0.0f	  
#define YAWM6020_SPEEDPID_KP  450.0f		  //530.0 
#define YAWM6020_SPEEDPID_KI  25.0f
#define YAWM6020_SPEEDPID_KD  0.0f
#define YAWM6020_CURRENTPID_KP 1.0f
#define YAWM6020_CURRENTPID_KI 0.0f
#define YAWM6020_CURRENTPID_KD 0.0f
#define YAW_INTERGRAL_VALVE 1

#define YAW_SPEEDKF 1.0f
#define PITCH_INTERGRAL_VALVE 60					//100


enum CAN_CMD_MODE{
	 CAN_CMD_VOLTAGE=0xFF,
	 CAN_CMD_CURRENT=0xFE
};

typedef struct{
	
	uint8_t motor_id;//电机id
	
	enum CAN_CMD_MODE cmd_mode;
	
	motor_measure_t basic_data;//电流，速度，温度
	
	pid_type_def  position_control_data;//位置环pid相关参数
	
	pid_type_def  speed_control_data;//速度环pid相关参数
	
	pid_type_def  current_control_data;//电压环pid相关参数
	
	
}M6020_HandleTypeDef;



/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:6020电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @param[in]      cmdmode:6020的控制模式( CAN_CMD_VOLTAGE：can发送电压控制信号，CAN_CMD_CURRENT：can发送电流控制信号)
  * @retval         none
  */
void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode);


/**
  * @func			void M6020_GetBasicData(M6020_HandleTypeDef* device)
  * @brief          获取6020电机相关参数
  * @param[in]      device:6020电机句柄
  * @retval         none
  */
void M6020_GetBasicData(M6020_HandleTypeDef* device);

#endif
