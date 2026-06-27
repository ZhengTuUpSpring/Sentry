/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       2006_driver.c/h
  * @brief      
  *             这里是2006的驱动函数，通过pid控制控制电机.
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
#ifndef __2006_DRIVER_H
#define __2006_DRIVER_H

#include "main.h"
#include "can_receive.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

#define M2006_REDUCTION_RATIO 11.000000f //减速比:36:1
//#define M2006_RECORD_EXP_ECDBOSX_LENGTH 10

#define TARGET_SPEED_TOLERANCE    4600    // 转速偏差阈值(rpm)：实际比目标小50rpm以上判定为偏差极大
#define BLOCK_DETECTION_TIMEOUT   70     	// 堵转判定超时时间(ms)：偏差超阈值持续1秒判定堵转
#define REVERSE_RUN_DURATION      80     // 反转脱困时长(ms)
#define REVERSE_SPEED             -1000   // 反转转速(rpm)：负数=反转，数值按需调整
#define SPEED_UPDATE_INTERVAL     10      // 转速检测间隔(ms)：建议10ms
#define BLOCK_STOP_DURATION       1000    // 多次堵转后停机时长（ms）
#define BLOCK_TIMES								5				// 堵转最大次数

/*位置PID参数*/
#define UP2006_SPEEDPID_POSITION_KP 10.0f
#define UP2006_SPEEDPID_POSITION_KI 0.0f
#define UP2006_SPEEDPID_POSITION_KD 0.0f

/*增量式PID参数*/
#define UP2006_SPEEDPID_DELTA_KP 10.0f
#define UP2006_SPEEDPID_DELTA_KI 0.0f
#define UP2006_SPEEDPID_DELTA_KD 0.0f

typedef enum {
    MOTOR_STATE_NORMAL,    			// 正常运行
    MOTOR_STATE_BLOCKED,   			// 堵转判定
    MOTOR_STATE_REVERSE,    		// 反转脱困
	  MOTOR_STATE_REVERSE_DELAY,  // 反转后延时
		MOTOR_STATE_BLOCK_STOP     	// 专门处理多次堵转后的停机逻辑
} MotorState;

typedef struct{
	
	uint8_t motor_id;//电机id
	
	int  expand_ecd;//电机减速箱旋转轴 绝对编码器值
	
	int  cylinder_number;//旋转圈数
	
	int16_t record_last_ecd;//这个为转子的上一次的绝对编码器值
	
//	int16_t record_exp_ecdboxs[M2006_RECORD_EXP_ECDBOSX_LENGTH];//记录转子绝对编码器值
	
	motor_measure_t basic_data;//电流，速度，温度
	
    pid_type_def  position_control_data;//位置环pid相关参数
	
	pid_type_def  speed_control_data;//速度环pid相关参数
	
	pid_type_def  currrent_control_data;//电流环pid相关参数
	
	MotorState  state;          // 电机当前状态
	
  uint32_t    block_timer;    // 堵转计时
	
  uint32_t    reverse_timer;  // 反转计时
	
  uint32_t reverse_delay_timer;  //反转后延时
	
	uint8_t block_count;          // 堵转次数
	
	uint32_t stop_timer;               // 停机计时器
	
}M2006_HandleTypeDef;


//extern M2006_HandleTypeDef up_trigger_motor;
//extern M2006_HandleTypeDef down_trigger_motor;

/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:2006电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode);


/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device);


/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机编码器绝对角度(对编码器角度范围进行了扩充)
  * @brief          该函数对低速旋转的电机有效，高速旋转可能会存在丢圈问题
  * @param[in]      device:2006电机句柄
  * @retval         扩充后的编码器角度值
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device);


/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          将2006电机(转子)编码器绝对角度值 转换为 (减速箱)编码器绝对角度值
  * @param[in]      absoluteencoder:2006电机(转子)编码器绝对角度值
  * @retval         (减速箱)编码器绝对角度值
  */
float M2006_AbsoluteEncoderToAngle(int absoluteencoder);

/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机堵转检测
  * @param[in]      device:2006电机句柄
  * @retval         bool值，user_true为电机堵转，否则为电机未堵转
  */
uint8_t M2006_LockedMotorDetertion(M2006_HandleTypeDef* device);
  
#endif
