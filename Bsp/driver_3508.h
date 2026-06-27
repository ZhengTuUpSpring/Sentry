#ifndef __M3508_H
#define __M3508_H
#include "main.h"
#include "can_receive.h"
#include "pid.h"
#include "stdbool.h"
	//**************位置PID******************** */
	//				驱动轮
	#define M3508_ID1_SPEEDPID_POSITION_KP   9.5f//19.0f
	#define M3508_ID1_SPEEDPID_POSITION_KI   0.1f//0.02f
	#define M3508_ID1_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define M3508_ID2_SPEEDPID_POSITION_KP   9.5f//21.0f//17.0f//15.0f
	#define M3508_ID2_SPEEDPID_POSITION_KI   0.1f//0.1f//0.009f//0.008f
	#define M3508_ID2_SPEEDPID_POSITION_KD   0.0f// 0.0f

	#define M3508_ID3_SPEEDPID_POSITION_KP   20.0f//17.0f
	#define M3508_ID3_SPEEDPID_POSITION_KI   0.11f//0.008f
	#define M3508_ID3_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define M3508_ID4_SPEEDPID_POSITION_KP   10.0f//20.0f//19.0f
	#define M3508_ID4_SPEEDPID_POSITION_KI   0.1f//0.02f
	#define M3508_ID4_SPEEDPID_POSITION_KD   0.0f//0.0f

	//				摩擦轮
	#define FRICTION_ID1_SPEEDPID_POSITION_KP   0.0f//19.0f
	#define FRICTION_ID1_SPEEDPID_POSITION_KI   0.0f//0.02f
	#define FRICTION_ID1_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define FRICTION_ID2_SPEEDPID_POSITION_KP   3.0f//21.0f//17.0f//15.0f
	#define FRICTION_ID2_SPEEDPID_POSITION_KI   0.1f//0.1f//0.009f//0.008f
	#define FRICTION_ID2_SPEEDPID_POSITION_KD   0.0f// 0.0f

	#define FRICTION_ID3_SPEEDPID_POSITION_KP   3.0f//17.0f
	#define FRICTION_ID3_SPEEDPID_POSITION_KI   0.1f//0.008f
	#define FRICTION_ID3_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define FRICTION_ID4_SPEEDPID_POSITION_KP   0.0f//20.0f//19.0f
	#define FRICTION_ID4_SPEEDPID_POSITION_KI   0.0f//0.02f
	#define FRICTION_ID4_SPEEDPID_POSITION_KD   0.0f//0.0f	
	
	/***************增量PID***********************/
	//				驱动轮
	#define M3508_ID1_SPEEDPID_DELTA_KP    9.5f
	#define M3508_ID1_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID1_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID2_SPEEDPID_DELTA_KP    15.0f
	#define M3508_ID2_SPEEDPID_DELTA_KI    0.2f
	#define M3508_ID2_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID3_SPEEDPID_DELTA_KP    20.0f
	#define M3508_ID3_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID3_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID4_SPEEDPID_DELTA_KP    15.0f
	#define M3508_ID4_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID4_SPEEDPID_DELTA_KD    0.0f	
	// 			  摩擦轮
	#define FRICTION_ID1_SPEEDPID_DELTA_KP   0.0f//19.0f
	#define FRICTION_ID1_SPEEDPID_DELTA_KI   0.0f//0.02f
	#define FRICTION_ID1_SPEEDPID_DELTA_KD   0.0f//0.0f

	#define FRICTION_ID2_SPEEDPID_DELTA_KP   0.0f//19.0f
	#define FRICTION_ID2_SPEEDPID_DELTA_KI   0.0f//0.02f
	#define FRICTION_ID2_SPEEDPID_DELTA_KD   0.0f//0.0f

	#define FRICTION_ID3_SPEEDPID_DELTA_KP   0.0f//19.0f
	#define FRICTION_ID3_SPEEDPID_DELTA_KI   0.0f//0.02f
	#define FRICTION_ID3_SPEEDPID_DELTA_KD   0.0f//0.0f

	#define FRICTION_ID4_SPEEDPID_DELTA_KP   0.0f//19.0f
	#define FRICTION_ID4_SPEEDPID_DELTA_KI   0.0f//0.02f
	#define FRICTION_ID4_SPEEDPID_DELTA_KD   0.0f//0.0f
typedef struct
{
    uint8_t motor_id;
    motor_measure_t basic_data;   
    pid_type_def speed_control_data;
    pid_type_def currrent_control_data;
	
} M3508_HandleTypeDef;
#define FRICTION_ESC_ID_L  2
#define FRICTION_ESC_ID_R  3

#define FRICTION_BOX_IDX_L 6
#define FRICTION_BOX_IDX_R 7
/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:3508电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M3508_GetBasicData(M3508_HandleTypeDef* device);
extern inline uint8_t Friction_GetBoxIndex(uint8_t esc_id);
/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device)
  * @brief          获取3508电机相关参数
  * @param[in]      device:3508电机句柄
  * @retval         none
  */
void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode, bool is_friction_wheel);

#endif
