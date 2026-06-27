/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "can.h"
#include "FreeRTOS.h"


#define CHASSIS_CAN hcan2   //底盘（双板）
#define GIMBAL_CAN  hcan1   //云台
#define Shoot_CAN hcan1     //摩擦轮
#define Trigger_CAN hcan1   //拨弹盘
#define NUC_CAN hcan2       //小电脑
#define DJI_MOTOR_NUMBER 10
/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_TRIGGER_MOTOR_ID=0x204,
    CAN_YAW_MOTOR_ID = 0x205,
//    CAN_PIT_MOTOR_ID = 0x206,
//    CAN_TRIGGER_DOMN_MOTOR_ID = 0x207,
	  
    CAN_GIMBAL_YAW_ID = 0x1FF,
	  CAN_GIMBAL_PITCH_ID = 0X101
} can_msg_id_e;

//#define CAN_TRIGGER_DOMN_MOTOR_ID 0x205
//#define CAN_TRIGGER_UP_MOTOR_ID 0x206

/******************************************** 大疆电机数据 ********************************************/
typedef struct
{
	uint16_t ecd;			//机械转角
	int16_t speed_rpm;  	//电机转速
	int16_t given_current;	//扭矩电流
	uint8_t temperate;		//温度
	int16_t last_ecd;		//上一次机械转角
	float ref_angle;    //当前角度
//	uint8_t is_online;      //是否收到过真实反馈帧
	
	 float   filtered_speed;   //摩擦轮滤波专用 
//	int cylinder_number;	//旋转圈数
} motor_measure_t;
extern motor_measure_t dji_motorbox[DJI_MOTOR_NUMBER];

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
//        (ptr)->is_online = 1;                                           \
    }


		
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw:   (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev:   (0x208) 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void CAN_cmd_gimbal_yaw(int16_t yaw);//CAN1
void CAN_cmd_gimbal_pitch(int16_t position, int16_t Velocity);
void CAN_cmd_NUC(float PointX, float PointY, uint8_t state);

//摩擦轮
void CAN_cmd_shootFric(int16_t motor1, int16_t motor2, int16_t trigger_motor);
//拨弹盘
void CAN_cmd_trigger(int16_t trigger_motor);

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);


/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void);


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

const motor_measure_t *get_shoot_frcitionmotor_measure_point(uint8_t i);

#endif
