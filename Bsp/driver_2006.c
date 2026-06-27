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
#include "driver_2006.h"
#include <math.h>
#define ABS(x) (((x) < 0) ? -(x) : (x))
//M2006_HandleTypeDef up_trigger_motor;
//M2006_HandleTypeDef down_trigger_motor;

/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:2006电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
{
	/*初始化电机相关参数*/
	device->motor_id=motorid;
	device->expand_ecd=0;
	device->cylinder_number=0;
	device->record_last_ecd=0;
//	for(int i=0;i< M2006_RECORD_EXP_ECDBOSX_LENGTH;i++){device->record_exp_ecdboxs[i]=0;}
	
	device->basic_data.ecd=0;
	device->basic_data.given_current=0;
	device->basic_data.last_ecd=0;
	device->basic_data.speed_rpm=0;
	device->basic_data.temperate=0;
	
	/*初始化pid相关参数*/
	fp32 pid_position[3]={0.0f};
	fp32 pid_speed[3]={0.0f};
    fp32 pid_current[3]={0.0f};
	if(pidmode==PID_POSITION)
	{
		switch(0x200+motorid)
		{
			case  CAN_TRIGGER_MOTOR_ID:
			
			pid_speed[0]=UP2006_SPEEDPID_POSITION_KP;
			pid_speed[1]=UP2006_SPEEDPID_POSITION_KI;
			pid_speed[2]=UP2006_SPEEDPID_POSITION_KD;
				break;
			default :
				break;
		}
	}
	else if(pidmode == PID_DELTA)
	{
		switch(0x200+motorid)
		{
			case  CAN_TRIGGER_MOTOR_ID:
			
			pid_speed[0]=UP2006_SPEEDPID_DELTA_KP;
			pid_speed[1]=UP2006_SPEEDPID_DELTA_KI;
			pid_speed[2]=UP2006_SPEEDPID_DELTA_KD;
			
				break;
			default :
				break;
		}		
	}
	//pid位置环，速度环，电流环初始化
  PID_init(&(device->position_control_data),pidmode, pid_position, 16384,16384);
  PID_init(&(device->speed_control_data),   pidmode, pid_speed,    30000,16384);
	PID_init(&(device->currrent_control_data),pidmode, pid_current,  16384,16384);
}

/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	switch(device->motor_id+0x200)
	{
      case CAN_TRIGGER_MOTOR_ID:
			device->basic_data.ecd=get_trigger_motor_measure_point()->ecd;
			device->basic_data.given_current=get_trigger_motor_measure_point()->given_current;
			device->basic_data.last_ecd=get_trigger_motor_measure_point()->last_ecd;
			device->basic_data.speed_rpm=get_trigger_motor_measure_point()->speed_rpm;
			device->basic_data.temperate=get_trigger_motor_measure_point()->temperate;
	  break;
		default:
			break;
			
	}
}

/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机编码器绝对角度(对编码器角度范围进行了扩充)
  * @brief          该函数对低速旋转的电机有效，高速旋转可能会存在丢圈问题
  * @param[in]      device:2006电机句柄
  * @retval         扩充后的编码器角度值
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return 0;
	}
	//用上一次位置-这一次位置判断是否有突变
	int16_t d_ecd=device->basic_data.ecd-device->record_last_ecd;
	/*有突变记录一圈*/
	if(d_ecd>8000)//反转突变
	{
		device->cylinder_number--;
	}
	else if(d_ecd<-8000)//正转突变
	{
		device->cylinder_number++;
	}
	else//未突变
	{
		/*计算范围拓展后的绝对编码器值*/ 
		if(device->cylinder_number>=0)
		{
			device->expand_ecd=device->cylinder_number*8191+device->basic_data.ecd;
		}
		else
		{
			device->expand_ecd=device->cylinder_number*8191+8191-device->basic_data.ecd+8191;
		}                
	}
	/*更新上一次绝对编码器值*/
	device->record_last_ecd=device->basic_data.ecd;
	
	return device->expand_ecd;
}

/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          将2006电机(转子)编码器绝对角度值 转换为 (减速箱)编码器绝对角度值
  * @param[in]      absoluteencoder:2006电机(转子)编码器绝对角度值
  * @retval         (减速箱)编码器绝对角度值
  */
//返回值
//单位：度（°）
//表示最终输出轴的机械角度
float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
{
	//转子角度转换
	float rev_angle=(float)absoluteencoder*360.0f/8191;
	//减速箱角度转换
	rev_angle/=M2006_REDUCTION_RATIO;
	
	return rev_angle;
}

//Locked-rotor detection
/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机堵转检测
  * @param[in]      device:2006电机句柄
  * @retval         bool值，user_true为电机堵转，否则为电机未堵转
  */
static inline uint8_t M2006_SpeedBlockCheck(M2006_HandleTypeDef *m)
{
    if ((int16_t)m->speed_control_data.set != 0)
    {
        if ((ABS(m->basic_data.given_current) > 9000) &&
            (ABS(m->basic_data.speed_rpm) < 30))
        {
            return 1;
        }
    }
    return 0;
}

static inline uint8_t M2006_PositionBlockCheck(M2006_HandleTypeDef *m)
{
    if ((int32_t)m->position_control_data.set !=
        (int32_t)m->position_control_data.fdb)
    {
        if (m->expand_ecd == m->record_last_ecd)
        {
            return 1;
        }
    }
    return 0;
}

uint8_t M2006_LockedMotorDetertion(M2006_HandleTypeDef *m)
{
    if (m == NULL)
        return 0;

    uint8_t blocked =
        M2006_SpeedBlockCheck(m) ||
        M2006_PositionBlockCheck(m);

    if (blocked)
    {
        if (m->block_timer < 300)
            m->block_timer++;
    }
    else
    {
        m->block_timer = 0;
    }

    if (m->block_timer >= 300)
    {
        m->block_count++;
        m->state = MOTOR_STATE_BLOCKED;
        m->block_timer = 0;   
        return 1;
    }

    return 0;
}
