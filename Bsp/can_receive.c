#include "can_receive.h"
#include "can.h"  

motor_measure_t dji_motorbox[DJI_MOTOR_NUMBER];

uint8_t testCanArr[8] = {0};
uint16_t tec = 0;
uint16_t rec = 0;
/**
* @brief          控制达妙4310电机(ID: ) 
  * @param[in]      position：位置数据
  * @param[in]      Velocity：速度数据
  * @retval         none
  */
void CAN_cmd_gimbal_pitch(int16_t position, int16_t Velocity)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  gimbal_tx_message;
	//CAN标准帧填充
	gimbal_tx_message.StdId = CAN_GIMBAL_PITCH_ID;//0x101
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	//低位在前
	uint8_t gimbal_can_send_data[8]; 
	//位置数据
	gimbal_can_send_data[0] = position >> 8;
	gimbal_can_send_data[1] = position ;

	//速度数据
	gimbal_can_send_data[2] = Velocity >> 8;
	gimbal_can_send_data[3] = Velocity ;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){};
	//发送数据
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw:   (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal_yaw(int16_t yaw)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  gimbal_tx_message;

	gimbal_tx_message.StdId = CAN_GIMBAL_YAW_ID;//0x1FF
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

	uint8_t gimbal_can_send_data[8];
	gimbal_can_send_data[0] = (yaw >> 8);
	gimbal_can_send_data[1] = yaw;
	gimbal_can_send_data[2] = (0 >> 8);
	gimbal_can_send_data[3] = 0;
	gimbal_can_send_data[4] = (0 >> 8);
	gimbal_can_send_data[5] = 0;
	gimbal_can_send_data[6] = (0 >> 8);
	gimbal_can_send_data[7] = 0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){};
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//拨弹盘电机
void CAN_cmd_trigger(int16_t trigger_motor)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  trigger_tx_message;

	trigger_tx_message.StdId = CAN_TRIGGER_MOTOR_ID;
	trigger_tx_message.IDE = CAN_ID_STD;
	trigger_tx_message.RTR = CAN_RTR_DATA;
	trigger_tx_message.DLC = 0x08;

	uint8_t trigger_can_send_data[8];
	trigger_can_send_data[0] = (0 >> 8);
	trigger_can_send_data[1] = 0;
	trigger_can_send_data[2] = (trigger_motor >> 8);
	trigger_can_send_data[3] = trigger_motor;
	trigger_can_send_data[4] = (0 >> 8);
	trigger_can_send_data[5] = 0;
	trigger_can_send_data[6] = (0 >> 8);
	trigger_can_send_data[7] = 0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){};

	HAL_CAN_AddTxMessage(&hcan1, &trigger_tx_message, trigger_can_send_data, &send_mail_box);
}

int mailFull_t = 0;

//摩擦轮电机电流 4->2
void CAN_cmd_shootFric(int16_t motor1, int16_t motor2, int16_t trigger_motor)
{
	CAN_TxHeaderTypeDef  shoot_tx_message;
  uint8_t shoot_can_send_data[8] = {0};
	
	uint32_t send_mail_box;
	shoot_tx_message.StdId = 0x200;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;

	shoot_can_send_data[0] = 0 >> 8;
	shoot_can_send_data[1] = 0;
	shoot_can_send_data[2] = motor1 >> 8;
	shoot_can_send_data[3] = motor1;
	shoot_can_send_data[4] = motor2 >> 8;
	shoot_can_send_data[5] = motor2;
	shoot_can_send_data[6] = trigger_motor >> 8;
	shoot_can_send_data[7] = trigger_motor;
  //用来监控 CAN 总线是否干扰大、是否掉线
	tec = hcan1.Instance->ESR & CAN_ESR_TEC;
	rec = (hcan1.Instance->ESR & CAN_ESR_REC) >> 8;
	 //等待 CAN 发送邮箱空闲
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){};
			//检测 CAN 总线是否离线（BOFF）
	if((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0)
	{
		mailFull_t  = 2;
	}
   HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
 if (result != HAL_OK) {
    // 检查具体错误原因
	 uint32_t error = HAL_CAN_GetError(&hcan1);
	if ((hcan1.Instance->TSR & CAN_TSR_TME0) == 0 && 
			(hcan1.Instance->TSR & CAN_TSR_TME1) == 0 && 
			(hcan1.Instance->TSR & CAN_TSR_TME2) == 0){
			// 所有邮箱都满了
		mailFull_t  = 1;
	}
 }
}

//将数据发给小电脑
void CAN_cmd_NUC(float PointX, float PointY, uint8_t state)
{
	CAN_TxHeaderTypeDef  shoot_tx_message;
	uint8_t shoot_can_send_data[8] = {0};
	
	uint32_t send_mail_box;
	shoot_tx_message.StdId = 0x120;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	
	int16_t pointx = PointX*1000;
	int16_t pointy = PointY*1000;
	
	shoot_can_send_data[0] = state;
	shoot_can_send_data[1] = 0;
	shoot_can_send_data[2] = pointx >> 8;
	shoot_can_send_data[3] = pointx;
	shoot_can_send_data[4] = pointy >> 8;
	shoot_can_send_data[5] = pointy;
	shoot_can_send_data[6] = 0;
	shoot_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &dji_motorbox[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &dji_motorbox[5];
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &dji_motorbox[8];
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &dji_motorbox[(i & 0x03)];
}

/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[2,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_shoot_frcitionmotor_measure_point(uint8_t i)
{
    return &dji_motorbox[4 + i];
}
