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

#include "bsp_dji.h"
#include "can.h"                         
#include "receive_task.h"
#include "arm_math.h"
#include "4310_control.h"
#include "nav_task.h"
#define SUPERPOWER_CAN_ID 0x2FF
#define CAN_RECEIVE_CHASSIS_ID 0x300
#define CAN_SEND_TO_BASEBOARD_NAV_ID 0x200
#define CAN_power_ID 0x2FF

/*
电机数据:
0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机;
6:摩擦轮电机1 3508电机; 7:摩擦轮电机2 3508电机;
8:拨弹盘电机 2006电机
总：
*** 0,1,2,3为底盘电机 *** 4,5为云台电机 *** 6,7为摩擦轮电机 *** 8为拨弹盘电机 ***
*/
motor_measure_t dji_motorbox[DJI_MOTOR_NUMBER];

uint8_t i;
uint8_t testCanArr[8] = {0};
int16_t count=0;
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**

  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
/******************* CAN1-FIFO0中断回调 *********************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//仅处理can1的FIFO0中断
	if(hcan != &hcan1)return;
	
	CAN_RxHeaderTypeDef CAN1_rx_header;
	uint8_t can1_rx_data[8];
	HAL_StatusTypeDef read_status;
	
	//1.检查FIFO0溢出
	if(__HAL_CAN_GET_FLAG (hcan, CAN_FLAG_FOV0)!=RESET)
	{
		__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0);
		return;
	}
	
	//2.读取FIF00消息(检查返回值，避免读取失败)
	read_status=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_rx_header, can1_rx_data);
	if(read_status!=HAL_OK)return;
	
	BaseType_t xHigherPriorityTaskWorken = pdFALSE;
	BaseType_t xsendStatus;

	if(hcan->Instance == CAN1)//can1的通讯包含了底盘电机电调的通讯，世界坐标系数据获取（改到can2，接收上板陀螺仪数据），超级电容数据获取。
		{
				switch (CAN1_rx_header.StdId)
				{
						/* 基础电机数据获取 */
					case CAN_3508_M1_ID:
					case CAN_3508_M2_ID:
					case CAN_3508_M3_ID:
					case CAN_3508_M4_ID:{
							ChassisMotorReceiveQueueData_t chassisMotor_queueData;
							chassisMotor_queueData.id = CAN1_rx_header.StdId;
							memcpy(chassisMotor_queueData.rx_data, can1_rx_data, 8*sizeof(uint8_t));
							if(chassisMotorReceive_queueHandle != NULL)
							{
								xsendStatus = xQueueSendFromISR(chassisMotorReceive_queueHandle, &chassisMotor_queueData, &xHigherPriorityTaskWorken);  
								        if(xsendStatus != pdPASS) {
             // 队列满，数据丢失 - 可选：记录或增加计数器
             // 这里可以留空，但至少知道可能有问题
                                 }
							}
						}break;
				  				/* J4310电机数据获取 */
	   case CAN_YAW_CONTROL:{
	      ChassisYawMotorReceiveQueueData_t  YAWMotor_queueData;
		    YAWMotor_queueData.id = CAN1_rx_header.StdId;
				memcpy(YAWMotor_queueData.rx_data, can1_rx_data, 8*sizeof(uint8_t));
		    if(YAWMotorReceive_queueHandle != NULL){
				  xsendStatus = xQueueSendFromISR(YAWMotorReceive_queueHandle, &YAWMotor_queueData, &xHigherPriorityTaskWorken);
				}
	 }break;
					  case CAN_power_ID:
					 {
						 ChassisPOWERReceiveQueueData_t ChassisPOWERReceive_queueDate;
						 ChassisPOWERReceive_queueDate.id = CAN1_rx_header.StdId;
						 memcpy(ChassisPOWERReceive_queueDate.rx_data, can1_rx_data, 8*sizeof(uint8_t));
						 if(chassisPOWERReceive_queueHandle != NULL){
								//如果接收方的队列成功创建，就发送数据过去（接收方队列再chassis）
								xsendStatus = xQueueSendFromISR(chassisPOWERReceive_queueHandle, &ChassisPOWERReceive_queueDate, &xHigherPriorityTaskWorken);
							 																	        if(xsendStatus != pdPASS) {

                                 }
						 }						 
					 }  break;

				}

		}
	/* 所有xQueueSendFromISR执行完毕后再统一判断一次是否需要切任务，
	   否则被唤醒的任务要等下一次tick(1ms@1000Hz)才切换 */
	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}


CAN_RxHeaderTypeDef CAN2_rx_header;
uint8_t can2_rx_data[8];

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		//仅处理can2的FIFO1中断
	if(hcan != &hcan2)return;

	HAL_StatusTypeDef read_status;
	
	//1.检查FIFO1溢出
	if(__HAL_CAN_GET_FLAG (hcan, CAN_FLAG_FOV1)!=RESET)
	{
		__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1);
		return;
	}
	
	//2.读取FIF01消息(检查返回值，避免读取失败)
	read_status=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_rx_header, can2_rx_data);
	if(read_status!=HAL_OK)return;
	
	BaseType_t xHigherPriorityTaskWorken = pdFALSE;
	BaseType_t xsendStatus;

		if(hcan->Instance == CAN2)//can2的通讯包含了下c板电机目标速度的传输。
		{
			 switch (CAN2_rx_header.StdId)
			 {
				 /* 上板遥控接收 */
				 case CAN_RECEIVE_CHASSIS_ID:
					 {
						 ChassisReceiveFromAdvQueueData_t chassisReceiveAdv_queueData;
						 chassisReceiveAdv_queueData.id = CAN2_rx_header.StdId;//id数据填充
						 memcpy(chassisReceiveAdv_queueData.rx_data, can2_rx_data, 8*sizeof(uint8_t));//把接收到的数据copy到实例的数据部分里
						 if(chassisMotorReceiveAdv_queueHandle != NULL){
								//如果接收方的队列成功创建，就发送数据过去（接收方队列再chassis）
								xsendStatus = xQueueSendFromISR(chassisMotorReceiveAdv_queueHandle, &chassisReceiveAdv_queueData, &xHigherPriorityTaskWorken);
							         if(xsendStatus != pdPASS) {
            // 队列满，数据丢失 - 可选：记录或增加计数器
            // 这里可以留空，但至少知道可能有问题
                    }
						 }
					 }
				 break;
				 /* 陀螺仪数据接收 */
				 case CAN_SEND_TO_BASEBOARD_NAV_ID:
					 {
						 YAWIMUReceiveQueueData_t YAWReceiveIMU_queueData;
						 YAWReceiveIMU_queueData.id = CAN2_rx_header.StdId;
						 memcpy(YAWReceiveIMU_queueData.rx_data, can2_rx_data, 8*sizeof(uint8_t));
						 if(YAWImuReceive_queueHandle != NULL){
								//如果接收方的队列成功创建，就发送数据过去（接收方队列再chassis）
								xsendStatus = xQueueSendFromISR(YAWImuReceive_queueHandle, &YAWReceiveIMU_queueData, &xHigherPriorityTaskWorken);
						 }						 
					 }
					break;
					/* 导航数据接收 */
					case NAV_CAN_REV_ID:
					  {
							ChassisNavReceiveQueueData_t chassisNav_queueData;
							chassisNav_queueData.id = CAN2_rx_header.StdId;
							if(chassisNav_queueData.id == 0x110){
							  memcpy(chassisNav_queueData.rx_data, can2_rx_data, 8*sizeof(uint8_t));							  
							}
							if(Nav2chassisReceive_queueHandle != NULL){
								xsendStatus = xQueueSendFromISR(Nav2chassisReceive_queueHandle, &chassisNav_queueData, &xHigherPriorityTaskWorken);
							}
						}
					 break; 

					default:{
				    }
				 break;
			 }
		}
	/* 所有xQueueSendFromISR执行完毕后再统一判断一次是否需要切任务，
	   否则被唤醒的任务要等下一次tick(1ms@1000Hz)才切换 */
	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}

uint16_t tec = 0;
uint16_t rec = 0;

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    CAN_TxHeaderTypeDef  chassis_tx_message;
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
	
	uint8_t chassis_can_send_data[8] = {0};
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          发送底盘电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
HAL_StatusTypeDef state;
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	/* 创建can报文头部信息结构体实例 */
	CAN_TxHeaderTypeDef  chassis_tx_message;
  uint32_t send_mail_box;
	/* 填充can报文的头部信息TxHeader */
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;//0x200
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
	/* 数组数据填充 */
	  uint8_t chassis_can_send_data[8] = {0};
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    uint32_t chassis_tx_timeout = 0xFFFF;
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){//发送数据前先判断是否有空闲邮箱，避免因邮箱满而导致的发送失败
        if(--chassis_tx_timeout == 0) return;//超时放弃，避免邮箱满时死等卡死整个任务
    }
    state = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);//发送数据
}
  

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &dji_motorbox[(i & 0x03)];//&0x03就相当于只取0011底两位数据为有效数据
}
