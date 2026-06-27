#include "SendBaseboard_task.h"
#include "can_receive.h"
#include "can.h"                         
#include "gimbal_task.h"
#include "arm_math.h"
#include "autoControl.h"
#include "shoot_task.h"
#include "gimbalControl.h"
#include "stdio.h"
#include "VTM03_Handler.h"

//TaskHandle define
TaskHandle_t NavReceive_taskHandle;
TaskHandle_t SendToNUC_taskHandle;
TaskHandle_t RS_taskHandle;

//QueueHandle define
QueueHandle_t NavReceive_queueHandle;

uint8_t VTM_buffer[256];

void Send_TaskInit(void)
{
	BaseType_t xRetruned;
  NavReceive_taskHandle=NULL;
	SendToNUC_taskHandle=NULL;
	RS_taskHandle=NULL;
	
	xRetruned = xTaskCreate(
		NavReceive_Task,
		"NavReceive_task",
		125,
		NULL,
		osPriorityHigh,
		&NavReceive_taskHandle
	);
	while(xRetruned!=pdPASS){};	
		
	xRetruned = xTaskCreate(
		VelTransToChassis_Task,
		"SendToNUC_task",
		125,
		NULL,
		osPriorityHigh,
		&SendToNUC_taskHandle
	);
	while(xRetruned!=pdPASS){};	

	xRetruned = xTaskCreate(
		RS_Task,
		"rs_task",
		125,
		NULL,
		osPriorityHigh,
		&RS_taskHandle
	);
	while(xRetruned!=pdPASS){};	
}

/**
* @brief            将IMU数据传输给下板（IMU数据*1000换成int16数据，下板再除1000）
  * @param[in]      pitch角度
  * @param[in]      yaw角度
  * @param[in]      roll角度
  * @retval         none
  */
void NAV_CAN_send_to_baseboard_chassis(int16_t pitch, int16_t yaw, int16_t roll)
{
	  CAN_TxHeaderTypeDef  nav_tx_message;
    uint32_t send_mail_box;
    nav_tx_message.StdId = CAN_SEND_TO_BASEBOARD_NAV_ID;
    nav_tx_message.IDE = CAN_ID_STD;
    nav_tx_message.RTR = CAN_RTR_DATA;
    nav_tx_message.DLC = 0x08;
	
	  uint8_t nav_can_send_data[8] = {0};
    nav_can_send_data[0] = pitch >> 8;//高位先行
    nav_can_send_data[1] = pitch;
    nav_can_send_data[2] = yaw >> 8;
    nav_can_send_data[3] = yaw;
    nav_can_send_data[4] = roll >> 8;
    nav_can_send_data[5] = roll;
    nav_can_send_data[6] = 0 >> 8;
    nav_can_send_data[7] = 0;
    while(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==0){};
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &nav_tx_message, nav_can_send_data, &send_mail_box);
}

/**
* @brief            将上板遥控器16位数据以12位长度发给下板
  * @param[in]      RC_ch0
  * @param[in]      RC_ch1
  * @param[in]      RC_roll
  * @param[in]      RC_ch2
  * @param[in]      RC_s1
  * @param[in]      RC_s2
  * @retval         none
  */
void RC_CAN_send_to_baseboard_chassis(int16_t RC_ch0, int16_t RC_ch1, int16_t RC_roll, int16_t RC_ch2, int16_t RC_s1, int16_t RC_s2)
{
	  CAN_TxHeaderTypeDef  chassis_tx_message;
    uint32_t send_mail_box;
	
    chassis_tx_message.StdId = CAN_SEND_TO_BASEBOARD_RC_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
	
		uint8_t chassis_can_send_data[8] = {0};

		// 处理ch0：高8位→data[0]，低4位→data[1]高4位
		int16_t temp_ch0 = RC_ch0;
		temp_ch0 = (temp_ch0 > 660) ? 660 : (temp_ch0 < -660 ? -660 : temp_ch0);
		uint16_t ch0_12bit = (uint16_t)(temp_ch0 & 0x0FFF);  // 仅保留12位有效数据
		chassis_can_send_data[0] = (uint8_t)(ch0_12bit >> 4);  // 高8位（bit4~11）
		chassis_can_send_data[1] = (uint8_t)((ch0_12bit & 0x000F) << 4);  // 低4位→高4位（bit4~7）

		// 处理ch1：高4位→data[1]低4位，低8位→data[2]
		int16_t temp_ch1 = RC_ch1;
		temp_ch1 = (temp_ch1 > 660) ? 660 : (temp_ch1 < -660 ? -660 : temp_ch1);
		uint16_t ch1_12bit = (uint16_t)(temp_ch1 & 0x0FFF);
		chassis_can_send_data[1] |= (uint8_t)(ch1_12bit >> 8);  // 高4位→低4位（bit0~3）
		chassis_can_send_data[2] = (uint8_t)(ch1_12bit & 0x00FF);  // 低8位（bit0~7）

		// 处理roller：高8位→data[3]，低4位→data[4]高4位
		int16_t temp_roll = RC_roll;
		temp_roll = (temp_roll > 660) ? 660 : (temp_roll < -660 ? -660 : temp_roll);
		uint16_t roll_12bit = (uint16_t)(temp_roll & 0x0FFF);
		chassis_can_send_data[3] = (uint8_t)(roll_12bit >> 4);  // 高8位（bit4~11）
		chassis_can_send_data[4] = (uint8_t)((roll_12bit & 0x000F) << 4);  // 低4位→高4位（bit4~7）

		// 处理ch2：高4位→data[4]低4位，低8位→data[5]
		int16_t temp_ch2 = RC_ch2;
		temp_ch2 = (temp_ch2 > 660) ? 660 : (temp_ch2 < -660 ? -660 : temp_ch2);
		uint16_t ch2_12bit = (uint16_t)(temp_ch2 & 0x0FFF);
		chassis_can_send_data[4] |= (uint8_t)(ch2_12bit >> 8);  // 高4位→低4位（bit0~3）
		chassis_can_send_data[5] = (uint8_t)(ch2_12bit & 0x00FF);  // 低8位（bit0~7）

		// 处理s1/s2
		chassis_can_send_data[6] = (uint8_t)((((RC_s1 & 0x03) << 4) & 0xF0) | (RC_s2 & 0x03));
		chassis_can_send_data[7] = 0;

    while(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==0){};
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
volatile int16_t x_speed, y_speed, z_speed;
uint8_t Control_mode = 0;
/**
  * @brief          接收小电脑发送下来的速度数据
  * @param[in]      none
  * @retval         none
  */
NavReceiveQueueData_t NavReceive_queueData;
void NavReceive_Task(void *argumemt)
{
	/* Creat a queue to receive the data of NUC */
	NavReceive_queueHandle = xQueueCreate(20, sizeof(NavReceiveQueueData_t));
	while(NavReceive_queueHandle == NULL){};

	BaseType_t xStatus;
	for(;;)
	{
		xStatus = xQueueReceive(NavReceive_queueHandle, &NavReceive_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(NavReceive_queueData.id)
			{
        case CAN_RECEIVE_FROM_NAV_CHASSIS_ID:{
					/* 数据处理 */
					Control_mode = NavReceive_queueData.rx_data[0];
					x_speed = (NavReceive_queueData.rx_data[2]<<8)|NavReceive_queueData.rx_data[3];
					y_speed = (NavReceive_queueData.rx_data[4]<<8)|NavReceive_queueData.rx_data[5];
					z_speed = (NavReceive_queueData.rx_data[6]<<8)|NavReceive_queueData.rx_data[7];
					/* 将数据转发给下板 */
					NUC_speed_send_to_baseBoard(x_speed, y_speed, z_speed, Control_mode);
				}break;
				default:{
//					x_speed = 0;
//					y_speed = 0;
//					z_speed = 0;
//					NUC_speed_send_to_baseBoard(x_speed, y_speed, z_speed, Control_mode);
				}
			}
		}
		else{
		}
	}
}

/**
  * @brief          发送信息给底盘
  * @param[in]      none
  * @retval         none
  */
uint8_t s2;
void VelTransToChassis_Task(void *argumemt)
{
//	CAN_TxHeaderTypeDef nuc_receive_message;
//  uint32_t send_mail_box;
//	uint8_t NUC_can_send_data[8] = {0};
		VTM_Handle_Init(&VTM_Handle, &huart6, SendToNUC_taskHandle, VTM_buffer, 256);
		uint32_t ulBitsToClearOnEntry = 0x00000000;
		uint32_t ulBitsToClearOnExit = 0xFFFFFFFF;
	  
	for(;;)
		{
			/* yaw速度环 */
//			printf("%f,%f\n", yaw_gimbal_motor.speed_control_data.set, yaw_gimbal_motor.speed_control_data.fdb);
			/* yaw角度环 */
//			printf("%f,%f,%f\n", yaw_gimbal_motor.position_control_data.set, yaw_gimbal_motor.basic_data.ref_angle, yaw_gimbal_motor.position_control_data.out);
			/* 拨弹盘速度环 */
//			printf("%f,%f\n", trigger_motor.speed_control_data.set, trigger_motor.speed_control_data.fdb);
			/* 摩擦轮速度环 */
//			printf("%f,%f,%f,%f\n", frictionWheel_motorHandle[0].speed_control_data.set, frictionWheel_motorHandle[0].speed_control_data.fdb
//														, frictionWheel_motorHandle[1].speed_control_data.set, frictionWheel_motorHandle[1].speed_control_data.fdb);
			xTaskNotifyWait(ulBitsToClearOnEntry,ulBitsToClearOnExit,0,portMAX_DELAY);
		  VTM_Recv_Process(&VTM_Handle);
			UpdateS2ByKeys(VTM_Handle.data.rc_left_key, VTM_Handle.data.rc_right_key, VTM_Handle.data.rc_stop, &s2);
			RC_CAN_send_to_baseboard_chassis(VTM_Handle.data.rc_ch0, VTM_Handle.data.rc_ch1, -VTM_Handle.data.rc_pause_button, VTM_Handle.data.rc_ch3, VTM_Handle.data.rc_s1, s2);
//			vTaskDelay(50);
		}
}

static uint8_t keyA_prev = 0;
static uint8_t keyB_prev = 0;
static uint8_t keyC_prev = 0;
void UpdateS2ByKeys(uint8_t keyA, uint8_t keyB, uint8_t keyC, uint8_t *s2)
{
    // 释放沿：prev=1 && now=0
    if (keyA_prev == 1 && keyA == 0) *s2 = 1;
    if (keyB_prev == 1 && keyB == 0) *s2 = 2;
    if (keyC_prev == 1 && keyC == 0) *s2 = 3;
    keyA_prev = keyA;
    keyB_prev = keyB;
    keyC_prev = keyC;
}

void RS_Task(void *argumemt)
{		

	 for(;;)
	{
		vTaskDelay(50);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	if(huart->Instance == USART6){
	  VTM_Event_Callback(&VTM_Handle, Size);
	}
}

void RS_SendToNuc(uint8_t progress, uint16_t current_hp, uint8_t robot_status, uint8_t disengage_status)
{
	CAN_TxHeaderTypeDef  shoot_tx_message;
	uint8_t shoot_can_send_data[5] = {0};
	
	uint32_t send_mail_box;
	shoot_tx_message.StdId = 0x400;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x05;
	
	shoot_can_send_data[0] = progress;
	shoot_can_send_data[1] = current_hp;
	shoot_can_send_data[2] = current_hp >> 8;
	shoot_can_send_data[3] = robot_status;
	shoot_can_send_data[4] = disengage_status;
	
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){};
  HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}
