/**
  ****************************(C) COPYRIGHT 2025 征途****************************
  * @file       ReceiveBaseboard_task.c/h
  * @brief      这里是上c板数据接收及裁判系统数据转发任务.（双板通信,主要用于接收电调，裁判系统，小电脑等数据）
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0                    chenzixi        
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "ReceiveBaseboard_task.h"
#include "can_receive.h"
#include "can.h"                         
#include "gimbal_task.h"
#include "arm_math.h"
#include "autoControl.h"
#include "shoot_task.h"
#include "SendBaseboard_task.h"
#include "rc_task.h"
#include "remoteControl.h"
#include "VTM03_Handler.h"

//RobotInfo Robot_All_Info;
NavReceiveQueueData_t Nav_queueData;
//rc thread
TaskHandle_t refereeSystem_taskHandle;				
void refereeSystem_Init(void)
{
  BaseType_t xRetruned;
	refereeSystem_taskHandle = NULL;
	
	xRetruned = xTaskCreate(
		RefereeSystemRetransmission_Task,
		"RefereeSystemRetransmission_task",//转发裁判系统数据到小电脑
		250,
		NULL,
		osPriorityHigh,
		&refereeSystem_taskHandle
	);
	while(xRetruned!=pdPASS){};	

}
refereeSystemQueueData_t refreeSystem_queueData;
uint16_t heat, heat_limit;
void RefereeSystemRetransmission_Task(void *argument)
{
	BaseType_t xStatus;
	refereeSystem_queueHandle = xQueueCreate(40, sizeof(refereeSystemQueueData_t));
	while(refereeSystem_queueHandle == NULL){};
	uint8_t progress;
	uint16_t current_hp;
	uint8_t robot_status;
	int16_t hp;
  for(;;)
	{
		xStatus = xQueueReceive(refereeSystem_queueHandle, &refreeSystem_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(refreeSystem_queueData.id)
			{
				case 0x500:{
					 progress = refreeSystem_queueData.rx_data[0];
					 current_hp = (refreeSystem_queueData.rx_data[1] << 8) | refreeSystem_queueData.rx_data[2];
					 robot_status = refreeSystem_queueData.rx_data[3];
					 heat = (refreeSystem_queueData.rx_data[4] << 8) | (refreeSystem_queueData.rx_data[5]);
					 heat_limit = (refreeSystem_queueData.rx_data[6]) << 8 | (refreeSystem_queueData.rx_data[7]);
				}break;
				default:{
				}
			}
		}
		else{
		}
		/* 裁判系统数据模拟代码，比赛前删去 */
		if((VTM_Handle.data.rc_s1 == 0) || (VTM_Handle.data.rc_s1 == 2)){
			hp =VTM_Handle.data.rc_ch2 + 600;
			if(hp>=600)hp=600;
			if(hp<=0)hp=0;
			if(s2 == 1){
			  RS_SendToNuc(4, hp, 1, disengage);
			}else if(s2 == 3){
				RS_SendToNuc(3, hp, 1, disengage);
			}else if(s2 == 2){
				RS_SendToNuc(4, 150, 1, disengage);
			}
		}else{
				RS_SendToNuc(progress, current_hp, robot_status, disengage);//保留
		}
	  osDelay(100);
	}
}

//void rs_data_process(refereeSystemQueueData_t *rs_data)
//{
//  
//}

//这里都是值判断canid然后把不同类型的数据数组发队列到对应的任务处理
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	BaseType_t xHigherPriorityTaskWorken = pdFALSE;
	BaseType_t xsendStatus;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan->Instance == CAN2){//can2用于双板通信，接收下板裁判系统数据（枪管热量，血量）
		switch (rx_header.StdId)
		{
	   //接收下板裁判系统数据
      case 0x500:{
        refereeSystemQueueData_t refreeSystem_queueData;
        refreeSystem_queueData.id = rx_header.StdId;
        memcpy(refreeSystem_queueData.rx_data, rx_data, 8*sizeof(uint8_t));
			  if(refereeSystem_queueHandle != NULL){
				  xsendStatus = xQueueSendFromISR(refereeSystem_queueHandle, &refreeSystem_queueData, &xHigherPriorityTaskWorken);
					if(xsendStatus==!pdPASS){}
        }
      }break;			
		}
  }
	else if(hcan->Instance == CAN1){
	  switch (rx_header.StdId)
	  {
		//双摩擦轮电机电调数据接收
		case 0x202:
		case 0x203:{
	    ShootMotorReceiveQueueData_t shootMotor_queueData;
			shootMotor_queueData.id = rx_header.StdId;
			memcpy(shootMotor_queueData.rx_data, rx_data, 8*sizeof(uint8_t));			
			if(shootMotorReceive_queueHandle != NULL){
			  xsendStatus = xQueueSendFromISR(shootMotorReceive_queueHandle, &shootMotor_queueData, &xHigherPriorityTaskWorken);
			}
	  }break;
		//云台电机电调数据接收（小yaw轴控制用）
    case CAN_YAW_MOTOR_ID:{
        GimbalMotorReceiveQueueData_t gimbalYawMotor_queueData;
        gimbalYawMotor_queueData.id = rx_header.StdId;
        memcpy(gimbalYawMotor_queueData.rx_data, rx_data, 8*sizeof(uint8_t));
        if(gimbalYawMotorReceive_queueHandle != NULL){
			    xsendStatus = xQueueSendFromISR(gimbalYawMotorReceive_queueHandle, &gimbalYawMotor_queueData, &xHigherPriorityTaskWorken);
			  }
      }break;
		//拨弹盘电机电调数据接收
		case 0x204:{
	    TriggerMotorReceiveQueueData_t triggerMotor_queueData;
			triggerMotor_queueData.id = rx_header.StdId;//id填充
			memcpy(triggerMotor_queueData.rx_data, rx_data, 8*sizeof(uint8_t));//数据填充
			if(triggerMotorReceive_queueHandle != NULL){
			  xsendStatus = xQueueSendFromISR(triggerMotorReceive_queueHandle, &triggerMotor_queueData, &xHigherPriorityTaskWorken);//发队列
			}
	  }break;
		//小电脑航向数据接收（导航用）
	  case NAV_CAN_REV_ID:{
			
			Nav_queueData.id = rx_header.StdId;
			memcpy(Nav_queueData.rx_data, rx_data, 8*sizeof(uint8_t));
			if(NavReceive_queueHandle != NULL){
				xsendStatus = xQueueSendFromISR(NavReceive_queueHandle, &Nav_queueData, &xHigherPriorityTaskWorken);//创建一个队列接收任务用来获取雷达数据
			}
		}break;
	  //达妙电机（pitch轴）数据接收
	 case CAN_PITCH_MOTOR_ID:{
	      GimbalPitchMotorReceiveQueueData_t gimbalPitchMotor_queueData;
		    gimbalPitchMotor_queueData.id = rx_header.StdId;
				memcpy(gimbalPitchMotor_queueData.rx_data, rx_data, 8*sizeof(uint8_t));
		    if(gimbalPitchMotorReceive_queueHandle != NULL){
				  xsendStatus = xQueueSendFromISR(gimbalPitchMotorReceive_queueHandle, &gimbalPitchMotor_queueData, &xHigherPriorityTaskWorken);
				}
	 }break;
		//自瞄数据接收	 
	 case AIM_CAN_REV_ID:{
			GimbalAimReceiveQueueData_t gimbalAim_queueData;
			gimbalAim_queueData.id = rx_header.StdId;
			memcpy(gimbalAim_queueData.rx_data, rx_data, 8*sizeof(uint8_t));
			if(gimbalAimReceive_queueHandle != NULL){
				xsendStatus = xQueueSendFromISR(gimbalAimReceive_queueHandle, &gimbalAim_queueData, &xHigherPriorityTaskWorken);
			}
		}break; 

	  }
	}
}

