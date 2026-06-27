//										#include "gimbal_task.h"
//#include "gimbalcontrol.h"
//#include "remoteControl.h"
//#include "autoControl.h"
//#include "driver_4310.h"
//#include "stdlib.h"
//#include "SendBaseboard_task.h"
//#include "VTM03_Handler.h"
//#include "ReceiveBaseboard_task.h"
//#include "4310_apply.h"
//#include "stdio.h"
//#define AUTOAIM_HOLD_MS 1000

///*define thread handle*/
//TaskHandle_t gimbal_taskHandle;				// main thread
//TaskHandle_t gimbalMotorReceive_taskHandle;	// the branch thread to receive yaw motor data
//TaskHandle_t gimbalImuReceive_taskHandle;	// the branch thread to receive imu data
//TaskHandle_t gimbalAimReceive_taskHandle;	// the branch thread to receive chassis motor data
//TaskHandle_t gimbalPitchReceive_taskHandle;// the branch thread of receive pitch motor data
///*define imu handle*/
//JY901_DataHandleTypedef  gimbalJY901_imuHandle;

///*define queue handle*/
//QueueHandle_t gimbalYawMotorReceive_queueHandle;
//QueueHandle_t gimbalIMUReceive_queueHandle;
//QueueHandle_t gimbalAimReceive_queueHandle;
//QueueHandle_t gimbalPitchMotorReceive_queueHandle;
//QueueHandle_t refereeSystem_queueHandle;

//float yaw_err;

//uint8_t zero_set=1;
///**
//  * @brief          chassis main thread 创建任务
//  * @param[in]      none
//  * @retval         none
//  */
//void Gimbal_TaskInit(void)
//{
//	BaseType_t xRetruned;
//	gimbal_taskHandle = NULL;
//	gimbalMotorReceive_taskHandle = NULL;
//	// chassisImuReceive_taskHandle = NULL;
//	
//	xRetruned = xTaskCreate(Gimbal_Task,"gimbal_task",250,NULL,osPriorityHigh,&gimbal_taskHandle);
//	while(xRetruned!=pdPASS){};	

//	xRetruned = xTaskCreate(GimbalMotorReceive_Task,"gimbal_motor_receive_task",125,NULL,osPriorityRealtime,&gimbalMotorReceive_taskHandle);
//	while(xRetruned!=pdPASS){};

//	xRetruned = xTaskCreate(
//		GimbalAimReceive_Task,
//		"gimbal_aim_receive_task",
//		125,
//		NULL,
//		osPriorityRealtime,
//		&gimbalAimReceive_taskHandle
//	);
//	while(xRetruned!=pdPASS){};
//		
// 	xRetruned = xTaskCreate(
//        GimbalImuReceive_Task,
//        "gimbal_imu_receive_task",
//        125,
//        NULL,
//        osPriorityRealtime,
//        &gimbalImuReceive_taskHandle
//    );
//    while(xRetruned!=pdPASS){}    
//			
// 	xRetruned = xTaskCreate(
//        GimbalPitchReceive_Task,
//        "gimbal_pitch_receive_task",
//        100,
//        NULL,
//        osPriorityRealtime2,
//        &gimbalPitchReceive_taskHandle
//    );
//    while(xRetruned!=pdPASS){}
//}

//void Gimbal_Task(void *argument)
//{
//	Gimbal_Init();
//	AutoControl_Init();
//	
//	float autoAimYawSetAngle=yaw_offset;
//	float autoAimPitchSetAngle=pitch_offset;
//	float autoAimPitchGetAngle=pitch_offset,autoAimYawGetAngle=yaw_offset;
//	VTM_Handle.data.rc_s1=1;
//	s2=1;
//	
//	static uint32_t auto_aim_active=0;
//	static uint8_t last_autoaim_time=0;
//	
//	for(;;)
//	{
//		M6020_GetBasicData(&yaw_gimbal_motor);
//		yaw_gimbal_motor.basic_data.ref_angle = ((yaw_gimbal_motor.basic_data.ecd / 8192.0f) * 360.0f) - 180.0f;
//		motor1_init(&DM_4310);
//		
//		switch (VTM_Handle.data.rc_s1){
//			case 1:
//         {
//					 if(s2==1)
//					 {
//						 Yaw_Motor_Control(yaw_offset);
//						 pitch_spepos_control(pitch_offset);
//					 }
//					 else if(s2==2)
//					 {
//							static uint8_t auto_aim_has_received = 0;
//						 uint8_t auto_aim_state=GetAutoAIM();
//						 if(auto_aim_state==AUTO_AIM_GET)
//						 {
//							 auto_aim_has_received=1;
//							 auto_aim_active=1;
//							 last_autoaim_time=HAL_GetTick();
//							 
//							 autoAimPitchGetAngle=DM_4310.para.pos;
//							 autoAimYawGetAngle=yaw_gimbal_motor.basic_data.ref_angle;
//							 autoAimYawSetAngle= autoAimYawGetAngle -(auto_aim.datafloat.d_yaw+2);
//							 autoAimPitchSetAngle=autoAimPitchGetAngle+ (auto_aim.datafloat.d_pitch+2);
//							 
//							 Float_Limit(autoAimYawSetAngle,YAW_MIN_ANGLE_DEG,YAW_MAX_ANGLE_DEG);
//							 Float_Limit(autoAimPitchSetAngle,PITCH_MIN,PITCH_MAX);
//							 
//							 Yaw_Motor_Control(autoAimYawSetAngle);
//							 pitch_spepos_control(autoAimPitchSetAngle);
//						 }
//						 else if(auto_aim_has_received&&auto_aim_state==AUTO_AIM_LOSE)
//						 {
//							 pitch_con();
//							 yaw_con();
//						 }
//						 else if(auto_aim_has_received==0)
//						 {
//							 auto_aim_active=0;
//							 Yaw_Motor_Control(yaw_offset);
//							 pitch_spepos_control(pitch_offset);
//						 }
//						 if(auto_aim_active&&HAL_GetTick()-last_autoaim_time>AUTO_AIM_TIMEOUT_MS)
//						 {
//							 auto_aim_active=0;
//						 }
//					 }
//					 else if(s2==3)
//					 {
//						 
//					 }
//				 }break;
//			case 0:{}
//		}
//	}
//}

//void GimbalMotorReceive_Task(void *argument)
//{
//	gimbalYawMotorReceive_queueHandle=xQueueCreate(40,sizeof(GimbalMotorReceiveQueueData_t));
//	configASSERT(gimbalYawMotorReceive_queueHandle!=NULL);
//	
//	BaseType_t xStatus;
//	GimbalMotorReceiveQueueData_t gimbalMotor_queueData;
//	for (;;)
//	{
//		xStatus=xQueueReceive(gimbalYawMotorReceive_queueHandle,&gimbalMotor_queueData,portMAX_DELAY);
//		if(xStatus!=pdFALSE)
//		{
//			switch(gimbalMotor_queueData.id-CAN_CHASSIS_ALL_ID)
//			{
//				case 0x05:{
//					get_motor_measure(&dji_motorbox[gimbalMotor_queueData.id-CAN_CHASSIS_ALL_ID-1],gimbalMotor_queueData.rx_data);
//					
//					
//				} break;
//				default:{
//				return;
//				}
//			}
//		}
//		
//	}
//}
//}