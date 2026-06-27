#include "gimbal_task.h"
#include "gimbalcontrol.h"
#include "remoteControl.h"
#include "autoControl.h"
#include "driver_4310.h"
#include "stdlib.h"
#include "SendBaseboard_task.h"
#include "VTM03_Handler.h"
#include "ReceiveBaseboard_task.h"
#include "4310_apply.h"
#include "stdio.h"
#define AUTOAIM_HOLD_MS 1000

/*define thread handle*/
TaskHandle_t gimbal_taskHandle;				// main thread
TaskHandle_t gimbalMotorReceive_taskHandle;	// the branch thread to receive yaw motor data
TaskHandle_t gimbalImuReceive_taskHandle;	// the branch thread to receive imu data
TaskHandle_t gimbalAimReceive_taskHandle;	// the branch thread to receive chassis motor data
TaskHandle_t gimbalPitchReceive_taskHandle;// the branch thread of receive pitch motor data
/*define imu handle*/
JY901_DataHandleTypedef  gimbalJY901_imuHandle;

/*define queue handle*/
QueueHandle_t gimbalYawMotorReceive_queueHandle;
QueueHandle_t gimbalIMUReceive_queueHandle;
QueueHandle_t gimbalAimReceive_queueHandle;
QueueHandle_t gimbalPitchMotorReceive_queueHandle;
QueueHandle_t refereeSystem_queueHandle;

float yaw_err;

uint8_t zero_set=1;
/**
  * @brief          chassis main thread 鍒涘缓浠诲姟
  * @param[in]      none
  * @retval         none
  */
void Gimbal_TaskInit(void)
{
	BaseType_t xRetruned;
	gimbal_taskHandle = NULL;
	gimbalMotorReceive_taskHandle = NULL;
	// chassisImuReceive_taskHandle = NULL;
	
	xRetruned = xTaskCreate(Gimbal_Task,"gimbal_task",250,NULL,osPriorityHigh,&gimbal_taskHandle);
	while(xRetruned!=pdPASS){};	

	xRetruned = xTaskCreate(GimbalMotorReceive_Task,"gimbal_motor_receive_task",125,NULL,osPriorityRealtime,&gimbalMotorReceive_taskHandle);
	while(xRetruned!=pdPASS){};

	xRetruned = xTaskCreate(
		GimbalAimReceive_Task,
		"gimbal_aim_receive_task",
		125,
		NULL,
		osPriorityRealtime,
		&gimbalAimReceive_taskHandle
	);
	while(xRetruned!=pdPASS){};
		
 	xRetruned = xTaskCreate(
        GimbalImuReceive_Task,
        "gimbal_imu_receive_task",
        125,
        NULL,
        osPriorityRealtime,
        &gimbalImuReceive_taskHandle
    );
    while(xRetruned!=pdPASS){}    
			
 	xRetruned = xTaskCreate(
        GimbalPitchReceive_Task,
        "gimbal_pitch_receive_task",
        100,
        NULL,
        osPriorityRealtime2,
        &gimbalPitchReceive_taskHandle
    );
    while(xRetruned!=pdPASS){}
}

/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
float smc_out;

float j4310_pid_speed = 0;
int16_t yaw_output = 0;
int16_t trigger_vel;
int16_t err;
int16_t aim_lose=0;
uint8_t disengage;
void Gimbal_Task(void *argument)                 
{
	Gimbal_Init();
	AutoControl_Init();

//	 float autoAimYawSetAngle =yaw_offset;
//	float  autoAimPitchSetAngle =pitch_offset;
//	float autoAimPitchGetAngle=pitch_offset, autoAimYawGetAngle=yaw_offset;
//float yaw_error;
//	uint8_t flag=1;

//		 VTM_Handle.data.rc_s1=1;
//		 s2=1;
//	uint8_t aim_lose;
//	static uint32_t autoaim_last_seen_time = 0;
//  static uint8_t autoaim_hold_enable = 0;
//  static uint8_t auto_aim_active = 0;
//	static uint32_t last_autoaim_time = 0;
	for(;;)
	{
		M6020_GetBasicData(&yaw_gimbal_motor);
    	    yaw_gimbal_motor.basic_data.ref_angle =
        ((yaw_gimbal_motor.basic_data.ecd / 8192.0f) * 360.0f) - 180.0f;
		
		motor1_init(&DM_4310);

/*
		switch(VTM_Handle.data.rc_s1){
				case 1:{
					if(s2 == 1){
//						yaw_con();
//					pitch_con();
            Yaw_Motor_Control(yaw_offset);
            pitch_spepos_control(pitch_offset);
					}
					else if(s2 == 2){
						static uint8_t auto_aim_has_received = 0;
						uint8_t auto_aim_state = GetAutoAIM();

						if(auto_aim_state == AUTO_AIM_GET){
							
							auto_aim_has_received = 1;
							auto_aim_active = 1;
							last_autoaim_time = HAL_GetTick();

							autoAimPitchGetAngle = DM_4310.para.pos;

							autoAimYawGetAngle = yaw_gimbal_motor.basic_data.ref_angle;
							autoAimYawSetAngle = autoAimYawGetAngle - (auto_aim.datafloat.d_yaw + 2);
							autoAimPitchSetAngle = autoAimPitchGetAngle + (auto_aim.datafloat.d_pitch + 2);//2->	
							
             autoAimYawSetAngle= Float_Limit(autoAimYawSetAngle,YAW_MIN_ANGLE_DEG,YAW_MAX_ANGLE_DEG);
						 autoAimPitchSetAngle = Float_Limit(autoAimPitchSetAngle,PITCH_MIN,PITCH_MAX);

							Yaw_Motor_Control(autoAimYawSetAngle);
							pitch_spepos_control(autoAimPitchSetAngle);
						}
						else if(auto_aim_has_received && auto_aim_state == AUTO_AIM_LOSE)
						{
							pitch_con();
							yaw_con();
						}
						else if(auto_aim_has_received == 0)
						{
							auto_aim_active = 0;
							Yaw_Motor_Control(yaw_offset);
							pitch_mit_control(pitch_offset);
						}

						if(auto_aim_active &&
						   HAL_GetTick() - last_autoaim_time > AUTO_AIM_TIMEOUT_MS)
						{
							auto_aim_active = 0; 
						}
					}
					else if(s2 == 3){
       pitch_con();
//				pitch_spepos_control(pitch_offset);
				yaw_con();
//         pitch_spepos_control(0);
//						Yaw_Motor_Control(yaw_offset);
					}

					
				}break;
				case 0:{
//					uint32_t now_time = HAL_GetTick();
//					if(s2 == 1){//
//							if(flag==1){

//								}

//					}
//					else if((s2 == 3) || (Control_mode == 0)){

////							autoaim_hold_enable = 0;
//							ammo_dial_anti_block_core(-0, 0);
//            mit_motor_position_with_gravity_deg(-45.0f,12.0f);
//			     Yaw_Motor_Control(85.0f);
//					}
//					else if(s2 == 2){
////							autoaim_hold_enable = 0;

//					}
				}break;

			case 2:{

					if(s2 == 3){

//			     Yaw_Motor_Control(85.0f);
						if(VTM_Handle.data.rc_trigger == 1){
							ammo_dial_anti_block_with_heat_limit(heat, heat_limit, -5500, 4800);
						}else{
							ammo_dial_anti_block_with_heat_limit(heat, heat_limit, -5500, 0);
						}
					}
					else if(s2 == 2){
          mit_motor_position_with_gravity_deg(-45.0f,12.0f);
			     Yaw_Motor_Control(85.0f);
						ammo_dial_anti_block_with_heat_limit(heat, heat_limit, 0, 0);
					}
					else if(s2 == 1){
          							yaw_con();
												pitch_con();
					}
				}break;

		}
*/
		if (RC_Ctl.rc.s1 == RC_SW_UP)
		{
			Yaw_Motor_Control(yaw_offset);
			pitch_spepos_control(pitch_offset);
		}
		else
		{
			float remote_yaw_target = Float_Limit((float)rc_user.yaw,
			                                      YAW_MIN_ANGLE_DEG,
			                                      YAW_MAX_ANGLE_DEG);
			float remote_pitch_target = Float_Limit((float)rc_user.pitch,
			                                        PITCH_MIN,
			                                        PITCH_MAX);

			Yaw_Motor_Control(remote_yaw_target);
			pitch_spepos_control(remote_pitch_target);
		}
		vTaskDelay(1);
	}
}

void GimbalMotorReceive_Task(void *argumemt)
{
		gimbalYawMotorReceive_queueHandle = xQueueCreate(40, sizeof(GimbalMotorReceiveQueueData_t));
   configASSERT(gimbalYawMotorReceive_queueHandle != NULL);
	  BaseType_t xStatus;
	
	GimbalMotorReceiveQueueData_t gimbalMotor_queueData;
	for(;;)
  {
    xStatus=xQueueReceive(gimbalYawMotorReceive_queueHandle,&gimbalMotor_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
    {
			switch(gimbalMotor_queueData.id - CAN_CHASSIS_ALL_ID)//0x205-0x200
			{
       case 0x05:{
			 get_motor_measure(&dji_motorbox[gimbalMotor_queueData.id-CAN_CHASSIS_ALL_ID - 1],gimbalMotor_queueData.rx_data);
			 }break;
			 default:{}
			}
		}
		
	}
}
}
UBaseType_t uxHighWaterMark;

void GimbalImuReceive_Task(void *argument)
{
    for(;;)
    {
							static uint32_t last_print = 0;
        if(HAL_GetTick() - last_print > 100)
        {
            last_print = HAL_GetTick();
//							printf("%f",auto_aim.datafloat.d_yaw);
//          printf("%d,%d\n",frictionWheel_motorHandle[0].basic_data.speed_rpm,frictionWheel_motorHandle[1].basic_data.speed_rpm);
					printf("%d,%d,%f,%f\n",yaw_gimbal_motor.basic_data.given_current,safe_current,yaw_set,yaw_gimbal_motor.basic_data.ref_angle);
//					printf("%f,%f\n",DM_4310.para.vel,DM_4310.para.pos);
//					printf("POSPout=%f,POSIout=%f",yaw_gimbal_motor.position_control_data.Pout,yaw_gimbal_motor.position_control_data.Iout);
//					printf("SPEKP=%f,SPEKI=%f",yaw_gimbal_motor.speed_control_data.Pout,yaw_gimbal_motor.speed_control_data.Iout);
        }
        
        osDelay(5);  
			
    }
}


GimbalMotorReceiveQueueData_t gimbalPitchMotor_queueData;
void GimbalPitchReceive_Task(void *argumemt)
{
	gimbalPitchMotorReceive_queueHandle = xQueueCreate(40, sizeof(GimbalMotorReceiveQueueData_t));
	while(gimbalPitchMotorReceive_queueHandle == NULL){};
	BaseType_t xStatus;

	for(;;)
	{
		xStatus = xQueueReceive(gimbalPitchMotorReceive_queueHandle, &gimbalPitchMotor_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(gimbalPitchMotor_queueData.id)
			{
				case 0x00:{
					dm_motor_fbdata(&DM_4310, (uint8_t *)gimbalPitchMotor_queueData.rx_data);
				}break;
				default:{
				}
			}
		}
		else{
		}
	}
}
	
GimbalAimReceiveQueueData_t gimbalAim_queueData;
void GimbalAimReceive_Task(void *argument)
{
    /* 鍒涘缓闃熷垪 */
    gimbalAimReceive_queueHandle =
        xQueueCreate(40, sizeof(GimbalAimReceiveQueueData_t));
    configASSERT(gimbalAimReceive_queueHandle);

    BaseType_t xStatus;
    GimbalAimReceiveQueueData_t queueData;

    for (;;)
    {
        /* 闃诲绛夊緟瑙嗚鏁版嵁 */
        xStatus = xQueueReceive(
            gimbalAimReceive_queueHandle,
            &queueData,
            portMAX_DELAY
        );
        if (xStatus != pdPASS)
            continue;

        if (queueData.id != AIM_CAN_REV_ID)
            continue;

         //1. 瑙ｆ瀽鍘熷閫氫俊鏁版嵁锛坕nt锛?
        int16_t raw_yaw   = (int16_t)(queueData.rx_data[0] << 8 | queueData.rx_data[1]);
        int16_t raw_pitch = (int16_t)(queueData.rx_data[2] << 8 | queueData.rx_data[3]);
        uint16_t raw_aim   = (uint16_t)(queueData.rx_data[4] << 8 | queueData.rx_data[5]);
        uint16_t raw_shoot = (uint16_t)(queueData.rx_data[6] << 8 | queueData.rx_data[7]);
         //2. 淇濆瓨涓婁竴娆℃湁鏁堟暟鎹紙婊ゆ尝鐢級
        auto_aim.datafloat.last_d_yaw   = auto_aim.datafloat.d_yaw;
        auto_aim.datafloat.last_d_pitch = auto_aim.datafloat.d_pitch;
         //3. 瑁呯敳鏉胯瘑鍒姸鎬佹満
        if (raw_aim == 1)
        {
            auto_aim.aim_signal = AUTO_AIM_GET;

            /* 閫氱煡搴曟澘锛氳繘鍏ヨ嚜鍔ㄧ瀯鍑?*/
            AutoAim_SendToYaw_baseBoard(1);
        }
        else
        {
            auto_aim.aim_signal = AUTO_AIM_LOSE;
        }
         // 4. 灏勫嚮璁稿彲鐘舵€佹満
        if (raw_shoot == 1)
            auto_aim.shoot_signal = AUTO_AIM_SHOOT_TURN_ON;
        else
            auto_aim.shoot_signal = AUTO_AIM_SHOOT_TURN_OFF;

        auto_aim.datafloat.d_yaw   = raw_yaw   / 10.0f;
        auto_aim.datafloat.d_pitch = raw_pitch / 10.0f;

        auto_aim.datafloat.d_yaw =
            auto_aim.datafloat.d_yaw * 0.6f +
            auto_aim.datafloat.last_d_yaw * 0.4f;

        auto_aim.datafloat.d_pitch =
            auto_aim.datafloat.d_pitch * 0.6f +
            auto_aim.datafloat.last_d_pitch * 0.4f;
        // 瑙掑害姝诲尯锛堥槻鎶栦簯鍙帮級
        if (fabsf(auto_aim.datafloat.d_yaw) < 0.3f)
            auto_aim.datafloat.d_yaw = 0.0f;

        if (fabsf(auto_aim.datafloat.d_pitch) < 0.3f)
            auto_aim.datafloat.d_pitch = 0.0f;
    }
}

void AutoAim_SendToYaw_baseBoard(int16_t auto_aim_flag)
{
	CAN_TxHeaderTypeDef  auto_aim_tx_message;
	uint8_t autoAim_data[8] = {0};
	
	uint32_t send_mail_box;
	auto_aim_tx_message.StdId = 0x230;
	auto_aim_tx_message.IDE = CAN_ID_STD;
	auto_aim_tx_message.RTR = CAN_RTR_DATA;
	auto_aim_tx_message.DLC = 0x08;
	
	autoAim_data[0] = auto_aim_flag;
	
	
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)==0){};
  HAL_CAN_AddTxMessage(&hcan2, &auto_aim_tx_message, autoAim_data, &send_mail_box);
}
