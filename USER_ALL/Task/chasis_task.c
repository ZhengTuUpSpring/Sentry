#include "chasis_task.h"
#include "arm_math.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "RC_control.h"
#include "chassis_control.h"
#include "4310_control.h"
#include "apply_4310.h"
#include "receive_task.h"
#include "string.h"
#include "math.h"
#include "limit_power_control.h"
#include "nav_task.h"
#include "chassis_angle_calculation.h"
/* define thread handle */
TaskHandle_t chassis_taskHandle;				// main thread

volatile int16_t x_speed=0,y_speed=0,z_speed=0,chassis_Yaw_acc=0;
volatile float chassis_Yaw=0.0f;
float chassis_yaw_ref_pos=0;
int16_t angle_rotation_get;
uint8_t pos_get_flag = 1;
void Chassis_TaskInit(void)
{
	BaseType_t xRetruned;
	chassis_taskHandle = NULL;
	
	xRetruned = xTaskCreate(
		Chassis_Task,
		"chassis_task",
		300,
		NULL,
		osPriorityRealtime7,
		&chassis_taskHandle
	);
	while(xRetruned!=pdPASS){}	;	
}

void Chassis_Task(void *argument)
{
    Chassis_Init();
		Chassis_Power_Modul_Init(&Chassis_Power_Modul); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    int16_t x_speed_old = 0, y_speed_old = 0, z_speed_old = 0;
    int16_t x_speed_ref = 0, y_speed_ref = 0, z_speed_ref = 0;
    uint8_t  s2_prev = 0;       /* 上次 s2 挡位 */
    uint8_t  s1_prev = 0xFF;    /* 上次 s1 挡位（0xFF 保证上电首轮触发一次复位） */
    uint16_t motor_init_tick = 0;

		uint8_t tx_data =0x01;
		float x_nav_speed;
	  float y_nav_speed;
		
    for(;;)
    {
			motor1_init(&DM_4310);
//        /* 每 50ms 调一次电机初始化 */
//        if (++motor_init_tick >= 50) {
//            motor_init_tick = 0;
//            
//					CAN_cmd_supcap_init(&tx_data);
//        }

//        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        x_speed_old = x_speed_ref;
        y_speed_old = y_speed_ref;
        z_speed_old = z_speed_ref;

        x_speed_ref = -apply_deadzone(rc.ch1);
        y_speed_ref = -apply_deadzone(rc.ch0);

        chassis_Yaw = apply_deadzone(rc.ch2) * 0.01f;
        z_speed_ref = apply_deadzone(rc.roller)*0.8f;  

        x_speed = x_speed_old * 0.7 + x_speed_ref * 0.3;
        y_speed = y_speed_old * 0.7 + y_speed_ref * 0.3;
        z_speed = z_speed_old * 0.7 + z_speed_ref * 0.3;
				

        uint8_t cur_s1 = (uint8_t)rc.s1;
        uint8_t cur_s2 = (uint8_t)rc.s2;
				
				
				//只要 rc.s1/rc.s2 和上一轮不一样，就调用一次 chassis_yaw_mode_reset()。
        if (s1_prev != cur_s1 || s2_prev != cur_s2) {
            s1_prev = cur_s1;
            s2_prev = cur_s2;
            chassis_yaw_mode_reset();   /* 发停止帧 + 清 PID + 清锁存 */
        }
						
        if (rc.s1 == 3) {
					
            if(rc.s2==2){
							if(auto_nav.dataint16_t.chassis_mode==2) {
							 x_nav_speed=auto_nav.dataint16_t.linear_x/E_P;
							 y_nav_speed=auto_nav.dataint16_t.linear_y/E_P;
								
							}
							else {
								x_nav_speed=0;
								y_nav_speed=0;
							}
							chassis_spin_counter(-x_nav_speed,-y_nav_speed,2000);
							/*最终使用*/
//							chassis_nav_auto(-x_nav_speed,-y_nav_speed,2000,2);
					}
						else if(rc.s2==1){
							
					     yaw_rc_hold(rc.ch2, 1, YAW_HOLD_MODE_VEL);
							Chassis_WheatWheel_Solution(x_speed * 9, y_speed * 9, z_speed * 4);
							
						}
        }
				else if(rc.s1==1){
					if(rc.s2==1){
					 yaw_rc_hold(rc.ch2, 1, YAW_HOLD_MODE_VEL);
					Chassis_WheatWheel_Solution(x_speed * 9, y_speed * 9, z_speed * 4);
					}
          else if(rc.s2==2){

					chassis_spin_counter(x_speed,y_speed,2000);
//						chassis_nav_auto(x_speed,y_speed,2000,base_con.get);
					}					
				}

			 if(rc.s2==3){
				 mit_ctrl(&hcan1,&DM_4310,DM_4310.id,0.0f,0.0f,0.0f,0.0f,0.0f);
				Chassis_WheatWheel_Solution(0,0,0);
				 
			 }
        vTaskDelay(1);
    }
}
