#include "shoot_task.h"
#include "shootControl.h"
#include "remoteControl.h"
#include <stdlib.h>
#include "autoControl.h"


/*define thread handle*/
TaskHandle_t shoot_taskHandle;				// main thread
TaskHandle_t shootMotorReceive_taskHandle;	// the branch thread to receive shooot motor data
TaskHandle_t triggerMotorReceive_taskHandle;  

/*define queue handle*/
QueueHandle_t shootMotorReceive_queueHandle;
QueueHandle_t triggerMotorReceive_queueHandle;

/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
void Shoot_TaskInit(void)
{
	BaseType_t xRetruned;
	shoot_taskHandle = NULL;
	shootMotorReceive_taskHandle = NULL;
	triggerMotorReceive_taskHandle = NULL;
	
	xRetruned = xTaskCreate(
		Shoot_Task,
		"shoot_task",
		300,
		NULL,
		osPriorityHigh,
		&shoot_taskHandle
	);
	configASSERT(xRetruned == pdPASS);

	xRetruned = xTaskCreate(
		ShootMotorReceive_Task,
		"shoot_motor_receive_task",
		125,
		NULL,
		osPriorityRealtime,
		&shootMotorReceive_taskHandle
	);
	configASSERT(xRetruned == pdPASS);
		
	xRetruned = xTaskCreate(
		TriggerMotorReceive_Task,
		"trigger_motor_receive_task",
		110,
		NULL,
		osPriorityRealtime,
		&triggerMotorReceive_taskHandle
	);
	configASSERT(xRetruned == pdPASS);
}

int16_t test_setWheelSpeed = 0;
uint16_t hootControl = 0x00;
uint16_t coolGun = 0x00;
/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
void Shoot_Task(void *argument)
{
    Shoot_Init();
    osDelay(10);

    for (;;)
    {
			  M3508_GetBasicData(&frictionWheel_motorHandle[0]);
        M3508_GetBasicData(&frictionWheel_motorHandle[1]);
        M2006_GetBasicData(&trigger_motor);
			ammo_dial_anti_block_core(0,0);
//        Shoot_TriggerAndFrictionWheelControl(0,10);
//         Shoot_TriggerAndFrictionWheelControl(0, 0);
        /* 摩擦轮未就绪，禁止拨弹 */
//        if (abs(frictionWheel_motorHandle[0].basic_data.speed_rpm) < 120 ||
//            abs(frictionWheel_motorHandle[1].basic_data.speed_rpm) < 120)
//        {
//            Shoot_TriggerAndFrictionWheelControl(0, 0);
//            osDelay(10);
//            continue;
//        }
//        osDelay(5);   // 固定 2ms 控制周期
    }
}


/**
  * @brief          接收发射部分电调数据
  * @param[in]      none
  * @retval         none
  */
void ShootMotorReceive_Task(void *argumemt)
{
	shootMotorReceive_queueHandle =
    xQueueCreate(5, sizeof(ShootMotorReceiveQueueData_t));
	configASSERT(shootMotorReceive_queueHandle != NULL);

	ShootMotorReceiveQueueData_t q;
	for(;;)
	{
       if (xQueueReceive(shootMotorReceive_queueHandle,
                          &q,
                          pdMS_TO_TICKS(5)) == pdPASS)
        {
            uint8_t box_index = Friction_GetBoxIndex(q.id);

            if (box_index != 0xFF)
            {
                get_motor_measure(&dji_motorbox[box_index], q.rx_data);
            }
        }
		else{
		}
	}
}
		}
/**
  * @brief          接收拨弹部分电调数据
  * @param[in]      none
  * @retval         none
  */
void TriggerMotorReceive_Task(void *argumemt)
{
	triggerMotorReceive_queueHandle = xQueueCreate(40, sizeof(TriggerMotorReceiveQueueData_t));
  configASSERT(triggerMotorReceive_queueHandle != NULL);
	BaseType_t xStatus;
	TriggerMotorReceiveQueueData_t TriggerMotor_queueData;

	for(;;)
	{
		xStatus = xQueueReceive(triggerMotorReceive_queueHandle, &TriggerMotor_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(TriggerMotor_queueData.id - 0x0200)
			{
        case 0x04:{
					get_motor_measure(&dji_motorbox[8], TriggerMotor_queueData.rx_data);
				}break;
				default:{
				}
			}
		}
		
	}
}
 }
