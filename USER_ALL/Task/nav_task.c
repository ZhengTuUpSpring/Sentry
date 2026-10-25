#include "nav_task.h"
#include "string.h"    

Nav_DataHandle_t auto_nav;

TaskHandle_t Nav2chassisReceive_taskHandle;

QueueHandle_t Nav2chassisReceive_queueHandle;

void Navigation_TaskInit(void)
{
	BaseType_t xRetruned;
	
	xRetruned = xTaskCreate(
		NAV_Receive_Task,
		"NAV_Receive_task",
		125,
		NULL,
		osPriorityNormal,
		&Nav2chassisReceive_taskHandle
	);
	while(xRetruned!=pdPASS){};
}

void NAV_Receive_Task(void *argumemt)
{
	 	/* 创建队列接收电调的数据 */
	Nav2chassisReceive_queueHandle = xQueueCreate(40, sizeof(ChassisNavReceiveQueueData_t));
	while(Nav2chassisReceive_queueHandle == NULL){};
  /* 创建队列数据接收结构体实例和接收状态量 */
	BaseType_t xStatus;
	ChassisNavReceiveQueueData_t chassisNav_queueData;
	for(;;)
	{
		xStatus = xQueueReceive(Nav2chassisReceive_queueHandle, &chassisNav_queueData, pdMS_TO_TICKS(100));
		if(xStatus != pdFALSE)
		{
			switch(chassisNav_queueData.id)
			{
				case NAV_CAN_REV_ID:
				{
					/* 开始处理速度数据 */
					auto_nav.dataint16_t.linear_x = chassisNav_queueData.rx_data[0] | (chassisNav_queueData.rx_data[1] << 8);
					auto_nav.dataint16_t.linear_y = chassisNav_queueData.rx_data[2] | (chassisNav_queueData.rx_data[3] << 8);
					auto_nav.dataint16_t.rotate_z = chassisNav_queueData.rx_data[4] | (chassisNav_queueData.rx_data[5] << 8);
					auto_nav.dataint16_t.chassis_mode = chassisNav_queueData.rx_data[6] | (chassisNav_queueData.rx_data[7] << 8);//导航运动模式
			  }
	   }
    }
		else
		{
			memset(&auto_nav, 0, sizeof(Nav_DataHandle_t));
		}
  }
}
