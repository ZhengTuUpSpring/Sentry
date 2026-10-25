#ifndef __NAV_TASK_H
#define __NAV_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "bsp_dji.h"
#include "semphr.h"

#define NAV_CAN_REV_ID 0x110  // 接收给小电脑的 CAN ID
extern TaskHandle_t Nav2chassisReceive_taskHandle;
extern QueueHandle_t Nav2chassisReceive_queueHandle;

//导航数据获取
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}ChassisNavReceiveQueueData_t;
extern ChassisNavReceiveQueueData_t chassisNav_queueData;

extern QueueHandle_t Nav2chassisReceive_queueHandle;

typedef struct{
	struct{
		double linear_x;
		double linear_y;
		double rotate_z;
	}datafloat_t;
	
	struct{
		int16_t linear_x;
		int16_t linear_y;
		int16_t rotate_z;
		uint8_t chassis_mode;
	}dataint16_t;

}Nav_DataHandle_t;

extern Nav_DataHandle_t auto_nav;
extern void NAV_Receive_Task(void *argumemt);
extern void Navigation_TaskInit(void);
#endif

