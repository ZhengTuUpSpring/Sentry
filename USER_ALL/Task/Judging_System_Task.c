#include "main.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"
#include "Judging_System_Handler.h"
#include "Judging_System_Task.h"
#include "can.h"
#include "bsp_dji.h"
/* DMA 接收缓冲区(同时作为 RingBuffer 底层存储, 大小 2 的幂) */
static uint8_t judg_sys_rx_buffer[JUDGSYS_BUFFER_SIZE];

/* 裁判系统接收任务句柄 */
static TaskHandle_t judg_sys_taskHandle = NULL;
TaskHandle_t HeatDataSend_taskHandle;

/* 成功解析帧计数(调试用, 上升说明数据链路通) */
volatile uint32_t g_judgsys_frame_count = 0;

/* USART6 空闲中断回调: 把 DMA 本轮收到的一段数据推进环形缓冲区并唤醒任务 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        judgSys_Event_CallBack(&JudgSysHandle, Size);
    }
}
/* 创建裁判系统接收任务 */
void JudgSys_TaskInit(void)
{
	BaseType_t xRetruned;
	HeatDataSend_taskHandle = NULL;
	
    BaseType_t ret = xTaskCreate(
        JudgSys_Task,
        "JudgSysTask",
        256,
        NULL,
        osPriorityNormal,
        &judg_sys_taskHandle
    );
    while (ret != pdPASS) {}
				
	xRetruned = xTaskCreate(
		CAN_send_shoot_Heatdata_task,
		"HeatDataSend_task",
		300,
		NULL,
		osPriorityAboveNormal7,
		&HeatDataSend_taskHandle
	);
	while(xRetruned!=pdPASS){};	
}

/* 裁判系统接收任务: 初始化 DMA 空闲接收后, 循环等待通知并解析 */
void JudgSys_Task(void *argument)
{
    (void)argument;

    /* 初始化: RingBuffer 绑定缓冲区 + 启动 USART6 空闲中断 DMA 接收 */
    judg_sys_handle_t_init(&JudgSysHandle,
                           &huart6,
                           xTaskGetCurrentTaskHandle(),
                           judg_sys_rx_buffer,
                           JUDGSYS_BUFFER_SIZE);
	
	 uint32_t ulBitsToClearOnEntry = 0x00000000;
	 uint32_t ulBitsToClearOnExit = 0xFFFFFFFF;

    for (;;)
    {
        /* 阻塞等待空闲中断发来的任务通知 */
        xTaskNotifyWait(ulBitsToClearOnEntry,ulBitsToClearOnExit,0,portMAX_DELAY);

        /* 把缓冲区里已就绪的完整帧尽量解析掉, 直到没有完整帧/数据不足 */
        Judging_System_Status_t st;
        do
        {
            st = judging_system_data_process(&JudgSysHandle);
            if (st == JUDGING_SYSTEM_OK)
            {
                g_judgsys_frame_count++;
            }
        } while (st == JUDGING_SYSTEM_OK);

    }
}
/*给上板的裁判数据*/ 
void CAN_send_shoot_Heatdata_task(void *argument)
{
	for(;;)
	{
		/* 比赛进程，机器人当前血量，机器人底盘电压，机器人发射热量，机器人发射热量限制 */
    RS_SendToHighCore(JudgSysHandle.sys.game_status.game_progress,
		JudgSysHandle.sys.robot_status.current_HP, JudgSysHandle.sys.robot_status.power_management_chassis_output, JudgSysHandle.sys.heat.shooter_17mm_1_barrel_heat, 260);
		vTaskDelay(10);
	}

}

void RS_SendToHighCore(uint8_t progress, uint16_t current_hp, uint8_t robot_status, uint16_t shoot_heat, uint16_t shoot_heat_limit)
{
    CAN_TxHeaderTypeDef  HeatData_tx_message;
    uint32_t send_mail_box;
    HeatData_tx_message.StdId = SEND_HEAT_DATA_ID;
    HeatData_tx_message.IDE = CAN_ID_STD;
    HeatData_tx_message.RTR = CAN_RTR_DATA;
    HeatData_tx_message.DLC = 0x08;
	  
	  uint8_t HeatData_can_send_data[8] = {0};
    HeatData_can_send_data[0] = progress;								//比赛进程
    HeatData_can_send_data[1] = current_hp >> 8;				//机器人血量(高8位)
    HeatData_can_send_data[2] = current_hp;							//机器人血量
    HeatData_can_send_data[3] = robot_status;						//机器人底盘供电状态
    HeatData_can_send_data[4] = shoot_heat >> 8;				//机器人发射热量(高8位)
    HeatData_can_send_data[5] = shoot_heat;							//机器人发射热量
    HeatData_can_send_data[6] = shoot_heat_limit >> 8;  //机器人发射热量上限
    HeatData_can_send_data[7] = shoot_heat_limit;				//机器人发射热量上限(高8位)

    HAL_CAN_AddTxMessage(&hcan2, &HeatData_tx_message, HeatData_can_send_data, &send_mail_box);	
}
