#include "rc_task.h"
#include "remoteControl.h"
#include "SendBaseboard_task.h"
#include "can_receive.h"
#include "gimbalControl.h"
#include "driver_4310.h"

/*define thread handle*/
TaskHandle_t rc_taskHandle;				// main thread

/*define queue handle*/
QueueHandle_t rc_queueHandle;

int n1;
void RC_TaskInit(void)
{
	BaseType_t xRetruned;
	rc_taskHandle = NULL;
	
	xRetruned = xTaskCreate(
		remoteRecevie_Task,
		"remote_receive_task",
		125,
		NULL,
		osPriorityHigh,
		&rc_taskHandle
	);
	while(xRetruned!=pdPASS){};	
}


uint8_t sbus_rx_buffer[18];
void remoteRecevie_Task(void *argumet)
{
	rc_queueHandle = xQueueCreate(18, 18*(sizeof(uint8_t)));
	while(rc_queueHandle == NULL){};
  REMOTE_DMA_Init(&huart3, sbus_rx_buffer, &RC_Ctl);

	BaseType_t xStatus;
  uint8_t buf[18] = {0};
//  J4310_EnableData_Send();
	
	for(;;)
	{
		xStatus = xQueueReceive(rc_queueHandle, buf, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			
      RC_Process(&RC_Ctl, buf);
			RC_User_Process(&RC_Ctl, &rc_user);
//			RC_CAN_send_to_baseboard_chassis(RC_Ctl.rc.ch0, RC_Ctl.rc.ch1, RC_Ctl.rc.roller, RC_Ctl.rc.ch2, RC_Ctl.rc.s1, RC_Ctl.rc.s2);
//			J4310_EnableData_Send();
      vCheckHeap();
		}
		else{
		}
	}
}

size_t xFreeHeap;
volatile uint16_t shootQueueErrCnt = 0;


//检查堆剩余量及时调整堆大小
void vCheckHeap(void) {
    xFreeHeap = xPortGetFreeHeapSize();
}

void remoteReceive_IDLE_ISR(UART_HandleTypeDef *huart)
{
   BaseType_t xHigherPeriorityTaskwoken = pdFALSE;
	 BaseType_t xHigherPerioityTaskwoken = pdFALSE;//是否需要切换上下文
	 BaseType_t xsendStatus;
	
	 uint32_t restDataCount = 0;
	 uint32_t receivedDataCount = 0;

    if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断串口dma接收是否处于空闲状态
    {
		__HAL_UART_CLEAR_IDLEFLAG(huart);//清除DMA空闲中断标志位
			
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)){//如果发生了接收溢出错误（ORE），则清除该错误标志，防止错误状态持续影响后续接收。
			__HAL_UART_CLEAR_OREFLAG(huart);//清除溢出标志位
		}
        HAL_UART_DMAStop(huart);//接收完成，停止dma接收，开始转发数据
		
		restDataCount = __HAL_DMA_GET_COUNTER(huart->hdmarx);//获取DMA剩余的传输计数
		receivedDataCount = 18 - restDataCount;//用预设传输数据长度减去剩余的传输计数得到数据长度

		if (receivedDataCount == 18 && rc_queueHandle != NULL)
   {
    xsendStatus = xQueueSendFromISR(
        rc_queueHandle,
        sbus_rx_buffer,
        &xHigherPeriorityTaskwoken);

      if (xsendStatus != pdPASS)
      {
        shootQueueErrCnt++;   
      }
     }
     else{}
		HAL_UART_Receive_DMA(huart, sbus_rx_buffer , 18);//每次接收18 字节进入一次中断
    }
    portYIELD_FROM_ISR(xHigherPerioityTaskwoken);//切换上下文
}

