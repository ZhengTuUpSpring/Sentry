#ifndef __RC_TASK_H
#define __RC_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "usart.h"

/*define thread handle*/
extern TaskHandle_t rc_taskHandle;				// main thread

/*define queue handle*/
extern QueueHandle_t rc_queueHandle;
 extern volatile uint32_t rc_frame_drop_cnt;

void RC_TaskInit(void);

void remoteRecevie_Task(void *argumet);

void remoteReceive_IDLE_ISR(UART_HandleTypeDef *huart);
void RC_FrameQueuePushFromISR(const uint8_t *frame);

void vCheckHeap(void);

#endif
