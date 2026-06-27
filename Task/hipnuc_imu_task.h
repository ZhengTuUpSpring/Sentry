#ifndef __HIPNUC_IMU_TASK_H 
#define __HIPNUC_IMU_TASK_H

#include "hipnuc_dec.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"

/*创建HiPNUC 数据句柄*/
extern hipnuc_raw_t chassis_hipnuc_imu;

/*创建HiPNUC 任务句柄*/
extern TaskHandle_t hipnuc_imu_taskHandle;

/*创建HIPNUC 空闲中断服务函数*/
void hipnuc_Init(void);
void vHiPNUC_Receive_IDLE_ISR(UART_HandleTypeDef* huart, hipnuc_raw_t *hipnuc_raw);
void hipnuc_imu_task(void *argument);


#endif
