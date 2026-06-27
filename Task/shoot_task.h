#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"


/*define thread handle*/
extern TaskHandle_t shoot_taskHandle;				// main thread
extern TaskHandle_t shootMotorReceive_taskHandle;	// the branch thread to receive chassis motor data

/*define queue handle*/

typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}ShootMotorReceiveQueueData_t;
extern QueueHandle_t shootMotorReceive_queueHandle;

typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}TriggerMotorReceiveQueueData_t;
extern QueueHandle_t triggerMotorReceive_queueHandle;



/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
void Shoot_TaskInit(void);


/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
void Shoot_Task(void *argument);                 


/**
  * @brief          chassis branch thread
  * @param[in]      none
  * @retval         none
  */
void ShootMotorReceive_Task(void *argumemt);

void TriggerMotorReceive_Task(void *argumemt);
#endif

