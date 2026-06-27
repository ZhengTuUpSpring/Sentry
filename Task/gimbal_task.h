#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "jy901.h"
#include "shoot_task.h"
#include "shootControl.h"

/*define thread handle*/
extern TaskHandle_t gimbal_taskHandle;				// main threade
extern TaskHandle_t gimbalMotorReceive_taskHandle;	// the branch thread to receive chassis motor data
extern TaskHandle_t gimbalAimReceive_taskHandle;	// the branch thread to receive chassis motor data
/*define imu handle*/
extern JY901_DataHandleTypedef  gimbalJY901_imuHandle;

/*define queue handle*/
extern QueueHandle_t gimbalYawMotorReceive_queueHandle;
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}GimbalMotorReceiveQueueData_t;

typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}GimbalPitchMotorReceiveQueueData_t;
extern QueueHandle_t gimbalPitchMotorReceive_queueHandle;

extern QueueHandle_t gimbalIMUReceive_queueHandle;
typedef struct{
    uint8_t rx_data[RXBUFFER_LEN];
}GimbalIMUReceiveQueueData_t;
// QueueHandle_t chassisIMUReceive_queueHandle;

extern QueueHandle_t gimbalAimReceive_queueHandle;
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}GimbalAimReceiveQueueData_t;
#define CLAMP(val, min, max) \
    ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))
/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
void Gimbal_TaskInit(void);
void yaw_con(void);
void pitch_con(void);
/**
  * @brief          chassis main thread
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Task(void *argument);

void GimbalPitchReceive_Task(void *argumemt);

void AutoAim_SendToYaw_baseBoard(int16_t auto_aim_flag);
/**
  * @brief          chassis branch thread
  * @param[in]      none
  * @retval         none
  */
void GimbalMotorReceive_Task(void *argumemt);

/**
  * @brief          chassis branch thread
  * @param[in]      none
  * @retval         none
  */
void GimbalImuReceive_Task(void *argument);

/**
 * @brief    idle receice handle
 * @note     freertos, be called in stm32h7xx_it.c file
 * @param    hipnuc_raw    hipnuc data handle
 */
void JY901Receive_IDLE_ISR(UART_HandleTypeDef *huart, JY901_DataHandleTypedef* jy901);

/**
  * @brief          chassis branch thread
  * @param[in]      none
  * @retval         none
  */
void GimbalAimReceive_Task(void *argumemt);

extern uint8_t disengage;
#endif
