#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"

#define CAN_SEND_TO_BASEBOARD_NAV_ID 0x200
#define CAN_SEND_TO_BASEBOARD_RC_ID 0x300
#define CAN_RECEIVE_FROM_NAV_CHASSIS_ID 0x520
#define CAN_SEND_TO_NUC_RC_ID 0x401


void Send_TaskInit(void);
void NAV_CAN_send_to_baseboard_chassis(int16_t pitch, int16_t yaw, int16_t roll);
void RC_CAN_send_to_baseboard_chassis(int16_t RC_ch0, int16_t RC_ch1, int16_t RC_roll, int16_t RC_ch2, int16_t RC_s1, int16_t RC_s2);
void NavReceive_Task(void *argumemt);
void VelTransToChassis_Task(void *argumemt);
void RS_Task(void *argumemt);
void RS_SendToNuc(uint8_t progress, uint16_t current_hp, uint8_t robot_status, uint8_t disengage_status);

typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}NavReceiveQueueData_t;
extern QueueHandle_t NavReceive_queueHandle;
extern uint8_t Control_mode;
extern uint8_t s2;
void UpdateS2ByKeys(uint8_t keyA, uint8_t keyB, uint8_t keyC, uint8_t *s2);
