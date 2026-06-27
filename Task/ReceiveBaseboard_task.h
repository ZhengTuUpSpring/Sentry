#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
void ReceiveBaseboard_Task(void *argument);
void refereeSystem_Init(void);
void ChassisNavReceive_Task(void *argumemt);

//typedef struct{
//    uint16_t id;
//    uint8_t rx_data[8];
//}NavReceiveQueueData_t;
//extern QueueHandle_t NavReceive_queueHandle;

//typedef  struct 
//{
// uint8_t game_type; 
// uint8_t game_progress; 
// uint16_t stage_remain_time; 
// uint64_t SyncTimeStamp; 
//}game_status_t;


//// 游戏机器人血量数据结构
//typedef struct {
//    uint16_t red_1_robot_HP; 
//    uint16_t red_2_robot_HP; 
//    uint16_t red_3_robot_HP; 
//    uint16_t red_4_robot_HP; 
//    uint16_t reserved1; 
//    uint16_t red_7_robot_HP; 
//    uint16_t red_outpost_HP; 
//    uint16_t red_base_HP; 
//    uint16_t blue_1_robot_HP; 
//    uint16_t blue_2_robot_HP; 
//    uint16_t blue_3_robot_HP; 
//    uint16_t blue_4_robot_HP; 
//    uint16_t reserved2; 
//    uint16_t blue_7_robot_HP; 
//    uint16_t blue_outpost_HP; 
//    uint16_t blue_base_HP; 
//} game_robot_HP_t;

//// 事件数据结构
//typedef struct { 
//    uint32_t event_data; 
//} event_data_t;

//// 飞镖信息结构
//typedef struct {
//    uint8_t dart_remaining_time; 
//    uint16_t dart_info; 
//} dart_info_t;

//// 机器人状态结构
//typedef struct { 
//    uint8_t robot_id; 
//    uint8_t robot_level; 
//    uint16_t current_HP; 
//    uint16_t maximum_HP; 
//    uint16_t shooter_barrel_cooling_value; 
//    uint16_t shooter_barrel_heat_limit; 
//    uint16_t chassis_power_limit; 
//    uint8_t power_management_gimbal_output; 
//    uint8_t power_management_chassis_output; 
//    uint8_t power_management_shooter_output; 
//} robot_status_t;

//// 能量与热量数据结构
//typedef struct {
//    uint16_t reserved1;
//    uint16_t reserved2; 
//    float reserved3; 
//    uint16_t buffer_energy; 
//    uint16_t shooter_17mm_1_barrel_heat; 
//    uint16_t shooter_17mm_2_barrel_heat; 
//    uint16_t shooter_42mm_barrel_heat; 
//} power_heat_data_t;

//// 机器人位置结构
//typedef struct { 
//    float x; 
//    float y; 
//    float angle; 
//} robot_pos_t;

//// 受伤数据结构
//typedef struct { 
//    uint8_t armor_id;
//    uint8_t HP_deduction_reason;
//} hurt_data_t;

//// 弹药允许量数据结构
//typedef struct { 
//    uint16_t projectile_allowance_17mm; 
//    uint16_t projectile_allowance_42mm; 
//    uint16_t remaining_gold_coin; 
//} projectile_allowance_t;

//// RFID状态数据结构
//typedef struct { 
//    uint32_t rfid_status; 
//} rfid_status_t;

//// 飞镖客户端命令数据结构
//typedef struct { 
//    uint8_t dart_launch_opening_status; 
//    uint8_t reserved; 
//    uint16_t target_change_time; 
//    uint16_t latest_launch_cmd_time; 
//} dart_client_cmd_t;

//// 地面机器人位置数据结构
//typedef struct { 
//    float hero_x; 
//    float hero_y; 
//    float engineer_x; 
//    float engineer_y; 
//    float standard_3_x; 
//    float standard_3_y; 
//    float standard_4_x; 
//    float standard_4_y; 
//    float reserved1; 
//    float reserved2; 
//} ground_robot_position_t;

//// 雷达标记数据结构
//typedef struct { 
//    uint8_t mark_progress; 
//} radar_mark_data_t;

//// 哨兵信息数据结构
//typedef struct { 
//    uint32_t sentry_info; 
//    uint16_t sentry_info_2; 
//} sentry_info_t;

//// 雷达信息数据结构
//typedef struct { 
//    uint8_t radar_info; 
//} radar_info_t;


//// 机器人所有信息结构
//typedef struct { 
//    game_status_t game_status;
//    game_robot_HP_t Robot_Data;
//    event_data_t event_data;
//    dart_info_t dart_info;
//    robot_status_t robot_status;
//    power_heat_data_t power_heat_data;
//    robot_pos_t robot_pos;
//    hurt_data_t hurt_data;
//    projectile_allowance_t projectile_allowance;
//    rfid_status_t rfid_status;
//    dart_client_cmd_t dart_client_cmd;
//    ground_robot_position_t ground_robot_position;
//    radar_mark_data_t radar_mark_data;
//    sentry_info_t sentry_info;
//    radar_info_t radar_info;
//} RobotInfo;

//extern RobotInfo Robot_All_Info;

extern QueueHandle_t refereeSystem_queueHandle;
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}refereeSystemQueueData_t;

extern uint16_t heat, heat_limit;

void RefereeSystemRetransmission_Task(void *argument);

