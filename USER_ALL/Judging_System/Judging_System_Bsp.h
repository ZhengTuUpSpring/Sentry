/**
 * @file: Judging_System_Bsp.h
 * @author: linruiyuan
 * @date: 2026-3-3
 * @version: 1.0
 * @description: RM裁判系统数据接收BSP层头文件
 *               本文件实现了RoboMaster裁判系统BSP层数据宏定义、结构体定义以及接口函数声明
 *               完全遵循RoboMaster官方裁判系统串口协议附录 V1.1.0
 *               定义了所有裁判系统数据类型的结构体、命令ID、缓冲区大小及相关枚举类型
 * 
 * @copyright Copyright (c) 2026 linruiyuan
 * @license Limited 
 * 
 * @warning 本代码仅供珠海科技学院学生学习和研究使用
 * @warning 未经作者明确书面许可，禁止任何形式的转载、分发或商业使用
 * @warning 严禁将本代码用于任何商业比赛或盈利性项目
 * 
 * @note 使用本代码请遵守以下条款： *       1. 仅限于珠海科技学院在校学生个人学习和研究
 *       2. 不得删除或修改本版权声明
 *       3. 严禁将代码上传至公开代码仓库或论坛
 *       4. 严禁将代码用于任何形式的商业用途
 *       5. 使用本代码产生的任何问题与后果由使用者自行承担
 * 
 * @see 相关实现文件: Judging_System_Bsp.c
 * @see 相关技术文档: RoboMaster裁判系统串口协议附录 V1.1.0
 * @see 如有问题请联系作者: 13825821198@163.com
 */
 
#ifndef JUDGING_SYSTEM_BSP
#define JUDGING_SYSTEM_BSP

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#define JUDGSYS_FRAME_HEADER_SOF 0xA5
#define JUDGSYS_FRAME_HEADER_SOF_SIZE 0x01

#define JUDGSYS_FRAME_HEADER_SIZE 0x05

#define GAME_STATUS_BUFFER_SIZE 11
#define GAME_RESULT_BUFFER_SIZE 1
#define GAME_ROBOT_HP_BUFFER_SIZE 16
#define EVENT_DATA_BUFFER_SIZE 4
#define REFEREE_WARNING_BUFFER_SIZE 3
#define DART_INFO_BUFFER_SIZE 3
#define ROBOT_STATE_BUFFER_SIZE 13
#define POWER_HEAT_DATA_BUFFER_SIZE 14
#define ROBOT_POS_BUFFER_SIZE 12
#define BUFF_BUFFER_SIZE 8
#define HURT_BUFFER_SIZE 1
#define SHOOT_DATA_BUFFER_SIZE 7
#define PROJECTILE_ALLOWANCE_BUFFER_SIZE 8
#define RFID_STATUS_BUFFER_SIZE 5
#define DART_CLIENT_CMD_BUFFER_SIZE 6
#define GROUND_ROBOT_POSITION_BUFFER_SIZE 40
#define RADAR_MARK_DATA_BUFFER_SIZE 2
#define SENTRY_INFO_BUFFER_SIZE 6

#define GAME_STATUS_ID 0x0001
#define GAME_RESULT_ID 0x0002
#define GAME_ROBOT_HP_ID 0x0003
#define EVENT_DATA_ID 0x0101
#define REFEREE_WARNING_ID 0x0104
#define DART_INFO_ID 0x0105
#define ROBOT_STATUS_ID 0x0201
#define POWER_HEAT_DATA_ID 0x0202
#define ROBOT_POS_ID 0x0203
#define BUFF_ID 0x0204
#define HURT_ID 0x0206
#define SHOOT_DATA_ID 0x0207
#define PROJECTILE_ALLOWANCE_ID 0x0208
#define RFID_STATUS_ID 0x0209
#define DART_CLIENT_CMD_ID 0x020A
#define GROUND_ROBOT_POSITION_ID 0x020B
#define RADAR_MARK_DATA_ID 0x020C
#define SENTRY_INFO_ID 0x020D

typedef enum
{
    RMUC = 1,       // RoboMaster 机甲大师超级对抗赛
    RMUT = 2,       // RoboMaster 机甲大师高校单项赛
    RMUA = 3,       // ICRA RoboMaster 高校人工智能挑战赛
    RMUL_3V3 = 4,   // RoboMaster 机甲大师高校联盟赛 3V3 对抗
    RUML_1V1 = 5,   // RoboMaster 机甲大师高校联盟赛步兵对抗
}Game_Type_t;

typedef enum
{
    MATCH_STATUS_NOT_STARTED      = 0,   // 未开始比赛
    MATCH_STATUS_PREPARATION      = 1,   // 准备阶段
    MATCH_STATUS_SELF_CHECK       = 2,   // 十五秒裁判系统自检阶段
    MATCH_STATUS_COUNTDOWN        = 3,   // 五秒倒计时
    MATCH_STATUS_IN_PROGRESS      = 4,   // 比赛中
    MATCH_STATUS_CALCULATING      = 5,   // 比赛结算中
}Game_Stage_t;

typedef enum
{
    WARNING_NONE = 0,      // 无判罚
    WARNING_YELLOW_BOTH,   // 双方黄牌
    WARNING_YELLOW,        // 黄牌
    WARNING_RED,           // 红牌
    WARNING_LOSS,          // 判负
}Warning_Level_t;

/*0x0001比赛状态数据*/
typedef struct 
{
    Game_Type_t game_type; 
    Game_Stage_t game_progress; 
    uint16_t stage_remain_time; 
    uint64_t SyncTimeStamp; 
}game_status_t;

/*0x0002比赛结果数据*/
typedef enum 
{ 
    Game_Result_Draw = 0,          //平局
    Game_Result_RED = 1,           //红方胜利
    Game_Result_BLUE = 2,          //蓝方胜利
    Game_Init = 3                  //比赛结果未定，或比赛尚未开始             
}Result_t;

typedef struct 
{
    Result_t result;
}Game_Result_t;


/* 0x0003 比赛机器人血量数据 */
typedef struct 
{ 
    uint16_t ally_1_robot_HP;      // 己方 1 号英雄机器人血量（0：未上场/罚下）
    uint16_t ally_2_robot_HP;      // 己方 2 号工程机器人血量
    uint16_t ally_3_robot_HP;      // 己方 3 号步兵机器人血量
    uint16_t ally_4_robot_HP;      // 己方 4 号步兵机器人血量
//    uint16_t reserved;            =// 保留位
    uint16_t ally_7_robot_HP;      // 己方 7 号哨兵机器人血量
    uint16_t ally_outpost_HP;      // 己方前哨站血量
    uint16_t ally_base_HP;         // 己方基地血量
} game_robot_HP_t;

/* 0x0101 场地事件数据 */
/*
 * event_data 位定义说明：
 * 
 * bit 0-2：补给区占领状态
 *   bit 0：己方与资源区区不重叠的补给区占领状态，1为已占领
 *   bit 1：己方与资源区重叠的补给区占领状态，1为已占领  
 *   bit 2：己方补给区的占领状态，1为已占领（仅RMUL适用）
 * 
 * bit 3-6：能量机关状态
 *   bit 3-4：己方小能量机关激活状态：0未激活，1已激活，2正在激活
 *   bit 5-6：己方大能量机关激活状态：0未激活，1已激活，2正在激活
 * 
 * bit 7-8：中央高地占领状态：1被己方占领，2被对方占领
 * 
 * bit 9-10：梯形高地占领状态：1为已占领
 * 
 * bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，默认0）
 * 
 * bit 20-22：对方飞镖最后一次击中目标：
 *   0：默认，1：前哨站，2：基地固定目标，
 *   3：基地随机固定目标，4：基地随机移动目标，5：基地末端移动目标
 * 
 * bit 23-24：中心增益点占领状态（仅RMUL适用）：
 *   0：未被占领，1：被己方占领，2：被对方占领，3：被双方占领
 * 
 * bit 25-26：己方堡垒增益点占领状态：
 *   0：未被占领，1：被己方占领，2：被对方占领，3：被双方占领
 * 
 * bit 27-28：己方前哨站增益点占领状态：
 *   0：未被占领，1：被己方占领，2：被对方占领
 * 
 * bit 29：己方基地增益点占领状态：1为已占领
 * 
 * bit 30-31：保留位
 */
typedef struct 
{ 
    uint32_t event_data;    // 场地事件数据（按位定义，详见下方说明）
}event_data_t;


/*0x0104裁判警告数据*/
typedef struct 
{ 
 Warning_Level_t level; 
 uint8_t offending_robot_id; // 己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101）
 uint8_t count;              // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。）
}referee_warning_t;

/*0x0105飞镖发射数据*/
typedef struct 
{ 
 uint8_t dart_remaining_time; 
 uint16_t dart_info; 
}dart_info_t;

/*0x0201机器人性能体系数据*/
typedef struct 
{ 
    uint8_t robot_id;                             // 本机器人ID
    uint8_t robot_level;                          // 机器人等级
    uint16_t current_HP;                          // 机器人当前血量
    uint16_t maximum_HP;                          // 机器人血量上限
    uint16_t shooter_barrel_cooling_value;        // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;           // 机器人射击热量上限
    uint16_t chassis_power_limit;                 // 机器人底盘功率上限
    
    // 电源管理模块的输出情况
    bool power_management_gimbal_output;          // gimbal口输出：0无输出，1为24V输出
    bool power_management_chassis_output;         // chassis口输出：0无输出，1为24V输出
    bool power_management_shooter_output;         // shooter口输出：0无输出，1为24V输出
} robot_status_t;

/* 0x0202 底盘缓冲能量与射击热量数据 */
typedef struct 
{ 
    //uint16_t reserved;                     // 保留位
    //uint16_t reserved;                     // 保留位
    //float reserved;                         // 保留位
    uint16_t buffer_energy;                  // 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat;     // 第 1 个 17mm 发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;       // 42mm 发射机构的射击热量
}power_heat_data_t;

/* 0x0203 机器人位置数据 */
typedef struct 
{ 
    float x;          // 本机器人位置 x 坐标，单位：m
    float y;          // 本机器人位置 y 坐标，单位：m
    float angle;      // 本机器人测速模块的朝向，单位：度（正北为 0 度）
} robot_pos_t;

/*
 * remaining_energy 位定义说明：
 * bit 0-6：机器人剩余能量值比例反馈（仅在剩余能量小于50%时反馈，其余默认0x80）
 *   bit 0：剩余能量≥125%时为1，其余为0
 *   bit 1：剩余能量≥100%时为1，其余为0  
 *   bit 2：剩余能量≥50%时为1，其余为0
 *   bit 3：剩余能量≥30%时为1，其余为0
 *   bit 4：剩余能量≥15%时为1，其余为0
 *   bit 5：剩余能量≥5%时为1，其余为0
 *   bit 6：剩余能量≥1%时为1，其余为0
 *   bit 7：保留位（默认0x80表示剩余能量≥50%）
 * 
 * 注：机器人初始能量视为100%
 */
/* 0x0204 机器人增益数据和底盘能量数据 */
typedef struct 
{ 
    uint8_t recovery_buff;          // 机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
    uint16_t cooling_buff;          // 机器人射击热量冷却增益（直接值，值为x表示热量冷却增加x/s）
    uint8_t defence_buff;           // 机器人防御增益（百分比，值为50表示50%防御增益）
    uint8_t vulnerability_buff;     // 机器人负防御增益（百分比，值为30表示-30%防御增益）
    uint16_t attack_buff;           // 机器人攻击增益（百分比，值为50表示50%攻击增益）
    uint8_t remaining_energy;       // 机器人剩余能量值反馈（位定义详见下方说明）
} buff_t;

/*
 * armor_id 位定义说明：
 * bit 0-3：扣血原因对应的装甲模块/测速模块ID编号
 *   - 当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该值为对应模块ID
 *   - 当其他原因导致扣血时，该值为0
 * 
 * HP_deduction_reason 位定义说明：
 * bit 4-7：血量变化类型
 *   0：装甲模块被弹丸攻击导致扣血
 *   1：装甲模块或超级电容管理模块离线导致扣血
 *   5：装甲模块受到撞击导致扣血
 */
/* 0x0206 伤害状态数据 */
typedef struct 
{ 
    uint8_t armor_id;                  // 扣血原因对应的装甲模块或测速模块ID（位定义详见下方说明）
    uint8_t HP_deduction_reason;        // 血量变化类型
} hurt_data_t;

/*
 * bullet_type 位定义说明：
 *   bit 1：17mm 弹丸
 *   bit 2：42mm 弹丸
 * 
 * shooter_number 取值说明：
 *   1：17mm 发射机构
 *   2：保留位
 *   3：42mm 发射机构
 */
/* 0x0207 实时射击数据，弹丸发射后发送 */
typedef struct 
{ 
    uint8_t bullet_type;            // 弹丸类型（位定义详见下方说明）
    uint8_t shooter_number;         // 发射机构ID
    uint8_t launching_frequency;    // 弹丸射速（单位：Hz）
    float initial_speed;            // 弹丸初速度（单位：m/s）
} shoot_data_t;

/* 0x0208 允许发弹量数据 */
typedef struct 
{ 
    uint16_t projectile_allowance_17mm;     // 机器人自身拥有的 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;     // 42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;           // 剩余金币数量
    uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备 17mm 弹丸允许发弹量（该值与机器人是否实际占领堡垒无关）
} projectile_allowance_t;

/*
 * rfid_status 位定义说明（bit值为1表示已检测到该增益点RFID卡，0表示未检测到）：
 * 
 * bit 0：己方基地增益点
 * bit 1：己方中央高地增益点
 * bit 2：对方中央高地增益点
 * bit 3：己方梯形高地增益点
 * bit 4：对方梯形高地增益点
 * bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
 * bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
 * bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
 * bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
 * bit 9：己方地形跨越增益点（中央高地下方）
 * bit 10：己方地形跨越增益点（中央高地上方）
 * bit 11：对方地形跨越增益点（中央高地下方）
 * bit 12：对方地形跨越增益点（中央高地上方）
 * bit 13：己方地形跨越增益点（公路下方）
 * bit 14：己方地形跨越增益点（公路上方）
 * bit 15：对方地形跨越增益点（公路下方）
 * bit 16：对方地形跨越增益点（公路上方）
 * bit 17：己方堡垒增益点
 * bit 18：己方前哨站增益点
 * bit 19：己方与资源区不重叠的补给区/RMUL补给区
 * bit 20：己方与资源区重叠的补给区
 * bit 21：己方装配增益点
 * bit 22：对方装配增益点
 * bit 23：中心增益点（仅RMUL适用）
 * bit 24：对方堡垒增益点
 * bit 25：对方前哨站增益点
 * bit 26：己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
 * bit 27：己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
 * bit 28：己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
 * bit 29：己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
 * bit 30：对方地形跨越增益点（隧道）（靠近对方一侧公路区下方）
 * bit 31：对方地形跨越增益点（隧道）（靠近对方一侧公路区上方）
 * 
 * rfid_status_2 位定义说明：
 * bit 0：对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
 * bit 1：对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
 * 
 * 注：所有RFID卡仅在赛内生效。在赛外，即使检测到对应的RFID卡，对应值也为0。
 */
/* 0x0209 RFID状态数据 */
typedef struct 
{ 
    uint32_t rfid_status;      // RFID状态位0-31（详见下方说明）
    uint8_t rfid_status_2;     // RFID状态位32-39（详见下方说明）
} rfid_status_t;


/*
 * dart_launch_opening_status 取值说明：
 *   0：已经开启
 *   1：关闭
 *   2：正在开启或者关闭中
 */
/* 0x020A 飞镖选手端指令数据 */
typedef struct 
{ 
    uint8_t dart_launch_opening_status;    // 当前飞镖发射站的状态（详见下方说明）
//    uint8_t reserved;                       // 保留位
    uint16_t target_change_time;            // 切换击打目标时的比赛剩余时间，单位：秒（无/未切换动作时默认为0）
    uint16_t latest_launch_cmd_time;        // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒（初始值为0）
} dart_client_cmd_t;

/* 0x020B 地面机器人位置数据 */
typedef struct 
{ 
    float hero_x;                // 己方英雄机器人位置 x 轴坐标，单位：m
    float hero_y;                // 己方英雄机器人位置 y 轴坐标，单位：m
    float engineer_x;            // 己方工程机器人位置 x 轴坐标，单位：m
    float engineer_y;            // 己方工程机器人位置 y 轴坐标，单位：m
    float standard_3_x;          // 己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float standard_3_y;          // 己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float standard_4_x;          // 己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float standard_4_y;          // 己方 4 号步兵机器人位置 y 轴坐标，单位：m
    //float reserved;             // 保留位
    //float reserved;             // 保留位
} ground_robot_position_t;

/* 0x020C 雷达标记进度数据 */
/*
 * mark_progress 位定义说明：
 * 
 * bit 0：对方1号英雄机器人易伤情况（被标记进度≥100时为1，<100时为0）
 * bit 1：对方2号工程机器人易伤情况（被标记进度≥100时为1，<100时为0）
 * bit 2：对方3号步兵机器人易伤情况（被标记进度≥100时为1，<100时为0）
 * bit 3：对方4号步兵机器人易伤情况（被标记进度≥100时为1，<100时为0）
 * bit 4：对方哨兵机器人易伤情况（被标记进度≥100时为1，<100时为0）
 * 
 * bit 5：己方1号英雄机器人特殊标识情况（被标记进度≥50时为1，<50时为0）
 * bit 6：己方2号工程机器人特殊标识情况（被标记进度≥50时为1，<50时为0）
 * bit 7：己方3号步兵机器人特殊标识情况（被标记进度≥50时为1，<50时为0）
 * bit 8：己方4号步兵机器人特殊标识情况（被标记进度≥50时为1，<50时为0）
 * bit 9：己方哨兵机器人特殊标识情况（被标记进度≥50时为1，<50时为0）
 * 
 * bit 10-15：保留位
 */
typedef struct 
{ 
    uint16_t mark_progress;    // 雷达标记进度状态
} radar_mark_data_t;


/* 0x020D 哨兵自主决策信息同步数据 */
/*
 * sentry_info 位定义说明：
 * 
 * bit 0-10：哨兵机器人成功兑换的允许发弹量（除远程兑换外）
 *           （开局为0，成功兑换后累加）
 * 
 * bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数
 *            （开局为0，成功远程兑换后累加）
 * 
 * bit 15-18：哨兵机器人成功远程兑换血量的次数
 *            （开局为0，成功远程兑换后累加）
 * 
 * bit 19：哨兵机器人当前是否可以确认免费复活
 *         （1：可以确认免费复活，0：不可以）
 * 
 * bit 20：哨兵机器人当前是否可以兑换立即复活
 *         （1：可以兑换立即复活，0：不可以）
 * 
 * bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数
 * 
 * bit 31：保留位
 * 
 * sentry_info_2 位定义说明：
 * 
 * bit 12-13：哨兵当前姿态
 *            1：进攻姿态
 *            2：防御姿态
 *            3：移动姿态
 * 
 * bit 14：己方能量机关是否能够进入正在激活状态
 *         （1：当前可激活，0：不可激活）
 * 
 * bit 15：保留位
 */
typedef struct 
{ 
    uint32_t sentry_info;      // 哨兵信息位0-31（详见上方说明）
    uint16_t sentry_info_2;    // 哨兵信息位32-47（详见上方说明）
} sentry_info_t;

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

bool robo_status_init(robot_status_t* state);
bool game_status_init(game_status_t* status);
bool game_result_init(Game_Result_t* result);
bool game_robot_HP_init(game_robot_HP_t* hp);
bool event_data_init(event_data_t* event);
bool referee_warning_init(referee_warning_t* warning);
bool dart_info_init(dart_info_t* dart);
bool power_heat_data_init(power_heat_data_t* data);
bool robot_pos_init(robot_pos_t* pos);
bool buff_init(buff_t* buff);
bool hurt_data_init(hurt_data_t* hurt);
bool shoot_data_init(shoot_data_t* shoot);
bool projectile_allowance_init(projectile_allowance_t* allowance);
bool rfid_status_init(rfid_status_t* rfid);
bool dart_client_cmd_init(dart_client_cmd_t* cmd);
bool ground_robot_position_init(ground_robot_position_t* pos);
bool radar_mark_data_init(radar_mark_data_t* mark);
bool sentry_info_init(sentry_info_t* sentry);

bool get_game_status(game_status_t* status,uint8_t *data);
bool get_game_result(Game_Result_t* result,uint8_t *data);
bool get_game_robot_HP(game_robot_HP_t* hp,uint8_t *data);
bool get_event_data(event_data_t* event,uint8_t *data);
bool get_referee_warning(referee_warning_t* warning,uint8_t *data);
bool get_dart_info(dart_info_t* dart,uint8_t *data);
bool get_robot_status(robot_status_t* state,uint8_t *data);
bool get_power_heat_data(power_heat_data_t* heat,uint8_t *data);
bool get_robot_pos(robot_pos_t* pos,uint8_t *data);
bool get_buff(buff_t* buff,uint8_t *data);
bool get_hurt_data(hurt_data_t* hurt,uint8_t *data);
bool get_shoot_data(shoot_data_t* shoot,uint8_t *data);
bool get_projectile_allowance(projectile_allowance_t* allowance,uint8_t *data);
bool get_rfid_status(rfid_status_t* rfid,uint8_t *data);
bool get_dart_client_cmd(dart_client_cmd_t* cmd,uint8_t *data);
bool get_ground_robot_position(ground_robot_position_t* pos,uint8_t *data);
bool get_radar_mark_data(radar_mark_data_t* mark,uint8_t *data);
bool get_sentry_info(sentry_info_t* sentry,uint8_t *data);

#endif
