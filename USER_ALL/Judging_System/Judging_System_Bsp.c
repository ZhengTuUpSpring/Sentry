/**
 * @file: Judging_System_Bsp.c
 * @author: linruiyuan
 * @date: 2026-3-3
 * @version: 1.0
 * @description: RM裁判系统数据接收BSP层实现文件
 *               本文件实现了RoboMaster裁判系统BSP层数据初始化、接收与处理接口
 *               包含CRC8/CRC16校验算法、数据结构初始化及各类型数据的解析函数
 *               完全遵循RoboMaster官方裁判系统串口协议实现
 * 
 * @copyright Copyright (c) 2026 linruiyuan
 * @license Limited 
 * 
 * @warning 本代码仅供珠海科技学院学生学习和研究使用
 * @warning 未经作者明确书面许可，禁止任何形式的转载、分发或商业使用
 * @warning 严禁将本代码用于任何商业比赛或盈利性项目
 * 
 * @note 使用本代码请遵守以下条款：
 *       1. 仅限于珠海科技学院在校学生个人学习和研究
 *       2. 不得删除或修改本版权声明
 *       3. 严禁将代码上传至公开代码仓库或论坛
 *       4. 严禁将代码用于任何形式的商业用途
 *       5. 使用本代码产生的任何问题与后果由使用者自行承担
 * 
 * @see 相关头文件: Judging_System_Bsp.h
 * @see 相关技术文档: RoboMaster裁判系统串口协议附录 V1.1.0
 * @see 如有问题请联系作者: 13825821198@163.com
 */

#include "Judging_System_Bsp.h"

//crc8 generator polynomial:G(x)=x8+x5+x4+1 
static const unsigned char CRC8_INIT = 0xff; 
static const unsigned char CRC8_TAB[256] = 
{ 
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 
    0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 
    0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 
    0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 
    0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 
    0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 
    0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 
    0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 
    0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 
    0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c,
    0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 
    0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 
    0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 
    0x35, 
}; 

static unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
    unsigned char ucIndex; 
    while (dwLength--) 
    { 
        ucIndex = ucCRC8^(*pchMessage++); 
        ucCRC8 = CRC8_TAB[ucIndex];
    } 
    return(ucCRC8); 
} 

/* 
** Descriptions: CRC8 Verify function 
** Input: Data to Verify,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) 
{ 
    unsigned char ucExpected = 0; 
    if ((pchMessage == 0) || (dwLength <= 2)) return 0; 
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT); 
    return ( ucExpected == pchMessage[dwLength-1] );
}

/* 
** Descriptions: append CRC8 to the end of data 
** Input: Data to CRC and append,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) 
{ 
    unsigned char ucCRC = 0; 
    if ((pchMessage == 0) || (dwLength <= 2)) return; 
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT); 
    pchMessage[dwLength-1] = ucCRC; 
} 

static uint16_t CRC_INIT = 0xffff; 
static const uint16_t wCRC_Table[256] = 
{ 
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78 
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
static uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
    uint8_t chData; 
    if (pchMessage == NULL) 
    { 
      return 0xFFFF; 
    } 
    while(dwLength--) 
    { 
        chData = *pchMessage++; 
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff]; 
    } 
    return wCRC; 
} 

/* 
** Descriptions: CRC16 Verify function 
** Input: Data to Verify,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) 
{ 
    uint16_t wExpected = 0; 
    if ((pchMessage == NULL) || (dwLength <= 2)) 
    { 
     return 0; 
    } 
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT); 
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == 
    pchMessage[dwLength - 1]); 
}
 
/* 
** Descriptions: append CRC16 to the end of data 
** Input: Data to CRC and append,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) 
{ 
    uint16_t wCRC = 0; 
    if ((pchMessage == NULL) || (dwLength <= 2)) 
    { 
      return; 
    } 
    wCRC = Get_CRC16_Check_Sum ((uint8_t *)pchMessage, dwLength-2, CRC_INIT ); 
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff); 
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

bool game_status_init(game_status_t* status)
{
    if(status == NULL)
    {
        return false;
    }
    status->game_type = 0;
    status->game_progress = MATCH_STATUS_NOT_STARTED;
    status->stage_remain_time = 0;
    status->SyncTimeStamp = 0;
    return true;
}

bool game_result_init(Game_Result_t *result)
{
    if(result == NULL)
    {
        return false;
    }
    result->result = Game_Init;
    return true;
}

bool game_robot_HP_init(game_robot_HP_t *HP)
{
    if(HP == NULL)
    {
        return false;
    }
    HP->ally_1_robot_HP = 0;
    HP->ally_2_robot_HP = 0;
    HP->ally_3_robot_HP = 0;
    HP->ally_4_robot_HP = 0;
    HP->ally_7_robot_HP = 0;
    HP->ally_outpost_HP = 0;
    HP->ally_base_HP = 0;
    return true;
}

bool event_data_init(event_data_t *event)
{
    if(event == NULL)
    {
        return false;
    }
    event->event_data = 0;
    return true;
}

bool referee_warning_init(referee_warning_t *warning)
{
    if(warning == NULL)
    {
        return false;
    }
    warning->level = WARNING_NONE;
    warning->offending_robot_id = 0;
    warning->count = 0;
    return true;
}

bool dart_info_t_init(dart_info_t *dart)
{
    if(dart == NULL)
    {
        return false;
    }
    dart->dart_remaining_time = 0;
    dart->dart_info = 0;
    return true;
}

bool robo_status_init(robot_status_t* state)
{
    if(state == NULL)
    {
        return false;
    }
    state->robot_id = 0;
    state->robot_level = 0;
    state->current_HP = 0;
    state->maximum_HP = 0;
    state->shooter_barrel_cooling_value = 0;
    state->shooter_barrel_heat_limit = 0;
    state->chassis_power_limit = 0;
    state->power_management_gimbal_output = 0;
    state->power_management_chassis_output = 0;
    state->power_management_shooter_output = 0;
    return true;
}

bool power_heat_data_init(power_heat_data_t* heat)
{
    if(heat == NULL)
    {
        return false;
    }
    heat->buffer_energy = 0;
    heat->shooter_17mm_1_barrel_heat = 0;
    heat->shooter_42mm_barrel_heat = 0;
    return true;
}

bool robot_pos_init(robot_pos_t* pos)
{
    if(pos == NULL)
    {
        return false;
    }
    pos->x = 0.0f;
    pos->y = 0.0f;
    pos->angle = 0.0f;
    return true;
}

bool buff_init(buff_t* buff)
{
    if(buff == NULL)
    {
        return false;
    }
    buff->recovery_buff = 0;
    buff->cooling_buff = 0;
    buff->defence_buff = 0;
    buff->vulnerability_buff = 0;
    buff->attack_buff = 0;
    buff->remaining_energy = 0;
    return true;
}

bool hurt_data_t_init(hurt_data_t* hurt)
{
    if(hurt == NULL)
    {
        return false;
    }
    hurt->armor_id = 0;
    hurt->HP_deduction_reason = 0;
    return true;
}

bool shoot_data_init(shoot_data_t* shoot)
{
    if(shoot == NULL)
    {
        return false;
    }
    shoot->bullet_type = 0;
    shoot->shooter_number = 0;
    shoot->launching_frequency = 0;
    shoot->initial_speed = 0.0f;
    return true;
}

bool projectile_allowance_init(projectile_allowance_t* projectile)
{
    if(projectile == NULL)
    {
        return false;
    }
    projectile->projectile_allowance_17mm = 0;
    projectile->projectile_allowance_42mm = 0;
    projectile->remaining_gold_coin = 0;
    projectile->projectile_allowance_fortress = 0;
    return true;
}

bool rfid_status_t_init(rfid_status_t* rfid)
{
    if(rfid == NULL)
    {
        return false;
    }
    rfid->rfid_status = 0;
    rfid->rfid_status_2 = 0;
    return true;
}

bool dart_client_cmd_t_init(dart_client_cmd_t* cmd)
{
    if(cmd == NULL)
    {
        return false;
    }
    cmd->dart_launch_opening_status = 0;
    cmd->latest_launch_cmd_time = 0;
    cmd->target_change_time = 0;
    return true;
}

bool ground_robot_position_t_init(ground_robot_position_t* pos)
{
    if(pos == NULL)
    {
        return false;
    }
    pos->engineer_x = 0.0f;
    pos->engineer_y = 0.0f;
    pos->hero_x = 0.0f;
    pos->hero_y = 0.0f;
    pos->standard_3_x = 0.0f;
    pos->standard_3_y = 0.0f;
    pos->standard_4_x = 0.0f;
    pos->standard_4_y = 0.0f;
    return true;
}

bool radar_mark_data_t_init(radar_mark_data_t* radar)
{
    if(radar == NULL)
    {
        return false;
    }
    radar->mark_progress = 0;
    return true;
}

bool sentry_info_t_init(sentry_info_t* sentry)
{
    if(sentry == NULL)
    {
        return false;
    }
    sentry->sentry_info = 0;
    sentry->sentry_info_2 = 0;
    return true;
}

bool get_game_status(game_status_t* status,uint8_t *data)
{
    if(status == NULL)
    {
        return false;
    }
    status->game_type = data[0] & 0x0F;
    status->game_progress = (data[0] >> 4) & 0x0F;
    status->stage_remain_time = data[2] << 8 | data[1];
    status->SyncTimeStamp = (uint64_t)data[10] << 56| (uint64_t)data[9] << 48 | 
                            (uint64_t)data[8] << 40 | (uint64_t)data[7] << 32 | 
                            (uint64_t)data[6] << 24 | (uint64_t)data[5] << 16 | 
                            (uint64_t)data[4] << 8  | (uint64_t)data[3];
    return true;
}

bool get_game_result(Game_Result_t* result, uint8_t *data)
{
    if(result == NULL)
    {
        return false;
    }
    result->result = data[0];
    return true;
}

bool get_game_robot_HP(game_robot_HP_t* HP, uint8_t *data)
{
    if(HP == NULL)
    {
        return false;
    }
    HP->ally_1_robot_HP = data[1] << 8 | data[0];
    HP->ally_2_robot_HP = data[3] << 8 | data[2];
    HP->ally_3_robot_HP = data[5] << 8 | data[4];
    HP->ally_4_robot_HP = data[7] << 8 | data[6];
    HP->ally_7_robot_HP = data[11] << 8 | data[10];
    HP->ally_outpost_HP = data[13] << 8 | data[12];
    HP->ally_base_HP    = data[15] << 8 | data[14];
    return true;
}

bool get_event_data(event_data_t* event, uint8_t *data)
{
    if(event == NULL)
    {
        return false;
    }
    event->event_data = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    return true;
}

bool get_referee_warning(referee_warning_t* warning, uint8_t *data)
{
    if(warning == NULL)
    {
        return false;
    }
    warning->level = data[0];
    warning->offending_robot_id = data[1];
    warning->count = data[2];
    return true;
}

bool get_dart_info(dart_info_t* dart, uint8_t *data)
{
    if(dart == NULL)
    {
        return false;
    }
    dart->dart_remaining_time = data[0];
    dart->dart_info = data[2] << 8 | data[1];
    return true;
}

bool get_robot_status(robot_status_t* state, uint8_t *data)
{
    if(state == NULL)
    {
        return false;
    }
    state->robot_id = data[0];
    state->robot_level = data[1];
    state->current_HP = data[3] << 8 | data[2];
    state->maximum_HP = data[5] << 8 | data[4];
    state->shooter_barrel_cooling_value = data[7] << 8 | data[6];
    state->shooter_barrel_heat_limit = data[9] << 8 | data[8];
    state->chassis_power_limit = data[11] << 8 | data[10];
    state->power_management_gimbal_output  = data[12] & 0x01;
    state->power_management_chassis_output = data[12] >> 1 & 0x01;
    state->power_management_shooter_output = data[12] >> 2 & 0x01;
    return true;
}

bool get_power_heat_data(power_heat_data_t* heat, uint8_t *data)
{
    if(heat == NULL)
    {
        return false;
    }
    heat->buffer_energy              = data[9] << 8 | data[8];
    heat->shooter_17mm_1_barrel_heat = data[11] << 8 | data[10];
    heat->shooter_42mm_barrel_heat   = data[13] << 8 | data[12];
    return true;
}

bool get_robot_pos(robot_pos_t* pos, uint8_t *data)
{
    if(pos == NULL)
    {
        return false;
    }
    memcpy(&pos->x, &data[0], sizeof(float));
    memcpy(&pos->y, &data[4], sizeof(float));
    memcpy(&pos->angle, &data[8], sizeof(float));
    return true;
}

bool get_buff(buff_t* buff, uint8_t *data)
{
    if(buff == NULL)
    {
        return false;
    }
    buff->recovery_buff      = data[0];
    buff->cooling_buff       = data[2] << 8 | data[1];
    buff->defence_buff       = data[3];
    buff->vulnerability_buff = data[4];
    buff->attack_buff        = data[6] << 8 | data[5];
    buff->remaining_energy   = data[7];
    return true;
}

bool get_hurt_data(hurt_data_t* hurt, uint8_t *data)
{
    if(hurt == NULL)
    {
        return false;
    }
    hurt->armor_id = data[0] & 0x0F;
    hurt->HP_deduction_reason = data[0] >> 4 & 0x0F;
    return true;
}

bool get_shoot_data(shoot_data_t* shoot, uint8_t *data)
{
    if(shoot == NULL)
    {
        return false;
    }
    shoot->bullet_type = data[0];
    shoot->shooter_number = data[1];
    shoot->launching_frequency = data[2];
    memcpy(&shoot->initial_speed, &data[3], sizeof(float));
    return true;
}

bool get_projectile_allowance(projectile_allowance_t* allowance, uint8_t *data)
{
    if(allowance == NULL)
    {
        return false;
    }
    allowance->projectile_allowance_17mm = data[1] << 8 | data[0];
    allowance->projectile_allowance_42mm = data[3] << 8 | data[2];
    allowance->remaining_gold_coin = data[5] << 8 | data[4];
    allowance->projectile_allowance_fortress = data[7] << 8 | data[6];
    return true;
}

bool get_rfid_status(rfid_status_t* rfid, uint8_t *data)
{
    if(rfid == NULL)
    {
        return false;
    }
    rfid->rfid_status = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    rfid->rfid_status_2 = data[4];
    return true;
}

bool get_dart_client_cmd(dart_client_cmd_t* cmd, uint8_t *data)
{
    if(cmd == NULL)
    {
        return false;
    }
    cmd->dart_launch_opening_status = data[0];
    cmd->target_change_time = data[3] << 8 | data[2];
    cmd->latest_launch_cmd_time = data[5] << 8 | data[4];
    return true;
}

bool get_ground_robot_position(ground_robot_position_t* pos, uint8_t *data)
{
    if(pos == NULL)
    {
        return false;
    }
    memcpy(&pos->hero_x, &data[0], sizeof(float));
    memcpy(&pos->hero_y, &data[4], sizeof(float));
    memcpy(&pos->engineer_x, &data[8], sizeof(float));
    memcpy(&pos->engineer_y, &data[12], sizeof(float));
    memcpy(&pos->standard_3_x, &data[16], sizeof(float));
    memcpy(&pos->standard_3_y, &data[20], sizeof(float));
    memcpy(&pos->standard_4_x, &data[24], sizeof(float));
    memcpy(&pos->standard_4_y, &data[28], sizeof(float));
    return true;
}

bool get_radar_mark_data(radar_mark_data_t* radar, uint8_t *data)
{
    if(radar == NULL)
    {
        return false;
    }
    radar->mark_progress = data[1] << 8 | data[0];
    return true;
}

bool get_sentry_info(sentry_info_t* sentry, uint8_t *data)
{
    if(sentry == NULL)
    {
        return false;
    }
    sentry->sentry_info = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    sentry->sentry_info_2 = data[5] << 8 | data[4];
    return true;
}