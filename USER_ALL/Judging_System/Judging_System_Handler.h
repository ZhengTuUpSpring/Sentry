/**
 * @file: Judging_System_Handler.h
 * @author: linruiyuan
 * @date: 2026-3-4
 * @version: 1.0
 * @description: RM裁判系统数据接收Handler层头文件
 *               本文件实现了RoboMaster裁判系统应用层结构体定义以及接口函数声明
 *               定义了裁判系统数据处理所需的数据类型、状态码和外部接口
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
 *       2. 不得删除或修改本版权声明 *       3. 严禁将代码上传至公开代码仓库或论坛
 *       4. 严禁将代码用于任何形式的商业用途
 *       5. 使用本代码产生的任何问题与后果由使用者自行承担
 * 
 * @see 相关实现文件: Judging_System_Handler.c
 * @see 相关模块: RingBuffer.h (环形缓冲区)、Judging_System_Bsp.h (数据解析)
 * @see 相关技术文档: RoboMaster裁判系统串口协议附录 V1.1.0
 * @see 如有问题请联系作者: 13825821198@163.com
 */

#ifndef JUDGING_SYSTEM_HANDLER_H
#define JUDGING_SYSTEM_HANDLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Judging_System_Bsp.h"
#include "usart.h"
#include "RingBuffer.h"

#define JUDGSYS_BUFFER_SIZE 512

typedef enum
{
    JUDGING_SYSTEM_OK = 0,
    JUDGING_HANDLE_ERROR = 1,
    JUDGING_PACKET_NOTFINED_FRAME_HEADER = 2,
    JUDGING_NOT_CMDID = 3,
    JUDGING_RB_ERROR = 4,
    JUDGING_CRC8_ERROR = 5,
    JUDGING_CRC16_ERROR = 6,
}Judging_System_Status_t;

typedef struct 
{
    game_status_t game_status;         //0x0001
    Game_Result_t result;              //0x0002
    game_robot_HP_t robot_HP;          //0x0003
    referee_warning_t warning;         //0x0104
    robot_status_t robot_status;       //0x0201
    power_heat_data_t heat;            //0x0202
    robot_pos_t pos;                   //0x0203
    buff_t buff;                       //0x0204
    shoot_data_t shoot;                //0x0207
    projectile_allowance_t allowance;  //0x0208
}my_judg_sys_t;

typedef struct 
{
    UART_HandleTypeDef *huart;  
    osThreadId_t task;   		
    RingBuffer_t rb;            
    my_judg_sys_t sys;          
}judg_sys_handle_t;

Judging_System_Status_t judg_sys_handle_t_init(judg_sys_handle_t *handle, UART_HandleTypeDef *huart,osThreadId_t task,
                                               uint8_t *judg_buffer,uint16_t buffer_size);
Judging_System_Status_t judging_system_data_process(judg_sys_handle_t* judgSys);
Judging_System_Status_t judgSys_Event_CallBack(judg_sys_handle_t* judgSys,uint16_t Size);

extern judg_sys_handle_t JudgSysHandle;

#endif
