/**
 * @file: Judging_System_Handler.c
 * @author: linruiyuan
 * @date: 2026-3-4
 * @version: 1.0
 * @description: RM裁判系统数据接收Handler层实现文件
 *               本文件实现了RoboMaster裁判系统数据应用层的初始化、接收与处理接口
 *               包含数据解析、帧头查找、CRC校验、数据更新等核心功能
 *               基于环形缓冲区实现高效的数据流处理
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
 * @see 相关头文件: Judging_System_Handler.h
 * @see 相关模块: RingBuffer.h (环形缓冲区)
 * @see 相关技术文档: RoboMaster裁判系统串口协议附录 V1.1.0
 * @see 如有问题请联系作者: 13825821198@163.com
 */

#include "Judging_System_Handler.h"
judg_sys_handle_t JudgSysHandle;
uint8_t Frame_Header_Sof[JUDGSYS_FRAME_HEADER_SOF_SIZE] = {JUDGSYS_FRAME_HEADER_SOF};

static void osTaskNotifyFromISR(TaskHandle_t task_handle)
{
	if(task_handle == NULL)
	return;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(task_handle, 
												0,
					  eNoAction, 
					  &xHigherPriorityTaskWoken);
	
	if (xHigherPriorityTaskWoken != pdFALSE) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

Judging_System_Status_t judgSys_Event_CallBack(judg_sys_handle_t* judgSys,uint16_t Size)
{
    if(judgSys == NULL)
    {
        return JUDGING_HANDLE_ERROR;
    }
    if(Rb_PushFromCallback(&judgSys->rb,Size) != RB_ERROR)
    {
        osTaskNotifyFromISR(judgSys->task);
        return JUDGING_SYSTEM_OK;
    }else
    {
        return JUDGING_RB_ERROR;
    }
}

static Judging_System_Status_t my_judg_sys_t_init(my_judg_sys_t* sys)
{
    if(sys == NULL)
    {
        return JUDGING_HANDLE_ERROR;
    }

    if(game_status_init(&sys->game_status) &&
       game_result_init(&sys->result) &&
       game_robot_HP_init(&sys->robot_HP) &&
       referee_warning_init(&sys->warning) &&
       robo_status_init(&sys->robot_status) &&
       power_heat_data_init(&sys->heat) &&
       robot_pos_init(&sys->pos) &&
       buff_init(&sys->buff) &&
       shoot_data_init(&sys->shoot)&&
       projectile_allowance_init(&sys->allowance))
    {
        return JUDGING_SYSTEM_OK;
    }
    else
    {
        return JUDGING_HANDLE_ERROR;
    }
}

Judging_System_Status_t judg_sys_handle_t_init(judg_sys_handle_t *handle, UART_HandleTypeDef *huart,osThreadId_t task,
                                               uint8_t *judg_buffer,uint16_t buffer_size)
{
    if(handle == NULL || huart == NULL || task == NULL || RingBuffer_Init(&handle->rb, judg_buffer, buffer_size) != RB_OK)
    {
        return JUDGING_HANDLE_ERROR;
    }
    handle->huart = huart;
    handle->task = task;
    if(my_judg_sys_t_init(&handle->sys) != JUDGING_SYSTEM_OK)
    {
        return JUDGING_HANDLE_ERROR;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart,judg_buffer,buffer_size);
    return JUDGING_SYSTEM_OK;
}

static Judging_System_Status_t judging_system_update(my_judg_sys_t* sys, uint16_t cmd_id, uint8_t* data)
{
    if(sys == NULL || data == NULL)
    {
        return JUDGING_HANDLE_ERROR;
    }

    switch(cmd_id)
    {
        case GAME_STATUS_ID:
            if(get_game_status(&sys->game_status, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case GAME_RESULT_ID:
            if(get_game_result(&sys->result, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case GAME_ROBOT_HP_ID:
            if(get_game_robot_HP(&sys->robot_HP, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case REFEREE_WARNING_ID:
            if(get_referee_warning(&sys->warning, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case ROBOT_STATUS_ID:
            if(get_robot_status(&sys->robot_status, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case POWER_HEAT_DATA_ID:
            if(get_power_heat_data(&sys->heat, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case ROBOT_POS_ID:
            if(get_robot_pos(&sys->pos, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case BUFF_ID:
            if(get_buff(&sys->buff, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case SHOOT_DATA_ID:
            if(get_shoot_data(&sys->shoot, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
        case PROJECTILE_ALLOWANCE_ID:
            if(get_projectile_allowance(&sys->allowance, data))
            {
                return JUDGING_SYSTEM_OK;
            }
            break;
    }
    return JUDGING_NOT_CMDID;
}

Judging_System_Status_t judging_system_data_process(judg_sys_handle_t* judgSys)
{
    if(judgSys == NULL)
    {
        return JUDGING_HANDLE_ERROR;
    }
    if(Rb_Location(&judgSys->rb,Frame_Header_Sof,JUDGSYS_FRAME_HEADER_SOF_SIZE)!=RB_OK)
    {
        return JUDGING_PACKET_NOTFINED_FRAME_HEADER;
    }

    uint8_t frame_header[JUDGSYS_FRAME_HEADER_SIZE] = {0};
    if(Rb_Peek(&judgSys->rb,frame_header,0,JUDGSYS_FRAME_HEADER_SIZE)!=RB_OK)
    {
        return JUDGING_RB_ERROR;
    }

    if(frame_header[0] == JUDGSYS_FRAME_HEADER_SOF)
    {
        if(Verify_CRC8_Check_Sum(frame_header,JUDGSYS_FRAME_HEADER_SIZE) == 0)
        {
            return JUDGING_CRC8_ERROR;
        }else
        {
            uint16_t data_length = frame_header[2] << 8 | frame_header[1];
            uint8_t packet[data_length + JUDGSYS_FRAME_HEADER_SIZE + 4];//cmd_id(2) + data_length + crc16(2)
            if(Rb_Take(&judgSys->rb,packet,data_length + 4 + 5)!=RB_OK)
            {
                return JUDGING_RB_ERROR;
            }else
            {
                if(Verify_CRC16_Check_Sum(packet,data_length + JUDGSYS_FRAME_HEADER_SIZE + 4) == 0)
                {
                    return JUDGING_CRC16_ERROR;
                }else
                {
                    uint16_t cmd_id = packet[6] << 8 | packet[5];
                    return judging_system_update(&judgSys->sys,cmd_id,&packet[7]);
                }
            }
        }
    }
    return JUDGING_SYSTEM_OK;
}


