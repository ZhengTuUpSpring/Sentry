#ifndef VTM03_HANDLER_H
#define VTM03_HANDLER_H

/*
 * @file: VTM03_Handler.h
 * @author: linruiyuan
 * @date: 2026-3-10
 * @version: 2.0
 * @description: VTM03遥控器数据接收Handler层头文件
 *               本文件实现了VTM03设备（遥控器/图传一体模块）应用层结构体定义以及接口函数声明
 *               包含环形缓冲区大小检查、状态枚举、数据结构体定义及外部接口声明
 * 
 * @copyright Copyright (c) 2026 linruiyuan
 * @license Limited 
 * 
 * @warning 本代码仅供珠海科技学院学生学习和研究学习使用
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
 * @see 相关实现文件: VTM03_Handler.c
 * @see 相关模块: VTM03_Bsp.h (基础数据类型)、RingBuffer.h (环形缓冲区)
 * @see 相关技术文档: VTM03通信协议手册
 * @see 如有问题请联系作者: 13825821198@163.com
 */ 

#include "VTM03_Bsp.h"

#define IS_POWER_OF_TWO(x) (((x) != 0) && (((x) & ((x) - 1)) == 0))

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
    #define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#elif defined(__cplusplus) && __cplusplus >= 201103L
    #define STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
    #define STATIC_ASSERT(cond, msg) \
        extern int static_assertion_##__LINE__[(cond) ? 1 : -1]
#endif

#define CHECK_BUFFER_SIZE(size) \
    STATIC_ASSERT(IS_POWER_OF_TWO(size), "Buffer size must be a power of two")

#define VTM_BUFFER_SIZE 256

CHECK_BUFFER_SIZE(VTM_BUFFER_SIZE);

typedef enum
{
	VTM_OK = 0,
	VTM_Handle_Error = 1,
	VTM_PACKET_NOTFIND_FRAME_HEADER = 2,
	VTM_PACKEY_LESS = 3,
	VTM_CRC_ERROR = 4,
    VTM_PACKET_ERROR = 5
}VTM_STATUS_t;

typedef struct 
{
	bool last_rc_stop_status;
	bool last_rc_left_key_status;
	bool last_rc_right_key_status;
	bool last_rc_trigger_status;
	bool last_mouse_left_status;
	bool last_mouse_right_status;
	bool last_mouse_mid_status;
	bool last_kb_w_status;
	bool last_kb_s_status;
	bool last_kb_a_status;
	bool last_kb_d_status;
	bool last_kb_shift_status;
	bool last_kb_ctrl_status;
	bool last_kb_q_status;
	bool last_kb_e_status;
	bool last_kb_r_status;
	bool last_kb_f_status;
	bool last_kb_g_status;
	bool last_kb_z_status;
	bool last_kb_x_status;
	bool last_kb_c_status;
	bool last_kb_v_status;
	bool last_kb_b_status;
}VTM_Data_LastStatus_t;

typedef struct 
{
	bool rc_stop_flag;
	bool rc_left_key_flag;
	bool rc_right_key_flag;
	bool rc_trigger_flag;
	bool mouse_left_flag;
	bool mouse_right_flag;
	bool mouse_mid_flag;
	bool kb_w_flag;
	bool kb_s_flag;
	bool kb_a_flag;
	bool kb_d_flag;
	bool kb_shift_flag;
	bool kb_ctrl_flag;
	bool kb_q_flag;
	bool kb_e_flag;
	bool kb_r_flag;
	bool kb_f_flag;
	bool kb_g_flag;
	bool kb_z_flag;
	bool kb_x_flag;
	bool kb_c_flag;
	bool kb_v_flag;
	bool kb_b_flag;
}VTM_Data_Flag_t;

typedef struct 
{	
	RingBuffer_t rb;
	UART_HandleTypeDef *huart;
	osThreadId_t task_handle;
	VTM_Data_t data;
	VTM_Data_LastStatus_t data_last_status;
	VTM_Data_Flag_t data_flag;
	unsigned long time;
}VTM_Handle_t;
extern VTM_Handle_t VTM_Handle;

VTM_STATUS_t VTM_Handle_Init(VTM_Handle_t *vt,UART_HandleTypeDef *huart,osThreadId_t task_handle,
														uint8_t *vtm_buffer, size_t buffer_size);
VTM_STATUS_t VTM_Reset(VTM_Handle_t *vt);
VTM_STATUS_t VTM_Recv_Process(VTM_Handle_t *vt);
VTM_STATUS_t VTM_Event_Callback(VTM_Handle_t *vt, uint16_t Size);

#endif
