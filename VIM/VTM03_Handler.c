#include "VTM03_Handler.h"

/**
 * @file: VTM03_Handler.c
 * @author: linruiyuan
 * @date: 2026-3-10
 * @version: 1.0
 * @description: VTM03遥控器数据接收Handler层实现文件
 *               本文件实现了VTM03设备（遥控器/图传一体模块）的数据接收、解析与处理接口
 *               包含CRC16校验、数据包解析、按键状态跟踪及标志位生成等功能
 *               基于环形缓冲区实现高效的数据流处理，支持DMA空闲中断接收
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
 * @see 相关头文件: VTM03_Handler.h
 * @see 相关模块: RingBuffer.h (环形缓冲区)
 * @see 相关技术文档: VTM03通信协议手册（具体版本请参考官方文档）
 * @see 如有问题请联系作者: 13825821198@163.com
 */

VTM_Handle_t VTM_Handle;

extern uint8_t VTM_Frame_Header[2];

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
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

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

static bool VTM_Data_LastStatus_Init(VTM_Data_LastStatus_t *data_last_status)
{
    if(data_last_status == NULL)return false;
    data_last_status->last_rc_stop_status = false;
    data_last_status->last_rc_left_key_status = false;
    data_last_status->last_rc_right_key_status = false;
    data_last_status->last_rc_trigger_status = false;
    data_last_status->last_mouse_left_status = false;
    data_last_status->last_mouse_right_status = false;
    data_last_status->last_mouse_mid_status = false;
    data_last_status->last_kb_w_status = false;
    data_last_status->last_kb_s_status = false;
    data_last_status->last_kb_a_status = false;
    data_last_status->last_kb_d_status = false;
    data_last_status->last_kb_shift_status = false;
    data_last_status->last_kb_ctrl_status = false;
    data_last_status->last_kb_q_status = false;
    data_last_status->last_kb_e_status = false;
    data_last_status->last_kb_r_status = false;
    data_last_status->last_kb_f_status = false;
    data_last_status->last_kb_g_status = false;
    data_last_status->last_kb_z_status = false;
    data_last_status->last_kb_x_status = false;
    data_last_status->last_kb_c_status = false;
    data_last_status->last_kb_v_status = false;
    data_last_status->last_kb_b_status = false;
    return true;
}

static bool VTM_Data_Flag_Init(VTM_Data_Flag_t *data_flag)
{
    if(data_flag == NULL)return false;
    data_flag->rc_stop_flag = false;
    data_flag->rc_left_key_flag = false;
    data_flag->rc_right_key_flag = false;
    data_flag->rc_trigger_flag = false;
    data_flag->mouse_left_flag = false;
    data_flag->mouse_right_flag = false;
    data_flag->mouse_mid_flag = false;
    data_flag->kb_w_flag = false;
    data_flag->kb_s_flag = false;
    data_flag->kb_a_flag = false;
    data_flag->kb_d_flag = false;
    data_flag->kb_shift_flag = false;
    data_flag->kb_ctrl_flag = false;
    data_flag->kb_q_flag = false;
    data_flag->kb_e_flag = false;
    data_flag->kb_r_flag = false;
    data_flag->kb_f_flag = false;
    data_flag->kb_g_flag = false;
    data_flag->kb_z_flag = false;
    data_flag->kb_x_flag = false;
    data_flag->kb_c_flag = false;
    data_flag->kb_v_flag = false;
    data_flag->kb_b_flag = false;
    return true;
}

VTM_STATUS_t VTM_Handle_Init(VTM_Handle_t *vt,UART_HandleTypeDef *huart,osThreadId_t task_handle,
			uint8_t *vtm_buffer, size_t buffer_size)
{
	if(vt == NULL ||RingBuffer_Init(&vt->rb,vtm_buffer,buffer_size) != RB_OK || huart == NULL || task_handle == NULL)
	{
	 return VTM_Handle_Error;
	}

	vt->huart = huart;
	vt->task_handle = task_handle;
	vt->time = xTaskGetTickCount();
	if(!VTM_Data_Init(&vt->data) || !VTM_Data_LastStatus_Init(&vt->data_last_status) || !VTM_Data_Flag_Init(&vt->data_flag))
	{
		return VTM_Handle_Error;
	}
	HAL_UARTEx_ReceiveToIdle_DMA(huart,vtm_buffer,buffer_size);
	return VTM_OK;
}

VTM_STATUS_t VTM_Reset(VTM_Handle_t *vt)
{
	if(vt != NULL)
	{	
		Rb_Reset(&vt->rb);
		vt->time = xTaskGetTickCount();
		if(!VTM_Data_Init(&vt->data) || !VTM_Data_LastStatus_Init(&vt->data_last_status) || !VTM_Data_Flag_Init(&vt->data_flag))
		{
			return VTM_Handle_Error;
		}
//		uint8_t timeover = 0x3F;
//		if(SendVTMOverTimeInstruction(&timeover) != HAL_OK)
//		{
//			return VTM_Handle_Error;
//		}
		return VTM_OK;
	}else return VTM_Handle_Error;
}

static inline void flag_process(bool status,bool *last_status,bool *flag)
{
    if(status == true && *last_status == false)
    {
        *flag = !*flag;
        *last_status = true;
    }
    else if(status == false && *last_status == true)
    {
        *last_status = false;
    }else
    {
        return;
    }
}

static VTM_STATUS_t VTM_Flag_Process(VTM_Handle_t *vt)
{
	if(vt == NULL)return VTM_Handle_Error;
	flag_process(vt->data.rc_stop,&vt->data_last_status.last_rc_stop_status,&vt->data_flag.rc_stop_flag);
    flag_process(vt->data.rc_left_key,&vt->data_last_status.last_rc_left_key_status,&vt->data_flag.rc_left_key_flag);
    flag_process(vt->data.rc_right_key,&vt->data_last_status.last_rc_right_key_status,&vt->data_flag.rc_right_key_flag);
    flag_process(vt->data.rc_trigger,&vt->data_last_status.last_rc_trigger_status,&vt->data_flag.rc_trigger_flag);
    flag_process(vt->data.mouse_left,&vt->data_last_status.last_mouse_left_status,&vt->data_flag.mouse_left_flag);
    flag_process(vt->data.mouse_right,&vt->data_last_status.last_mouse_right_status,&vt->data_flag.mouse_right_flag);
    flag_process(vt->data.mouse_mid,&vt->data_last_status.last_mouse_mid_status,&vt->data_flag.mouse_mid_flag);
    flag_process(vt->data.kb_w,&vt->data_last_status.last_kb_w_status,&vt->data_flag.kb_w_flag);
    flag_process(vt->data.kb_s,&vt->data_last_status.last_kb_s_status,&vt->data_flag.kb_s_flag);
    flag_process(vt->data.kb_a,&vt->data_last_status.last_kb_a_status,&vt->data_flag.kb_a_flag);
    flag_process(vt->data.kb_d,&vt->data_last_status.last_kb_d_status,&vt->data_flag.kb_d_flag);
    flag_process(vt->data.kb_shift,&vt->data_last_status.last_kb_shift_status,&vt->data_flag.kb_shift_flag);
    flag_process(vt->data.kb_ctrl,&vt->data_last_status.last_kb_ctrl_status,&vt->data_flag.kb_ctrl_flag);
    flag_process(vt->data.kb_q,&vt->data_last_status.last_kb_q_status,&vt->data_flag.kb_q_flag);
    flag_process(vt->data.kb_e,&vt->data_last_status.last_kb_e_status,&vt->data_flag.kb_e_flag);
    flag_process(vt->data.kb_r,&vt->data_last_status.last_kb_r_status,&vt->data_flag.kb_r_flag);
    flag_process(vt->data.kb_f,&vt->data_last_status.last_kb_f_status,&vt->data_flag.kb_f_flag);
    flag_process(vt->data.kb_g,&vt->data_last_status.last_kb_g_status,&vt->data_flag.kb_g_flag);
    flag_process(vt->data.kb_z,&vt->data_last_status.last_kb_z_status,&vt->data_flag.kb_z_flag);
    flag_process(vt->data.kb_x,&vt->data_last_status.last_kb_x_status,&vt->data_flag.kb_x_flag);
    flag_process(vt->data.kb_c,&vt->data_last_status.last_kb_c_status,&vt->data_flag.kb_c_flag);
    flag_process(vt->data.kb_v,&vt->data_last_status.last_kb_v_status,&vt->data_flag.kb_v_flag);
    flag_process(vt->data.kb_b,&vt->data_last_status.last_kb_b_status,&vt->data_flag.kb_b_flag);
	return VTM_OK;
}
VTM_STATUS_t VTM_Recv_Process(VTM_Handle_t *vt)
{
	if(Rb_Location(&vt->rb,VTM_Frame_Header,VTM_FRAME_HEADER_SIZE) != RB_OK)
	{
		return VTM_PACKET_NOTFIND_FRAME_HEADER;
	}

	if(vt->rb.recv_size < VTM_PACKEY_SIZE)
	{
		return VTM_PACKEY_LESS;
	}

	vt->time = xTaskGetTickCount();

	uint8_t packet[VTM_PACKEY_SIZE];

	if(Rb_Take(&vt->rb,packet,VTM_PACKEY_SIZE) != RB_OK)
    {
        return VTM_PACKET_ERROR;
    }

//	uint16_t crc_value = HAL_CRC_Calculate(&hcrc,(uint32_t *)packet,VTM_PACKEY_SIZE- 2) & 0xFFFF;

//	if(crc_value == ((packet[19]) | (packet[20] << 8)))
//	{
//			SendVTMDataInstruction(packet);
//			if(!VTM_Data_Process(&vt->data,&packet[2]) || !VTM_Flag_Process(vt))
//			{
//				return VTM_Handle_Error;
//			}
//			return VTM_OK;
//	}else
//	{
//		return VTM_CRC_ERROR;
//	}
		if(verify_crc16_check_sum(packet,VTM_PACKEY_SIZE) == true)
		{
//			SendVTMDataInstruction(packet);
			if(!VTM_Data_Process(&vt->data,&packet[2]) || !VTM_Flag_Process(vt))
			{
				return VTM_Handle_Error;
			}
			return VTM_OK;
		}else return VTM_CRC_ERROR;
}

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

VTM_STATUS_t VTM_Event_Callback(VTM_Handle_t *vt, uint16_t Size)
{
	if(Rb_PushFromCallback(&vt->rb,Size) != RB_ERROR)
	{
		osTaskNotifyFromISR(vt->task_handle);
		return VTM_OK;
	}else return VTM_Handle_Error;
}
