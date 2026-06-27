#ifndef VTM03_BSP_H
#define VTM03_BSP_H

#include "string.h"
#include "usart.h"
#include "stdbool.h"
//#include "FDCANHandler.h"
//#include "crc.h"
#include "RingBuffer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"

#define VTM_PACKEY_SIZE 21
#define FRAME_HEADER_1 0xA9
#define FRAME_HEADER_2 0x53
#define VTM_FRAME_HEADER_SIZE 2

#define VTM_Mid_Value 1024

typedef struct
{
	int16_t rc_ch0;   			 /*右摇杆X轴*/
 	int16_t rc_ch1;   			 /*右摇杆Y轴*/
 	int16_t rc_ch2;   			 /*左摇杆X轴*/
 	int16_t rc_ch3;   			 /*左摇杆Y轴*/
 	uint8_t rc_s1;    			 /*挡位切换开关*/
	bool		rc_stop;	 			 /*暂停按键*/
	bool		rc_left_key;		 /*左侧按键*/
	bool		rc_right_key;		 /*右侧按键*/
	bool		rc_trigger;			 /*扳机键*/
	int16_t     rc_pause_button;
	int16_t     mouse_x;		 /*鼠标x轴速度*/
	int16_t     mouse_y;		 /*鼠标y轴速度*/
	int16_t     mouse_z;		 /*鼠标滚轮速度*/
	bool		mouse_left;
	bool		mouse_right;
	bool		mouse_mid;
	bool		kb_w;
	bool		kb_s;
	bool		kb_a;
	bool		kb_d;
	bool		kb_shift;
	bool		kb_ctrl;
	bool		kb_q;
	bool		kb_e;
	bool		kb_r;
	bool		kb_f;
	bool		kb_g;
	bool		kb_z;
	bool		kb_x;
	bool    kb_c;
	bool		kb_v;
	bool		kb_b;
}VTM_Data_t;

bool VTM_Data_Init(VTM_Data_t *data);
bool VTM_Data_Process(VTM_Data_t *data,uint8_t *packet);

#endif
