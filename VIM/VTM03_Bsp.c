#include "VTM03_Bsp.h"

uint8_t VTM_Frame_Header[VTM_FRAME_HEADER_SIZE] = {FRAME_HEADER_1,FRAME_HEADER_2};

bool VTM_Data_Init(VTM_Data_t *data)
{
	if(data == NULL)return false;
	data->rc_ch0 		  = 0;	
	data->rc_ch1 		  = 0;
	data->rc_ch2 		  = 0;
	data->rc_ch3 		  = 0;
	data->rc_s1 		  = 1;
	data->rc_stop	  	  = false;
	data->rc_left_key  	  = false;
	data->rc_right_key 	  = false;
	data->rc_trigger 	  = false;
	data->rc_pause_button = 0;
	data->mouse_x 		  = 0;
	data->mouse_y 		  = 0;
	data->mouse_z 		  = 0;
	data->mouse_left  	  = false;
	data->mouse_right 	  = false;
	data->mouse_mid 	  = false;
	data->kb_w			  = false;
	data->kb_s			  = false;
	data->kb_a			  = false;
	data->kb_d			  = false;
	data->kb_shift 		  = false;
	data->kb_ctrl 		  = false;
	data->kb_q 			  = false;
	data->kb_e 			  = false;
	data->kb_r 			  = false;
	data->kb_f 			  = false;
	data->kb_g 			  = false;
	data->kb_z 			  = false;
	data->kb_x 			  = false;
	data->kb_c 			  = false;
	data->kb_v 			  = false;
	data->kb_b 			  = false;
	return true;
}

bool VTM_Data_Process(VTM_Data_t *data,uint8_t *packet)
{
	if(data == NULL || packet == NULL)return false;

	data->rc_ch0  = (int16_t)(((packet[0] 	 )|(packet[1] << 8)) & 0x07FF) - VTM_Mid_Value;
	data->rc_ch1  = (int16_t)(((packet[1] >> 3 )|(packet[2] << 5)) & 0x07FF) - VTM_Mid_Value;
	data->rc_ch2  = (int16_t)(((packet[2] >> 6 )|(packet[3] << 2) | (packet[4] << 10)) & 0x07FF) - VTM_Mid_Value;
	data->rc_ch3  = (int16_t)(((packet[4] >> 1 )|(packet[5] << 7)) & 0x07FF) - VTM_Mid_Value;
	data->rc_s1   = (packet[5] >> 4) & 0x03;
	data->rc_stop = (packet[5] >> 6) & 0x01; 
	data->rc_left_key = (packet[5] >> 7) & 0x01;
	data->rc_right_key = packet[6] & 0x01;
	data->rc_pause_button = (int16_t)(((packet[6] >> 1) | (packet[7] << 7)) & 0x07FF) - VTM_Mid_Value;
	data->rc_trigger		= (packet[7] >> 4)&0x01;
	
	data->mouse_x	= (int16_t)(packet[8] | packet[9] << 8);
	data->mouse_y	= (int16_t)(packet[10] | packet[11] << 8);
	data->mouse_z	= (int16_t)(packet[12] | packet[13] << 8);
	data->mouse_left = packet[14] & 0x03;
	data->mouse_right = (packet[14] >> 2) & 0x03;
	data->mouse_mid  = (packet[14] >> 4) & 0x03;
	
	data->kb_w	  = packet[15] & 0x01;
	data->kb_s	  = (packet[15] >> 1) & 0x01;
	data->kb_a	  = (packet[15] >> 2) & 0x01;
	data->kb_d	  = (packet[15] >> 3) & 0x01;
	data->kb_shift = (packet[15] >> 4) & 0x01;
	data->kb_ctrl  = (packet[15] >> 5) & 0x01;
	data->kb_q	  = (packet[15] >> 6) & 0x01;
	data->kb_e	  = (packet[15] >> 7) & 0x01;
	data->kb_r	  =  packet[16] & 0x01;
	data->kb_f	  = (packet[16] >> 1) & 0x01;
	data->kb_g	  = (packet[16] >> 2) & 0x01;
	data->kb_z	  = (packet[16] >> 3) & 0x01;
	data->kb_x	  = (packet[16] >> 4) & 0x01;
	data->kb_c	  = (packet[16] >> 5) & 0x01;
	data->kb_v	  = (packet[16] >> 6) & 0x01;
	data->kb_b	  = (packet[16] >> 7) & 0x01;
	return true;
}
