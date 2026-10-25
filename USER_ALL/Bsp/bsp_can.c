#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
HAL_StatusTypeDef status;
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
	  //can1专属过滤器bank
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  can_filter_st.SlaveStartFilterBank = 14;
	  
	  //配置can过滤器并查看结果
    status = HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	  while(status!=HAL_OK){};
			
	  //使能can1+使能FIFO0消息待处理中断
    status = HAL_CAN_Start(&hcan1);
		while(status!=HAL_OK){};
    status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		while(status!=HAL_OK){};
			
    /*****************can2专属配置*****************/
    can_filter_st.FilterBank = 14;
		can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;//匹配消息存入FIFO1
			
		//配置can2过滤器，检查状态
    status = HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
		while(status!=HAL_OK){};	
			
		//启动can2 + 使能FIFO1消息待处理中断(触发HAL_CAN_RxFifo1MsgPendingCallback)
    status = HAL_CAN_Start(&hcan2);
		while(status!=HAL_OK){};
    status = HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
		while(status!=HAL_OK){};
}

uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    if (hcan == NULL || data == NULL || len > 8) {
        return 1; 
    }
    
    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;
    

    uint32_t timeout = 0xFFFF;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if (--timeout == 0) return 1;
    }
    

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
        return 1;
    }
    
    return 0;
}
