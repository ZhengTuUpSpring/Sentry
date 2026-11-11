/**
 * @file: RingBuffer.c
 * @author: linruiyuan
 * @date: 2026-2-7
 * @version: 1.0
 * @description: 环形缓冲区实现文件
 *               提供环形缓冲区的初始化、数据压入、查找、读取和重置等功能
 *               适用于嵌入式系统中的数据流处理，特别是串口数据接收场景
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
 * @see 相关头文件: RingBuffer.h
 * @see 如有问题请联系作者: 13825821198@163.com
*/

#include  "RingBuffer.h"

Rb_Status_t RingBuffer_Init(RingBuffer_t *rb,uint8_t *buffer, uint16_t buffer_size)
{
	if(rb == NULL || buffer == NULL || buffer_size == 0)
	return RB_ERROR;
	rb->pbuffer = buffer;
	rb->buffer_size = buffer_size;
	rb->head = 0;
	rb->tail = 0;
	rb->recv_size = 0;
	return RB_OK;
}

Rb_Status_t Rb_PushFromCallback(RingBuffer_t *rb,uint16_t currentPos)
{
	if(rb == NULL)
	{
		return RB_ERROR;
	}
	uint16_t oldPos = rb->tail;

	rb->tail = currentPos;

	uint16_t bytesWritten = currentPos >= oldPos ?
	                       	currentPos - oldPos :
	                       	rb->buffer_size - oldPos + currentPos;

	if(bytesWritten + rb->recv_size > rb->buffer_size)
	{
		uint16_t overwrittenBytes = (bytesWritten + rb->recv_size) - rb->buffer_size;
		rb->head = (rb->head + overwrittenBytes) & (rb->buffer_size-1);
		rb->recv_size = rb->buffer_size;
		return RB_FULL;
	}
	else
	{
		rb->recv_size += bytesWritten;
		return RB_OK;
	}
}

Rb_Status_t Rb_Location(RingBuffer_t *rb, uint8_t *data, uint8_t length)
{
    if(rb == NULL || data == NULL || length == 0 || length > rb->recv_size)
    {
        return RB_ERROR;
    }
    
    // 在环形缓冲区中搜索，考虑跨边界情况
    for(uint16_t offset = 0; offset <= rb->recv_size - length; offset++)
    {
        bool found = true;
        
        // 比较每个字节
        for(uint8_t i = 0; i < length; i++)
        {
            uint16_t index = (rb->head + offset + i) & (rb->buffer_size-1);
            if(rb->pbuffer[index] != data[i])
            {
                found = false;
                break;
            }
        }
        if(found)
        {
            // 找到匹配，更新缓冲区状态
            rb->head = (rb->head + offset) & (rb->buffer_size-1);
            rb->recv_size -= offset;
            return RB_OK;
        }
    }
    
    // 没找到，清除数据
    rb->head = (rb->head + rb->recv_size) & (rb->buffer_size-1);
    rb->recv_size = 0;
    return RB_EMPTY;
}

Rb_Status_t Rb_Take(RingBuffer_t *rb, uint8_t *data, uint16_t length)
{
	if(rb == NULL || data == NULL || length == 0)
	{
		return RB_ERROR;
	}
	if(rb->recv_size == 0)
	{
		return RB_EMPTY;
	}
	if(length > rb->recv_size)
	{
		return RB_LEN_LESS;
	}
//	uint16_t readLength = (length > rb->recv_size) ? rb->recv_size : length;

	// 4. 读取数据，处理可能的回绕
	uint16_t SpaceToBufferEnd = rb->buffer_size - rb->head;
	if (length <= SpaceToBufferEnd) {
		// 可以直接读取连续数据
		memcpy(data, rb->pbuffer + rb->head, length);
		rb->head += length;
		// 检查head是否需要回绕
		if (rb->head == rb->buffer_size) {
			rb->head = 0;
		}
	} else {
		// 数据需要回绕
		uint16_t firstlen = SpaceToBufferEnd;
		uint16_t secondlen = length - firstlen;

		memcpy(data, rb->pbuffer + rb->head, firstlen); // 读取第一部分
		memcpy(data + firstlen, rb->pbuffer, secondlen); // 读取第二部分
		rb->head = secondlen; // 更新 head 到第二部分数据之后的位置

	}
	rb->recv_size -= length;
	return RB_OK;
}

Rb_Status_t Rb_Peek(RingBuffer_t *rb, uint8_t *data, uint16_t offset, uint16_t length)
{
	if(rb == NULL || data == NULL || length == 0)
	{
		return RB_ERROR;
	}
	if(offset + length > rb->recv_size)
	{
		return RB_PEEK_LEN_ERROR;
	}
	
	uint16_t ReadIdx = rb->head + offset & (rb->buffer_size-1);
	uint16_t SpaceToBufferEnd = rb->buffer_size - ReadIdx;

	if (length <= SpaceToBufferEnd) {
		memcpy(data, rb->pbuffer + ReadIdx, length);
	} else {
		uint16_t firstlen = SpaceToBufferEnd;
		uint16_t secondlen = length - firstlen;

		memcpy(data, rb->pbuffer + ReadIdx, firstlen); // 读取第一部分
		memcpy(data + firstlen, rb->pbuffer, secondlen); // 读取第二部分
	}
	return RB_OK;
}

Rb_Status_t Rb_Reset(RingBuffer_t *rb)
{
	if(rb == NULL)
	{
		return RB_ERROR;
	}
	rb->head = 0;
	rb->tail = 0;
	rb->recv_size = 0;
	return RB_OK;
}


