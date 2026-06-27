/**
 * @file: RingBuffer.h
 * @author: linruiyuan
 * @date: 2026-2-7
 * @version: 1.0
 * @description: 环形缓冲区头文件
 *               定义环形缓冲区相关的数据结构、枚举状态和函数声明
 *               提供线程安全的环形缓冲区操作接口，适用于嵌入式实时系统
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
 * @see 相关实现文件: RingBuffer.c
 * @see 如有问题请联系作者: 13825821198@163.com
 * 
 * @details 数据结构说明：
 *          1. Rb_Status_t - 环形缓冲区操作状态枚举
 *             - RB_OK               : 操作成功
 *             - RB_ERROR            : 参数错误或操作失败
 *             - RB_FULL             : 缓冲区已满
 *             - RB_EMPTY            : 缓冲区为空
 *             - RB_LEN_LESS         : 请求长度超过可用数据
 *             - RB_PEEK_LEN_ERROR   : Peek操作长度错误
 *          
 *          2. RingBuffer_t - 环形缓冲区结构体
 *             - pbuffer      : 数据缓冲区指针
 *             - buffer_size  : 缓冲区大小(必须是2的幂次方)
 *             - head         : 读指针(volatile修饰，支持多线程)
 *             - tail         : 写指针(volatile修饰，支持多线程)
 *             - recv_size    : 缓冲区中有效数据长度
 * 
 * @note 重要技术说明：
 *       1. 缓冲区大小必须为2的幂次方，这是为了使用位与运算(&)替代取模运算(%)提高效率
 *       2. head和tail使用volatile修饰，确保在多线程/中断环境中变量的可见性
 *       3. 本环形缓冲区采用读指针和写指针分离的设计，支持单生产者单消费者模式
 * 
 * @function 函数功能概述：
 *           RingBuffer_Init     - 初始化环形缓冲区
 *           Rb_PushFromCallback - 从回调函数中更新数据写入位置
 *           Rb_Location         - 在缓冲区中查找指定数据序列
 *           Rb_Take             - 从缓冲区读取并移除数据
 *           Rb_Peek             - 查看缓冲区数据但不移除
 *           Rb_Reset            - 重置缓冲区状态
 */
#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

typedef enum 
{
	RB_OK = 0,
	RB_ERROR = 1,
	RB_FULL = 2,
	RB_EMPTY = 3,
	RB_LEN_LESS = 4,
	RB_PEEK_LEN_ERROR = 5
}Rb_Status_t;

typedef struct
{
	uint8_t *pbuffer;
	uint16_t buffer_size;
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint16_t recv_size;
}RingBuffer_t;

// 函数声明
Rb_Status_t RingBuffer_Init(RingBuffer_t *rb,uint8_t *buffer, uint16_t buffer_size);
Rb_Status_t Rb_PushFromCallback(RingBuffer_t *rb,uint16_t currentPos);
Rb_Status_t Rb_Location(RingBuffer_t *rb, uint8_t *data, uint8_t length);
Rb_Status_t Rb_Take(RingBuffer_t *rb, uint8_t *data, uint16_t length);
Rb_Status_t Rb_Peek(RingBuffer_t *rb, uint8_t *data, uint16_t offset, uint16_t length);
Rb_Status_t Rb_Reset(RingBuffer_t *rb);

#endif
