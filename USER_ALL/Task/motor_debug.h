#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

#include <stdint.h>

/**
 * @brief 电机调试数据结构 - 每个轮子保存最新的 wheel_t 和 wheel_w
 * @note  由 receive_Task 写入，由 debug 任务读取打印
 *        仅写入 float 在 Cortex-M4 上是原子的，无需临界区保护
 */
#define MOTOR_DEBUG_NUM 4

typedef struct {
    float wheel_t;    /**< 预测转矩 (N·m) */
    float wheel_w;    /**< 转速反馈 (rad/s) */
} MotorDebugData_t;

extern MotorDebugData_t motor_debug_data[MOTOR_DEBUG_NUM];

void MotorDebug_PrintTask(void *argument);
void MotorDebug_Init(void);

#endif /* MOTOR_DEBUG_H */