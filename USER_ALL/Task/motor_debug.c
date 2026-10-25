#include "motor_debug.h"
#include "FreeRTOS.h"
#include "task.h"

/* 全局数据数组 - 由接收任务写入 */
MotorDebugData_t motor_debug_data[MOTOR_DEBUG_NUM] = {0};

static TaskHandle_t motor_debug_taskHandle = NULL;

void MotorDebug_Init(void)
{
    BaseType_t xReturned;
    xReturned = xTaskCreate(
        MotorDebug_PrintTask,
        "MotorDebugPrint",
        256,                    /* stack size, 够用了 */
        NULL,
        osPriorityLow,          /* 最低优先级，不影响任何实时任务 */
        &motor_debug_taskHandle
    );
    (void)xReturned;
}

/**
 * @brief 调试打印任务 - 每 50ms 打印一次各轮子数据
 * @note  优先级最低，只在 CPU 空闲时运行
 */
void MotorDebug_PrintTask(void *argument)
{
    (void)argument;

    /* 打印表头 */
    printf("\r\n========== Motor Debug Start ==========\r\n");
    printf("  Time  | M1_T(Nm) M1_W(rad/s) | M2_T(Nm) M2_W(rad/s) | "
           "M3_T(Nm) M3_W(rad/s) | M4_T(Nm) M4_W(rad/s)\r\n");
    printf("--------|----------------------|----------------------|"
           "----------------------|----------------------\r\n");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t tick_ms = 0;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));

        printf("%7lu |", (unsigned long)tick_ms);
        for (int i = 0; i < MOTOR_DEBUG_NUM; i++)
        {
            printf(" %+7.4f %+10.2f |",
                   (double)motor_debug_data[i].wheel_t,
                   (double)motor_debug_data[i].wheel_w);
        }
        printf("\r\n");

        tick_ms += 50;
    }
}