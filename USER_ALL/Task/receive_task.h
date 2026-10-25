#ifndef RECEIVE_TASK
#define RECEIVE_TASK

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "bsp_dji.h"
#include "semphr.h"

extern TaskHandle_t chassis_receive_taskHandle;	// main thread
extern TaskHandle_t  chassis_yaw_receive_taskHandle;
extern TaskHandle_t YAWImuReceive_taskHandle; 

extern QueueHandle_t chassisMotorReceive_queueHandle;
extern QueueHandle_t chassisMotorReceiveAdv_queueHandle;
extern QueueHandle_t YAWMotorReceive_queueHandle;;
extern QueueHandle_t chassisPOWERReceive_queueHandle;;
//底盘电机数据获取
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}ChassisMotorReceiveQueueData_t;

//上板速度获取
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}ChassisReceiveFromAdvQueueData_t;

//大yaw4310数据获取
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}ChassisYawMotorReceiveQueueData_t;

typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}ChassisPOWERReceiveQueueData_t;

//上板IMU数据获取(队列传输用的数据包格式)
typedef struct{
    uint16_t id;
    uint8_t rx_data[8];
}YAWIMUReceiveQueueData_t;
extern QueueHandle_t YAWImuReceive_queueHandle;

//IMU数据解析后的、可直接用于控制算法的姿态数据
typedef struct{
		float pitch;
		float roll;
		float yaw;
	  float z_gry;
	  float yaw_last;
		float yaw_count;
		float yaw_total_angle;
}YAWImuReceiveData_t;
extern YAWImuReceiveData_t YAWImuReceiveData;

/* ===== 上板 IMU (0x200) 数据时效 =====
 * 超时阈值: 上板正常周期按 100Hz(10ms) 估算, 取 50ms 允许丢 4~5 帧。
 * 超时后 YAWImuReceiveData.yaw 为陈旧值, 任何依赖它的坐标变换都会假漂,
 * 使用方必须通过 YAWImu_IsValid() 判定并降级(回零/停车)。 */
#define YAWIMU_TIMEOUT_MS   50
extern volatile uint32_t g_yawimu_rx_tick;
extern uint8_t YAWImu_IsValid(void);

/* 电机调试数据快照 - 接收任务写入，调试任务读取 */
#define MOTOR_DEBUG_NUM 4
typedef struct {
    float wheel_t;    /**< 预测转矩 (N·m) */
    float wheel_w;    /**< 转速反馈 (rad/s) */
} MotorDebugData_t;
extern MotorDebugData_t motor_debug_data[MOTOR_DEBUG_NUM];

/* ===== 同拍快照：由 Chassis_Task 在 Chassis_WheatWheel_Solution 内一次性填写 =====
 * 目的：让打印出的 4 轮 wheel_t/wheel_w 与 pre_chassis_power 属于同一控制拍。
 * 说明：4 个 M3508 反馈本身是 4 个独立 CAN 帧（1Mbps 下帧间约 111us），
 *       因此"严格同一物理时刻"不可能做到；本快照保证的是
 *       "同一次 Chassis_WheatWheel_Solution 调用 + 同一个 tick"，误差收敛到 <=1ms。
 * 注意：Chassis_WheatWheel_Solution 只在有效挡位分支下被调用，
 *       若 seq 长时间不变，说明控制没在跑（挡位不在有效分支 / 遥控掉线），
 *       此时快照为陈旧值 —— tick 与 seq 就是用来识别这种情况的。 */
typedef struct {
    float    wheel_t[4];            /**< 轮端预测转矩 (N·m) */
    float    wheel_w[4];            /**< 轮端转速 (rad/s) */
    float    pre_chassis_power;     /**< 预测底盘功率 (W) */
    uint32_t tick;                  /**< 快照生成时刻 (ms) */
    uint32_t seq; 	/**< 递增序号，用于识别丢拍/陈旧 */
	  float act_chassis_power;
} MotorSnapshot_t;
extern MotorSnapshot_t g_motor_snap;

/* 实际功率(act_chassis_power)最近一次收包时刻(ms)。功率来自 CAN1 0x2FF 超级电容板，
 * 与控制拍异步，用它和 g_motor_snap.tick 求差可算出功率滞后量。 */
extern volatile uint32_t g_act_power_rx_tick;

extern void Chassis_Receive_TaskInit(void);
extern void ChassisMotorReceive_Task(void *argumemt);
extern void Chassis_Yaw_Task(void *argumemt)  ;
extern void ChassisMotorReceiveAdv_Task(void *argumemt) ;
extern void YAWImuReceive_Task(void *argument);
extern void CBorad_IMURead(void *argument);
extern void MotorDebug_Init(void);
extern void Chassis_POWER_Task(void *argument);
/* 底盘本地BMI088 Yaw轴角速度 (rad/s) */
extern float CBorad_YawRate;

#endif
