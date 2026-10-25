#include "receive_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "4310_control.h"
#include "RC_control.h"
#include "string.h"
#include "limit_power_control.h"
#include <stdio.h>
#include "BMI088.h"
#include "spi.h"
#include "chassis_control.h"
#include "nav_task.h"
TaskHandle_t chassis_receive_taskHandle;	// main thread
TaskHandle_t  chassis_yaw_receive_taskHandle;
TaskHandle_t YAWImuReceive_taskHandle; 
TaskHandle_t CBorad_IMURead_taskHandle;
TaskHandle_t chassisPowerTaskHandle;

QueueHandle_t chassisMotorReceive_queueHandle;
QueueHandle_t YAWMotorReceive_queueHandle;
QueueHandle_t chassisMotorReceiveAdv_queueHandle;
QueueHandle_t YAWImuReceive_queueHandle;

QueueHandle_t chassisPOWERReceive_queueHandle;
YAWImuReceiveData_t YAWImuReceiveData;

/* ===== 上板 IMU (0x200) 数据时效监控 =====
 * 每次成功解析 0x200 时刷新 g_yawimu_rx_tick。
 * 上板掉线后 YAWImuReceiveData.yaw 会冻结在最后一帧, 而依赖它的坐标变换角
 * c_angle = yaw - gimbal_rel 仍会随 gimbal_rel / 底盘转动而假漂
 * (反向双倍自旋挡位下可达 2ω ≈ 616 °/s), 若不判定失效, 车会静默地
 * 越走越偏直至画圈 —— 必须靠时间戳兜底。
 * 注意: 0x300 遥控那边已有 100ms 超时清零(见 ChassisMotorReceiveAdv_Task),
 *       本处为 0x200 补齐对等保护, 阈值见 receive_task.h / YAWIMU_TIMEOUT_MS。 */
volatile uint32_t g_yawimu_rx_tick = 0;

/**
 * @brief  判断上板 IMU (0x200) 数据是否仍在有效期内
 * @param  none
 * @retval 1=有效  0=超时未更新(或上电后尚未收到过任何一帧)
 */
uint8_t YAWImu_IsValid(void)
{
    TickType_t now;

    if (g_yawimu_rx_tick == 0) {
        return 0;                       /* 上电后还没收到过 */
    }

    now = xTaskGetTickCount();
    /* TickType_t 为 32bit 无符号, 差值天然处理回绕 */
    return ((uint32_t)(now - g_yawimu_rx_tick)
            <= (uint32_t)pdMS_TO_TICKS(YAWIMU_TIMEOUT_MS)) ? 1 : 0;
}

/* 电机调试数据快照 - 接收任务写入，调试任务读取并 printf */
MotorDebugData_t motor_debug_data[MOTOR_DEBUG_NUM] = {0};

/* ===== 同拍快照实例 =====
 * 由 Chassis_Task 在 Chassis_WheatWheel_Solution() 内填写（见 chassis_control.c），
 * 打印任务只读不写。定义放这里是为了与 motor_debug_data 放在一起便于对照。
 * 注：motor_debug_data 的写入暂时保留（旧路径，未删），打印已改走 g_motor_snap。 */
MotorSnapshot_t g_motor_snap = {0};
volatile uint32_t g_act_power_rx_tick = 0;
//整体接收任务初始化
void Chassis_Receive_TaskInit(void)
{
  BaseType_t xRetruned;
	
	chassis_receive_taskHandle = NULL;
	chassis_yaw_receive_taskHandle=NULL;
	
	xRetruned = xTaskCreate(
		ChassisMotorReceive_Task,
		"ChassisMotorReceive_Task",
		300,
		NULL,
		osPriorityNormal,
		&chassis_receive_taskHandle
	);
	while(xRetruned!=pdPASS){};	
		
			xRetruned = xTaskCreate(
		Chassis_Yaw_Task,
		"Chassis_Yaw_Task",
		125,
		NULL,
		osPriorityNormal,
		&chassis_yaw_receive_taskHandle
	);
	while(xRetruned!=pdPASS){};
		
	xRetruned = xTaskCreate(
		ChassisMotorReceiveAdv_Task,
		"ChassisMotorReceiveAdv_Task",
		125,
		NULL,
		osPriorityNormal,
		NULL
	);
	while(xRetruned!=pdPASS){};
		
		xRetruned = xTaskCreate(
		YAWImuReceive_Task,
		"Chassis_ImuReceive_task",
		125,
		NULL,
		osPriorityNormal,
		&YAWImuReceive_taskHandle
	);
	while(xRetruned!=pdPASS){};
		
		xRetruned = xTaskCreate(
		CBorad_IMURead,
		"CBorad_IMURead_task",
		300,
		NULL,
		osPriorityRealtime,
		&CBorad_IMURead_taskHandle
	);
	while(xRetruned!=pdPASS){};

	xTaskCreate(
    Chassis_POWER_Task,          // 任务函数名
    "chassisPower",              // 任务名（调试用，随便起）
    256,                         // 栈大小（word），根据实际情况调
    NULL,                        // 传给任务的参数，不需要就 NULL
    3,                           // 优先级，数字越大越高
    &chassisPowerTaskHandle      // 可选，不需要拿句柄就传 NULL
);	
		while(xRetruned!=pdPASS){};
}

/**
  * @brief          接收底盘电机电调反馈数据
  * @param[in]      none
  * @retval         none
  */
void ChassisMotorReceive_Task(void *argumemt)
{
	/* 创建队列接收电调的数据 */
	chassisMotorReceive_queueHandle = xQueueCreate(40, sizeof(ChassisMotorReceiveQueueData_t));
	while(chassisMotorReceive_queueHandle == NULL){};
  /* 创建队列数据接收结构体实例和接收状态量 */
	BaseType_t xStatus;
	ChassisMotorReceiveQueueData_t chassisMotor_queueData;
	for(;;)
	{
		xStatus = xQueueReceive(chassisMotorReceive_queueHandle, &chassisMotor_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(chassisMotor_queueData.id - CAN_CHASSIS_ALL_ID)
			{
				case 0x01:
				case 0x02:
				case 0x03:
				case 0x04:
				{
					uint8_t motor_idx = chassisMotor_queueData.id - CAN_CHASSIS_ALL_ID - 1;
					get_motor_measure(&dji_motorbox[motor_idx], chassisMotor_queueData.rx_data);
		
                motor_debug_data[motor_idx].wheel_t = (float)dji_motorbox[motor_idx].given_current * Wheel_T_Coeff;
                motor_debug_data[motor_idx].wheel_w = (float)dji_motorbox[motor_idx].speed_rpm * Velocity_Constant;


				}break;
				default:{
				}
			}
		}
		else{  
		}
	}
}

ChassisYawMotorReceiveQueueData_t yaw4310Motor_queueData;
void Chassis_Yaw_Task(void *argumemt)
{
	YAWMotorReceive_queueHandle = xQueueCreate(40, sizeof(ChassisYawMotorReceiveQueueData_t));
	configASSERT(YAWMotorReceive_queueHandle != NULL);

	BaseType_t xStatus;

	for(;;)
	{
		xStatus = xQueueReceive(YAWMotorReceive_queueHandle, &yaw4310Motor_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(yaw4310Motor_queueData.id)
			{
				case 0x05:{
					dm_motor_fbdata(&DM_4310, (uint8_t *)yaw4310Motor_queueData.rx_data);
				}break;
				default:{
				}
			}
		}
		else{
		}
	}
}
static float power;
static int32_t raw;

void Chassis_POWER_Task(void *argument)
{
	  chassisPOWERReceive_queueHandle = xQueueCreate(40, sizeof(ChassisPOWERReceiveQueueData_t));
    configASSERT(chassisPOWERReceive_queueHandle != NULL);
    ChassisPOWERReceiveQueueData_t d;
		
    for (;;)
    {
        if (xQueueReceive(chassisPOWERReceive_queueHandle, &d, portMAX_DELAY) == pdPASS)
        {
         raw= (uint32_t)d.rx_data[0] | ((uint32_t)d.rx_data[1] << 8)
             | ((uint32_t)d.rx_data[2] << 16) | ((uint32_t)d.rx_data[3] << 24);
         memcpy(&power, &raw, 4);        // 位模式 → float
				Chassis_Power_Modul.act_chassis_power=power;
				g_act_power_rx_tick = (uint32_t)xTaskGetTickCount();
        }
				osDelay(1);
    }
}

/**
  * @brief          接收上板（遥控）数据任务
  * @param[in]      none
  * @retval         none
  */
void ChassisMotorReceiveAdv_Task(void *argumemt)
{
	/* 创建队列接收上c板遥控的数据 */
	chassisMotorReceiveAdv_queueHandle = xQueueCreate(40, sizeof(ChassisReceiveFromAdvQueueData_t));
	while(chassisMotorReceiveAdv_queueHandle == NULL){};
  /* 创建队列数据接收结构体实例和接收状态量 */
	BaseType_t xStatus;
	ChassisReceiveFromAdvQueueData_t chassisMotorAdv_queueData;
	for(;;)
	{
		xStatus = xQueueReceive(chassisMotorReceiveAdv_queueHandle, &chassisMotorAdv_queueData, pdMS_TO_TICKS(100));
		if(xStatus != pdFALSE)
		{
			switch(chassisMotorAdv_queueData.id)
			{
				case 0x300:
				{
				  /* 处理收到上板传输的速度数据 并给回Chassis_Task做速度赋值 */
					
					RC_CAN_parse_to_RC_struct(chassisMotorAdv_queueData.rx_data, &rc);
					
				}break;
				default:{
				}
			}
		}
		else{
			/* 超过100ms未收到上板遥控数据，将rc结构体清零，防止失控 */
			memset(&rc, 0, sizeof(RC_t));
			
		}
	}
}
/**
  * @brief          接收上板IMU数据任务
  * @param[in]      none
  * @retval         none
  */
//定义一个IMU（惯性测量单元）数据接收队列的变量
YAWIMUReceiveQueueData_t YAWReceiveIMU_queueData;

void YAWImuReceive_Task(void *argument)
{
	YAWImuReceive_queueHandle = xQueueCreate(40, sizeof(YAWIMUReceiveQueueData_t));
	while(YAWImuReceive_queueHandle == NULL){};

	BaseType_t xStatus;
	
	for(;;)
	{
	   xStatus = xQueueReceive(YAWImuReceive_queueHandle, &YAWReceiveIMU_queueData, portMAX_DELAY);
		if(xStatus != pdFALSE)
		{
			switch(YAWReceiveIMU_queueData.id)
			{
				case 0x200:
				{
					YAWImuReceiveData.pitch = (float)((int16_t)(YAWReceiveIMU_queueData.rx_data[0]<<8) | (int16_t)YAWReceiveIMU_queueData.rx_data[1])/100.0f;
					YAWImuReceiveData.yaw = (float)((int16_t)(YAWReceiveIMU_queueData.rx_data[2]<<8) | (int16_t)YAWReceiveIMU_queueData.rx_data[3])/100.0f;
					YAWImuReceiveData.roll = (float)((int16_t)(YAWReceiveIMU_queueData.rx_data[4]<<8) | (int16_t)YAWReceiveIMU_queueData.rx_data[5])/100.0f;
					YAWImuReceiveData.z_gry = (float)((int16_t)(YAWReceiveIMU_queueData.rx_data[6]<<8) | (int16_t)YAWReceiveIMU_queueData.rx_data[7])/100.0f;
					/* 刷新数据时效时间戳(供 YAWImu_IsValid 判定) */
					g_yawimu_rx_tick = (uint32_t)xTaskGetTickCount();
				}break;
				default:{
				}
			}
		}
		else{
		}
	}
}

/**
  * @brief          底盘本地BMI088 IMU读取任务
  * @note           周期性读取BMI088六轴数据，提取Yaw角速度供控制算法使用
  * @param[in]      argument: 未使用
  * @retval         none
  */
void CBorad_IMURead(void *argument)
{
	(void)argument;

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(1); /* 1ms周期 ≈ 1000Hz */
	BMI088_Init(&hspi1,1);
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* 读取BMI088六轴数据到全局结构体 BMI088 */
		BMI088_Read(&BMI088);

	}
}

/* ===== 电机调试打印任务 ===== */
static TaskHandle_t motor_debug_taskHandle = NULL;
static void MotorDebug_PrintTask(void *argument);

void MotorDebug_Init(void)
{
	xTaskCreate(
		MotorDebug_PrintTask,
		"MotorDebugPrint",
		256,
		NULL,
		osPriorityLow,          /* 最低优先级, 不影响任何实时任务 */
		&motor_debug_taskHandle
	);
}

/**
 * @brief 调试打印任务 - 每 40ms 打印一次四个轮子的 wheel_t / wheel_w
 * @note  运行在最低优先级, 只在 CPU 空闲时执行, 不影响 CAN 接收和底盘控制
 */
void MotorDebug_PrintTask(void *argument)
{
	(void)argument;

	TickType_t last_wake = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(40); /* 40ms周期 = 25Hz */
	// vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100)); /* 原 100ms 周期 = 10Hz */

	for (;;)
	{
		vTaskDelayUntil(&last_wake, xPeriod);
//		printf("%f\r\n",YAWImuReceiveData.yaw);
//		printf("%f\r\n", power);       // 应该输出 5.27 左右
		//		printf("%f,%f\r\n",DM_4310.para.vel,BMI088.Gyro[2]);
//		printf("%f,%f\r\n",yaw_target_speed,DM_4310.++-.vel);
//		printf("%f,%f,%f\r\n",yaw_sin_target_speed,yaw_sin_current_speed,yaw_getpid_speed);
// printf("pwr=%.2f\r\n", power);
// for (int i = 0; i < MOTOR_DEBUG_NUM; i++)
// {
//    printf("  M%d: wheel_t=%.8f  wheel_w=%.4f\r\n", i + 1,
//           motor_debug_data[i].wheel_t,
//           motor_debug_data[i].wheel_w);
// }
//        printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//            motor_debug_data[0].wheel_t, motor_debug_data[0].wheel_w,
//            motor_debug_data[1].wheel_t, motor_debug_data[1].wheel_w,
//            motor_debug_data[2].wheel_t, motor_debug_data[2].wheel_w,
//            motor_debug_data[3].wheel_t, motor_debug_data[3].wheel_w,
//            Chassis_Power_Modul.wheel.pre_chassis_power);
//			printf("%f,%f\r\n",Chassis_Power_Modul.wheel.pre_chassis_power,Chassis_Power_Modul.act_chassis_power);

		/* ===== 同拍快照打印 =====
		 * 11 个字段来自同一次 Chassis_WheatWheel_Solution 调用，顺序为：
		 *   tick, seq, M1_t, M1_w, M2_t, M2_w, M3_t, M3_w, M4_t, M4_w, pre_power
		 * 整份拷贝放在临界区内，避免读到"半新半旧"的组合。
		 * seq 不变 = 控制没在跑（挡位不在有效分支/遥控掉线），此时是陈旧值。 */
//		MotorSnapshot_t snap_local;
//		taskENTER_CRITICAL();
//		snap_local = g_motor_snap;
//		taskEXIT_CRITICAL();
//			printf("%f,%f,%f\r\n",rotion_jisuan.yaw_unwarp,rotion_jisuan.DM4310_angle,rotion_jisuan.world2chassis_angle);
//		printf("%.3f,%.3f,%lu\r\n",
//		       (double)snap_local.pre_chassis_power,
//		       (double)Chassis_Power_Modul.act_chassis_power,
//		       (unsigned long)g_act_power_rx_tick);
//printf("%.3f,%.3f\r\n",(double)snap_local.pre_chassis_power,Chassis_Power_Modul.wheel.pre_chassis_power);
//   printf("%d,%d,%d,%d\r\n",auto_nav.dataint16_t.linear_x,auto_nav.dataint16_t.linear_y,auto_nav.dataint16_t.rotate_z,auto_nav.dataint16_t.chassis_mode);
	}           
	
}
