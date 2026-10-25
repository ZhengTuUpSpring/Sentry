#include "stdio.h"
#include "3508_caontrol.h"
#include "arm_math.h"
#include "stdbool.h"

#define SUPCAP_INIT_ID 0x301 
#define SUPCAP_CONTROL_ID 0x300 
#define SUPCAP_RECV_ID 0x2FF 

//#define SUPERCAPACITOR 0x2FF

/**
 * @defgroup 功率模型参数组
 * @brief 功率计算模型的关键参数定义
 * @{
 */

//#define Static_Power 0.219f        /**< 静态功率消耗（W） */

//#define Wheel_K1 530.9f            /**< 轮毂转矩平方系数 */
//#define Wheel_K2 0.009f           /**< 轮毂绝对速度系数 */
//#define Wheel_K3 0.933f            /**< 轮毂转矩速度交叉项系数 */
//#define Wheel_K1  303.0f
//#define Wheel_K2  0.032f
//#define Wheel_K3  1.22f
//#define Static_Power 0.55f
#define Wheel_K1    331.0f   // 原 530.9 偏大，三批数据均指向 300~330
#define Wheel_K2    0.031f   // 原 0.009 偏小
#define Wheel_K3    1.24f    // 三批数据锁定 1.22~1.24
#define Static_Power 0.60f   // 静止段标定 0.55~0.60


/**
 * @defgroup 转矩转换系数组
 * @brief 电流到转矩的转换系数
 * @{
 */
#define Wheel_Kr   0.0156223893065998f         //经过减速箱后、在轮端输出的转矩常数 0.3/(3591/187)
/** @} */

/**
 * @defgroup 最终转矩系数组
 * @brief 考虑CAN数据缩放后的转矩系数
 * @{
 */
 //CAN 总线上电流值每增加 1，轮端输出转矩增加约 0.000019 N·m
#define Wheel_T_Coeff   ((Wheel_Kr) * (20.0f/16384.0f))     /**< 偏航轴最终转矩系数 */ //(20.0f/16384.0f) 就是把这个 CAN 原始数据转成真实安培值的系数

/**< 速度转换常数（RPM到rad/s） */
#define Velocity_Constant 0.1047120418848168f 

#define Max_Power 60.0f           /**< 底盘最大允许功率（W） */\

typedef enum
{
	UNINIT = 0,
	POWER_METER  = 1,
	POWER_CONTROL = 2
}SupCap_State_Handle_t;

/**
 * @brief 功率模块数据结构
 * @description 封装单个电机轴（轮毂或偏航）的功率计算功能
 * @note 使用函数指针实现策略模式，便于模块化扩展
 * @usage 作为Chassis_Power_Modul_t的子结构
 */
typedef struct
{
		float pre_chassis_power;                /**< 上一周期底盘功率（W） */
		float (*get_rpm_feedback)(M3508_HandleTypeDef *motor);      /**< 获取当前转速函数指针 */
	  float (*get_tor_feedback)(M3508_HandleTypeDef *motor);      /**< 获取当前转矩函数指针 */
		float (*get_tor_forecast)(float current);       /**< 获取预测转矩函数指针 */
		float (*get_a)();                                        /**< 获取二次方程系数A函数指针 */
	  float (*get_b)(M3508_HandleTypeDef *motor);                  /**< 获取二次方程系数B函数指针 */
	  float (*get_c)(M3508_HandleTypeDef *motor,float power);   /**< 获取二次方程系数C函数指针 */
	  float (*get_delta)(M3508_HandleTypeDef *motor,float power); /**< 获取判别式Δ函数指针 */
}Power_Modul_t;

/**
 * @brief 底盘功率模块主数据结构
 * @description 管理整个底盘功率控制系统
 * @usage 作为功率控制模块的主控制结构
 * @example
 *   Chassis_Power_Modul_t modul;
 *   Chassis_Power_Modul_Init(&modul);
 */
typedef struct 
{
		SupCap_State_Handle_t sup_state;
		float act_chassis_power;          /**< 实际底盘总功率（W） */
		float supercap_power;             /**< 超级电容输出功率（W） */
		Power_Modul_t wheel;              /**< 轮毂功率模块 */
		bool (*float_equal)(float a,float b); /**< 浮点数近似相等判断函数指针 */
}Chassis_Power_Modul_t;

bool SuperCap_Modify(Chassis_Power_Modul_t *modul,SupCap_State_Handle_t state);
HAL_StatusTypeDef CAN_cmd_supcap_init(uint8_t *txdata);
/**
 * @brief 底盘功率模块初始化函数
 * @description 初始化底盘功率控制模块的所有参数和函数指针
 * @param[inout] modul 底盘功率模块数据结构指针
 * @note 必须在系统启动时调用一次
 * @usage 
 *   - 系统初始化阶段调用
 *   - 初始化所有函数指针和参数
 * @example
 *   Chassis_Power_Modul_t modul;
 *   Chassis_Power_Modul_Init(&modul);
 */
void Chassis_Power_Modul_Init(Chassis_Power_Modul_t *modul);
extern Chassis_Power_Modul_t Chassis_Power_Modul;
/**
 * @brief 预测功率计算函数
 * @description 根据当前状态和预测转矩计算未来功率需求
 * @param[in] modul 底盘功率模块数据结构指针
 * @param[in] steer 转向模块控制句柄指针
 * @param[in] fs 预测转向数据结构指针
 * @return Power_t 计算得到的预测功率
 * @note 使用二次功率模型计算预测功率
 * @usage 在功率控制循环中计算功率需求
 * @example
 *   Power_t forecast = Forecast_Power_Cal(&modul, &steer, &fs);
 */
float Forecast_Power_Cal(Chassis_Power_Modul_t *modul,M3508_HandleTypeDef *motor,int16_t forecastcurrent);
/**
 * @brief 功率控制主函数
 * @description 根据功率限制调整预测转矩，确保总功率不超限
 * @param[inout] modul 底盘功率模块数据结构指针
 * @param[in] steer 转向模块控制句柄指针数组（4个轮子）
 * @param[inout] fs 预测转向数据结构数组（将被调整）
 * @param[in] power 原始功率数据数组
 * @return bool 功率控制是否成功执行
 * @note 
 *   - 根据功率分配策略（Power_Factor）调整转矩
 *   - 使用二次方程求解最优转矩值
 *   - 处理判别式异常情况
 * @usage 在主控制循环中调用以限制功率
 * @example
 *   bool success = Power_Control(&modul, steer_array, fs_array, power_array);
 */
bool Power_Control(Chassis_Power_Modul_t *modul,M3508_HandleTypeDef *wheel,float *fs,float *power);

HAL_StatusTypeDef CAN_cmd_supcap_control(float power);

