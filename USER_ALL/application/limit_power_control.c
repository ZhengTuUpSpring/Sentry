#include "limit_power_control.h"

#define Power_Factor 0.75f
Chassis_Power_Modul_t Chassis_Power_Modul = {UNINIT};

/**
 * @brief 获取轮毂电机转速
 * @description 从M3508电机数据计算轮毂实际转速（rad/s）
 * @param[in] steer 转向模块控制句柄指针
 * @return float 轮毂电机转速（rad/s）
 * @note 使用速度常数将电机RPM转换为角速度
 * @usage 在功率计算中获取当前电机转速
 * @example
 *   float rpm = Get_Wheel_Rpm(&steer_handle);
 */

static inline float Get_Wheel_Rpm(M3508_HandleTypeDef *motor)
{
	return motor->basic_data.speed_rpm * Velocity_Constant;
}

/**
 * @brief 获取轮毂电机转矩
 * @description 从M3508电流数据计算轮毂实际转矩（N·m）
 * @param[in] steer 转向模块控制句柄指针
 * @return float 轮毂电机转矩（N·m）
 * @note 使用转矩系数将电流值转换为转矩值
 * @usage 在功率计算中获取当前轮毂转矩
 * @example
 *   float torque = Get_Wheel_Torque(&steer_handle);
 */
static inline float Get_Wheel_Torque(M3508_HandleTypeDef *motor)
{
	return motor->basic_data.given_current * Wheel_Kr;
}

/**
 * @brief 获取轮毂电机预测转矩
 * @description 从预测转向数据结构计算轮毂预测转矩
 * @param[in] steer 预测转向数据结构指针
 * @return float 轮毂预测转矩（N·m）
 * @note 直接使用预测转矩系数进行计算
 * @usage 在功率预测中获取未来轮毂转矩
 * @example
 *   float forecast_torque = Get_Wheel_Torque_Forecast(&forecast_steer);
 */
static inline float Get_Wheel_Torque_Forecast(float current)
{
	return current * Wheel_T_Coeff;
}

/**
 * @brief 获取轮毂功率二次方程系数A
 * @description 计算轮毂功率二次方程标准形式中的系数A
 * @return float 二次项系数A
 * @note 对应公式 A = Wheel_K1 * (Wheel_T_Coeff)^2
 * @usage 在求解功率方程根时使用
 * @example
 *   float A = Get_Wheel_A();
 */
static inline float Get_Wheel_A()
{
	return Wheel_K1 * Wheel_T_Coeff * Wheel_T_Coeff;
}

/**
 * @brief 获取轮毂功率二次方程系数B
 * @description 计算轮毂功率二次方程标准形式中的系数B
 * @param[in] steer 转向模块控制句柄指针
 * @return float 一次项系数B
 * @note 对应公式 B = Wheel_K3 * 转速 * Wheel_T_Coeff
 * @usage 在求解功率方程根时使用
 * @example
 *   float B = Get_Wheel_B(&steer_handle);
 */
static inline float Get_Wheel_B(M3508_HandleTypeDef *motor)
{
	return Wheel_K3 * Get_Wheel_Rpm(motor) * Wheel_T_Coeff;
}

/**
 * @brief 获取轮毂功率二次方程系数C
 * @description 计算轮毂功率二次方程标准形式中的系数C
 * @param[in] steer 转向模块控制句柄指针
 * @param[in] power 功率数据结构指针
 * @return float 常数项系数C
 * @note 对应公式 C = Wheel_K2 * |转速| + 静态功耗 - 轮毂功率
 * @usage 在求解功率方程根时使用
 * @example
 *   float C = Get_Wheel_C(&steer_handle, &power_data);
 */
static inline float Get_Wheel_C(M3508_HandleTypeDef *motor,float power)
{
	float wheel_w = 0;
	wheel_w = Get_Wheel_Rpm(motor);
	float wheel_abs_w;
	arm_abs_f32(&wheel_w,&wheel_abs_w,1);
	return Wheel_K2 * wheel_abs_w + Static_Power - power;
}

/**
 * @brief 计算轮毂功率二次方程判别式
 * @description 计算轮毂功率二次方程判别式（Δ）
 * @param[in] steer 转向模块控制句柄指针
 * @param[in] power 功率数据结构指针
 * @return float 判别式值
 * @note Δ = B^2 - 4AC
 * @usage 判断功率方程解的情况（实数根/复数根）
 * @example
 *   float delta = Get_Wheel_Delta(&steer_handle, &power_data);
 */
static inline float Get_Wheel_Delta(M3508_HandleTypeDef *motor,float power)
{
	return (Get_Wheel_B(motor) * Get_Wheel_B(motor)) - (4.0f * Get_Wheel_A() * Get_Wheel_C(motor,power));
}

/**
 * @brief 浮点数近似相等判断
 * @description 判断两个浮点数是否近似相等（容差1e-6）
 * @param[in] a 第一个浮点数
 * @param[in] b 第二个浮点数
 * @return bool 相等返回true，否则返回false
 * @note 使用CMSIS-DSP库的arm_abs_f32函数计算绝对值
 * @usage 在判断判别式是否为零时使用
 * @example
 *   bool equal = Float_Equal(a, b);
 */
static inline bool Float_Equal(float a,float b)
{
	float diff = a - b;
	float abs;
	arm_abs_f32(&diff,&abs,1);
	bool state = abs < 1e-6f ? true : false;
	return state;
}

/**
 * @brief 底盘功率模块初始化函数
 * @description 初始化底盘功率控制模块，设置函数指针和初始值
 * @param[inout] modul 底盘功率模块数据结构指针
 * @note 
 *   - 初始化实际底盘功率和预测功率为零
 *   - 绑定各功能函数指针
 *   - 发送超级电容初始化命令
 * @usage 在系统启动时调用，初始化功率控制模块
 * @example
 *   Chassis_Power_Modul_t modul;
 *   Chassis_Power_Modul_Init(&modul);
 */
void Chassis_Power_Modul_Init(Chassis_Power_Modul_t *modul)
{
	  modul->act_chassis_power = modul->wheel.pre_chassis_power= 0;
		modul->supercap_power = 0;
		modul->wheel.get_rpm_feedback = Get_Wheel_Rpm;
		modul->wheel.get_tor_feedback = Get_Wheel_Torque;
		modul->wheel.get_tor_forecast = Get_Wheel_Torque_Forecast;
	  modul->wheel.get_a			  		= Get_Wheel_A;
	  modul->wheel.get_b			  		= Get_Wheel_B;
	  modul->wheel.get_c			  		= Get_Wheel_C;
	  modul->wheel.get_delta		  	= Get_Wheel_Delta;
		modul->float_equal			  		= Float_Equal;
		
		modul->sup_state = UNINIT;
}

bool SuperCap_Modify(Chassis_Power_Modul_t *modul,SupCap_State_Handle_t state)
{
	if(modul->sup_state == state)
	{
		return true;
	}
	uint8_t txdata = state;
	if(CAN_cmd_supcap_init(&txdata) == HAL_OK) return true;
	return false;
}

HAL_StatusTypeDef CAN_cmd_supcap_control(float power)
{
    CAN_TxHeaderTypeDef  chassis_tx_message;
    uint32_t send_mail_box;
    chassis_tx_message.StdId = SUPCAP_CONTROL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x04;
		uint8_t buff[4];
		memcpy(buff,&power,sizeof(float));
    return HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, buff, &send_mail_box);
}

HAL_StatusTypeDef CAN_cmd_supcap_init(uint8_t *txdata)
{
    CAN_TxHeaderTypeDef  chassis_tx_message;
    uint32_t send_mail_box;
    chassis_tx_message.StdId = SUPCAP_INIT_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x01;

    return HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, txdata, &send_mail_box);
}

/**
 * @brief 预测功率计算函数
 * @description 根据当前电机状态和预测转矩计算预测功率
 * @param[in] modul 底盘功率模块数据结构指针
 * @param[in] steer 转向模块控制句柄指针
 * @param[in] fs 预测转向数据结构指针
 * @return Power_t 计算得到的预测功率（轮毂和偏航）
 * @note 
 *   - 轮毂功率公式：P_wheel = K1*t2 + K2*|w| + K3*t*w + 静态功耗
 *   - 偏航功率公式：P_yaw = K1*t2 + K2*|w| + K3*t*w + 静态功耗
 * @usage 在功率控制循环中计算预测功率
 * @example
 *   Power_t forecast = Forecast_Power_Cal(&modul, &steer, &fs);
 */
float Forecast_Power_Cal(Chassis_Power_Modul_t *modul,M3508_HandleTypeDef *motor,int16_t forecastcurrent)
{
	 float wheel_power = 0.0f;
   
	 float wheel_t = modul->wheel.get_tor_forecast(forecastcurrent);
	 float wheel_w = modul->wheel.get_rpm_feedback(motor);
	 float abs_wheel_w;
	 arm_abs_f32(&wheel_w,&abs_wheel_w,1);
	 
	 wheel_power = Wheel_K1 * wheel_t * wheel_t+
	 							 Wheel_K2 * abs_wheel_w + 
	 							 Wheel_K3 * wheel_t * wheel_w + Static_Power;
	
	 return wheel_power;
}

/**
 * @brief 功率控制主函数
 * @description 根据当前功率限制，调整预测转矩使总功率不超过限制
 * @param[inout] modul 底盘功率模块数据结构指针
 * @param[in] steer 转向模块控制句柄指针数组（4个轮子）
 * @param[inout] fs 预测转向数据结构数组（将被调整）
 * @param[in] power 原始功率数据数组
 * @return bool 功率控制是否成功执行
 * @note 
 *   - 功率分配策略：优先保证偏航轴功率（占比Power_Factor）
 *   - 使用二次方程求解方法计算最大允许转矩
 *   - 处理判别式为负数或接近零的特殊情况
 * @usage 在主控制循环中调用，进行功率限制
 * @example
 *   bool success = Power_Control(&modul, steer_array, fs_array, power_array);
 */
bool Power_Control(Chassis_Power_Modul_t *modul,M3508_HandleTypeDef *wheel,float *fs,float *power)
{
	float allo_power[4];
	
	float k = (Max_Power - modul->supercap_power -10.0f) / modul->wheel.pre_chassis_power;

	 if(k > 1)
	 {
		 return false;
	 }else{
	   for(uint8_t i = 0;i < 4;i++)
	   {
	      allo_power[i] = power[i] * k;
		    float wheel_a,wheel_b,wheel_delta,wheel_delta_sqrt;
		    wheel_a = wheel_b  = wheel_delta = wheel_delta_sqrt = 0.0f;

		    wheel_a = modul->wheel.get_a();
	      wheel_b = modul->wheel.get_b(&wheel[i]);
		    wheel_delta = modul->wheel.get_delta(&wheel[i],allo_power[i]);		     

			float orig_wheel = fs[i]; /* 保留输入预测值，作为选择根的基准 */
			if(modul->float_equal(wheel_delta,0.0f) || wheel_delta <= 1e-12f)
			{
				fs[i] = (-wheel_b / (2.0f * wheel_a));
			}
			else
			{
				/* 避免 tiny negative 导致 sqrtf NaN -> 对小负数裁剪为0 */
				if(wheel_delta < 0.0f && wheel_delta > -1e-6f) wheel_delta = 0.0f;
				if(wheel_delta < 0.0f)
				{
					fs[i] = -wheel_b / (2.0f * wheel_a);
				}
				else
				{
					arm_sqrt_f32(wheel_delta,&wheel_delta_sqrt);
					float r1 = (-wheel_b + wheel_delta_sqrt) / (2.0f * wheel_a);
					float r2 = (-wheel_b - wheel_delta_sqrt) / (2.0f * wheel_a);
					/* 选择与原始预测值差值最小的根，避免符号抖动 */
					float d1 = r1 - orig_wheel; d1 = d1 < 0.0f ? -d1 : d1;
					float d2 = r2 - orig_wheel; d2 = d2 < 0.0f ? -d2 : d2;
					fs[i] = (d1 <= d2) ? r1 : r2;
				}
			}
			/* 防护：当系数 a 极小时退回等式解的稳妥值 */
			if(fabsf(wheel_a) < 1e-9f) fs[i] = 0.0f;
	   }
	 }
	return true;
}  
