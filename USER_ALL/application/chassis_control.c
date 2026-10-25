#include "main.h"
#include "chassis_control.h"
#include "3508_caontrol.h"
#include "math.h"
#include "stdlib.h"
#include "pid.h"
#include "stdlib.h"
#include "arm_math.h"
#include "4310_control.h"
#include "receive_task.h"
#include "BMI088.h"
#include "limit_power_control.h"
#include "RC_control.h"
#define DEADLINE 100
#define RADIAN_COEF_CHASSIS 0.017453292519943295f
#define CHASSIS_SQRT2_HALF 0.70710678f
#define YAW_VEL_SLEW_LIMIT  3.0f

/*define chassis motor handle*/
M3508_HandleTypeDef wheelJoint_motorHandle[4];

pid_type_def chassis_angle_pid;
chassis_rotation rotion_jisuan;

/**
  * @func			void Chassis_Init(void)
  * @brief          底盘驱动初始化
  * @param[in]      none
  * @retval         none
  */
Chassis_CoordinateSystem_Typedef chassis_vectors;
void Chassis_Init(void)
{ 
	/*底盘电机初始化*/
	#if (CHASSiS_SPEED_PID_MODE == 0)
	M3508_Init(0x01, &wheelJoint_motorHandle[0], PID_DELTA, 1);
	M3508_Init(0x02, &wheelJoint_motorHandle[1], PID_DELTA, 1);
	M3508_Init(0x03, &wheelJoint_motorHandle[2], PID_DELTA, 1);
	M3508_Init(0x04, &wheelJoint_motorHandle[3], PID_DELTA, 1);
	#else
	M3508_Init(0x01, &wheelJoint_motorHandle[0], PID_DELTA);
	M3508_Init(0x02, &wheelJoint_motorHandle[1], PID_DELTA);
	M3508_Init(0x03, &wheelJoint_motorHandle[2], PID_DELTA);
	M3508_Init(0x04, &wheelJoint_motorHandle[3], PID_DELTA);
	#endif
	/*底盘坐标系初始化*/
	chassis_vectors.c_x=0;
	chassis_vectors.c_y=0;
	chassis_vectors.c_rotate=0;
	chassis_vectors.cos=0.0f;
	chassis_vectors.sin=0.0f;
	
	/*底盘相关pid初始化*/
	fp32 pid_angle[3]={
	 CHASSIS_ANGLEPID_KP,
	 CHASSIS_ANGLEPID_KI,
   CHASSIS_ANGLEPID_KD
	};

	PID_init(&chassis_angle_pid,PID_POSITION,pid_angle, 5000, 5000);
	 
	YAW_4310motor_init();
	
}

/**
  * @brief          全向轮底盘运动学解算
  * @param[in]      chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]      chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]      chassisRotatevector:底盘(世界)坐标系下，底盘旋转的速度
  * @retval         none
  */
float avaible_power;
void Chassis_WheatWheel_Solution(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector)
{
	/*获取底盘电机基础数据*/
	M3508_GetBasicData(&wheelJoint_motorHandle[0], 1);
	M3508_GetBasicData(&wheelJoint_motorHandle[1], 1);
	M3508_GetBasicData(&wheelJoint_motorHandle[2], 1);
	M3508_GetBasicData(&wheelJoint_motorHandle[3], 1);

	/* 全向轮解算 */
	float targetspeedMA=chassisXvector*CHASSIS_SQRT2_HALF+chassisYvectory*CHASSIS_SQRT2_HALF-chassisRotatevector;
	float targetspeedMB=chassisXvector*CHASSIS_SQRT2_HALF-chassisYvectory*CHASSIS_SQRT2_HALF-chassisRotatevector;
	float targetspeedMC=-chassisXvector*CHASSIS_SQRT2_HALF-chassisYvectory*CHASSIS_SQRT2_HALF-chassisRotatevector;
	float targetspeedMD=-chassisXvector*CHASSIS_SQRT2_HALF+chassisYvectory*CHASSIS_SQRT2_HALF-chassisRotatevector;
	
	/* pid运算 */
	float set_current[4];
	set_current[0]=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[0].speed_control_data), wheelJoint_motorHandle[0].basic_data.speed_rpm, targetspeedMA);
	set_current[1]=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[1].speed_control_data), wheelJoint_motorHandle[1].basic_data.speed_rpm, targetspeedMB);
	set_current[2]=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[2].speed_control_data), wheelJoint_motorHandle[2].basic_data.speed_rpm, targetspeedMC);
	set_current[3]=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[3].speed_control_data), wheelJoint_motorHandle[3].basic_data.speed_rpm, targetspeedMD);
	
	float power[4];
	float total_power = 0.0f;
	
	power[0] = Forecast_Power_Cal(&Chassis_Power_Modul,&wheelJoint_motorHandle[0],set_current[0]);
	power[1] = Forecast_Power_Cal(&Chassis_Power_Modul,&wheelJoint_motorHandle[1],set_current[1]);
	power[2] = Forecast_Power_Cal(&Chassis_Power_Modul,&wheelJoint_motorHandle[2],set_current[2]);
	power[3] = Forecast_Power_Cal(&Chassis_Power_Modul,&wheelJoint_motorHandle[3],set_current[3]);
	
	for(uint8_t i = 0;i<4;i++)
	{
		total_power += power[i];
	}
	
	Chassis_Power_Modul.wheel.pre_chassis_power = total_power;
	
	
//	if(Chassis_Power_Modul.wheel.pre_chassis_power >= Max_Power - Chassis_Power_Modul.supercap_power - 10.0f )
//	{
//		Power_Control(&Chassis_Power_Modul,&wheelJoint_motorHandle[0],&set_current[0],&power[0]);
//	}
//	
//	avaible_power = Max_Power - total_power;
//	CAN_cmd_supcap_control(avaible_power);

	/* ===== 同拍调试快照 =====
	 * 把本拍的 4 轮数据与预测功率一次性封存，供 MotorDebug_PrintTask 打印。
	 * 数据源用本拍开头 M3508_GetBasicData() 刚拷进 wheelJoint_motorHandle 的值，
	 * 因此 9 个字段严格出自同一次 Chassis_WheatWheel_Solution 调用。
	 * 临界区约 30 条指令 (~0.2us)，CAN 帧间隔 111us，不影响接收。 */
	taskENTER_CRITICAL();
	for(uint8_t dbg_i = 0; dbg_i < 4; dbg_i++)
	{
		g_motor_snap.wheel_t[dbg_i] = (float)wheelJoint_motorHandle[dbg_i].basic_data.given_current * Wheel_T_Coeff;
		g_motor_snap.wheel_w[dbg_i] = (float)wheelJoint_motorHandle[dbg_i].basic_data.speed_rpm * Velocity_Constant;
	}
	g_motor_snap.pre_chassis_power = total_power;
	
	g_motor_snap.tick = xTaskGetTickCount();
	g_motor_snap.seq++;
	taskEXIT_CRITICAL();

	CAN_cmd_chassis((int16_t )set_current[0], (int16_t )set_current[1], (int16_t )set_current[2], (int16_t )set_current[3]);
}
/**
  * @brief          底盘小陀螺任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      worldZrotate：小陀螺转速
  * @param[in]      c_angle：世界/云台坐标系与底盘坐标系的夹角 //读取宇树go数据
  * @retval         none
  */
void ChassisGyro_Task(int16_t worldXvector,int16_t worldYvector,int16_t worldZrotate,float c_angle_deg)
{
	/* c_angle_deg: 世界/云台坐标系与底盘坐标系的夹角，单位：degree */
	chassis_vectors.angle_cmtw = c_angle_deg * RADIAN_COEF_CHASSIS;
	/* cos,sin计算：arm_sin_cos_f32输入单位为degree */
	arm_sin_cos_f32(c_angle_deg, &chassis_vectors.sin, &chassis_vectors.cos);
	/*将世界坐标系下的目标速度映射到底盘坐标系*/
	chassis_vectors.c_x = -worldXvector*chassis_vectors.cos - worldYvector*chassis_vectors.sin;
	chassis_vectors.c_y = worldYvector*chassis_vectors.cos - worldXvector*chassis_vectors.sin;
	//扔给底盘结算函数
	Chassis_WheatWheel_Solution(chassis_vectors.c_x, chassis_vectors.c_y, worldZrotate);
}

 float Limit_Float(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}
float Normalize_Angle360(float angle)
{
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }

    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    return angle;
}

 float Normalize_Angle180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}
//底盘IMU版本
 static float YAWIMUtarget = 0.0f;  
 static int16_t initY_flag=0;
void chassis_holdYAW_UNGO(int16_t chassis_selfZ_vel, float x_speed, float y_speed,float rc_yaw_vel)
{	
	    if (!initY_flag) {
        YAWIMUtarget = YAWImuReceiveData.yaw;
        initY_flag = 1;
    }
		YAWIMUtarget = Normalize_Angle360(YAWIMUtarget);
		
	float get_chassis_speed = BMI088.Gyro[2];
		
		static float f_speed=0;	

    f_speed = DM_PositionLoopPIDController(
        &yaw_pos_pid,
        YAWImuReceiveData.yaw,
        YAWIMUtarget
    );
	 float YAW_VEL=0;
		
	 YAW_VEL=get_chassis_speed+f_speed+rc_yaw_vel;
		
	  if (YAW_VEL >  YAW_VELMAX)  YAW_VEL =  YAW_VELMAX;
    if (YAW_VEL < -YAW_VELMAX)  YAW_VEL = -YAW_VELMAX;	
		
	  yaw_target_speed=YAW_VEL;	
		float tor= friction_compensation_calc_torque(&YAW_fric, YAW_VEL);
		
	 mit_ctrl(&hcan1,&DM_4310,DM_4310.id,0.0f,YAW_VEL,0.0f,YAW_IMUKD,tor);
	
    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
		
    float c_angle_deg = yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel;

  ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_selfZ_vel, c_angle_deg);
}
void sin_control(float x_speed,float y_speed,float rc_yaw)
{
	  static float phase = 0.0f;
    static TickType_t last_tick = 0;

    /* 计算真实时间步长 */
    TickType_t now = xTaskGetTickCount();
    float dt = (last_tick == 0) ? 0.001f
              : (float)(now - last_tick) / (float)configTICK_RATE_HZ;
    if (dt > 0.1f) dt = 0.1f;
    last_tick = now;

    /* DDS 相位累加 */
    const float freq = 0.08f;
    phase += 2.0f * PI * freq * dt;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;

    /* 正弦变速自旋 */
    const float amplitude = 2000.0f;
    float sin_value =1000+ amplitude * fabsf(arm_sin_f32(phase));
		
		gimbal_world_lock(x_speed,y_speed,rc_yaw,sin_value);
}
static float YAW_IMUtarget = 0.0f;  
static int16_t init_Y_flag=0;
float yaw_sin_target_speed=0.0f;
float yaw_sin_current_speed=0.0f;
float yaw_getpid_speed=0.0f;
float tor_YAW=0.0f;

void chassis_GO_with_sin_rotation(float x_speed, float y_speed, float rc_yaw)
{
    if (!init_Y_flag) {
        YAW_IMUtarget = YAWImuReceiveData.yaw;
        init_Y_flag = 1;
    }

    static float phase = 0.0f;
    static TickType_t last_tick = 0;

    /* 计算真实时间步长 */
    TickType_t now = xTaskGetTickCount();
    float dt = (last_tick == 0) ? 0.001f
              : (float)(now - last_tick) / (float)configTICK_RATE_HZ;
    if (dt > 0.1f) dt = 0.1f;
    last_tick = now;

    /* DDS 相位累加 */
    const float freq = 0.08f;
    phase += 2.0f * PI * freq * dt;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;

    /* 正弦变速自旋 */
    const float amplitude = 2000.0f;
    float sin_value =1000+ amplitude * fabsf(arm_sin_f32(phase));

    /* 位置环：输出速度修正量（级联外环） */
    float pos_speed_cmd = DM_PositionLoopPIDController(
        &yaw_pos_pid,
        YAWImuReceiveData.yaw,
        YAW_IMUtarget
    );

		float sin_value_rad=wheel_to_rad(sin_value);

    /* 速度目标 = 正弦目标速度 + 位置环修正 + 遥控 */
    float speed_target = sin_value_rad  + rc_yaw;		
		
    if (speed_target >  YAW_VELMAX) speed_target =  YAW_VELMAX;
    if (speed_target < -YAW_VELMAX) speed_target = -YAW_VELMAX;
		
		yaw_sin_target_speed=speed_target;
    /* 速度环：追合并后的速度目标（级联内环，微分先行 + 抗饱和） */
    float get_chassis_speed = BMI088.Gyro[2];
		
		yaw_sin_current_speed=get_chassis_speed;
    float YAW_VEL = PID_Calc(&YAW_IMU_SPEED_PID, speed_target, get_chassis_speed, dt);

		yaw_getpid_speed=YAW_VEL;
		
		if (YAW_VEL >  YAW_VELMAX) YAW_VEL =  YAW_VELMAX;
    if (YAW_VEL < -YAW_VELMAX) YAW_VEL = -YAW_VELMAX;

    float tor = friction_compensation_calc_torque(&YAW_fric, YAW_VEL);
		
		tor_YAW=tor;
				
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, YAW_VEL, 0.0f, YAW_IMUKD, tor);

    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
		
    float c_angle_deg = yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel;

   ChassisGyro_Task(-x_speed * 6, y_speed * 6, sin_value, c_angle_deg);
}

#define RAD2DEG            57.29578f
#define DM4310_POS_RANGE   6.28318531f   /* 2π，pos 回绕周期（输出轴一圈） */
#define DM4310_POS_HALF    3.14159265f   /* π，周期的一半，用于回绕判据 */

/* =================================================================== */
/*  云台传动比: 上板 IMU(yaw) 与大 yaw 编码器(gimbal_rel) 之间的减速比     */
/*                                                                     */
/*  实测(转5圈, 干净数据): yaw=-1803.26°, gimbal_rel=-471.84°           */
/*        → RATIO = 1803.26/471.84 ≈ 3.822                             */
/*                                                                     */
/*  公式: c_angle = yaw - RATIO * gimbal_rel                            */
/*        把「慢轴」gimbal_rel 放大 RATIO 倍, 对齐「快轴」yaw, 再相减。     */
/*                                                                     */
/*  符号: 顺时针转, yaw 与 gimbal_rel 同号(都变负) → RATIO=+3.822        */
/* =================================================================== */
#define GIMBAL_GEAR_RATIO   3.822f

/**
 * @brief  把达妙 DM4310 的 pos（弧度，±π 回绕）展开成连续角度（度）
 * @param  raw_pos   DM_4310.para.pos 原始值（弧度）
 * @return 连续的云台相对底盘角度（度）
 */
float DM4310_Pos_Unwrap_Deg(float raw_pos)
{
    static float pos_unwrapped = 0.0f;
    static float pos_last      = 0.0f;
    static uint8_t pos_first   = 1;

    if (pos_first) {
        pos_unwrapped = raw_pos;
        pos_last      = raw_pos;
        pos_first     = 0;
    } else {
        float delta = raw_pos - pos_last;

        /* 回绕检测：变化量超过半周期，说明跨过了 ±边界 */
        if (delta >  DM4310_POS_HALF) delta -= DM4310_POS_RANGE;
        if (delta < -DM4310_POS_HALF) delta += DM4310_POS_RANGE;

        pos_unwrapped += delta;
        pos_last       = raw_pos;
    }
    return -pos_unwrapped * RAD2DEG;
}

/**
 * @brief  把上板 IMU 的 yaw（度，-180~+180 会回绕）展开成连续角度（度）
 * @param  raw_yaw   YAWImuReceiveData.yaw 原始值（度）
 * @return 连续无跳变的云台世界朝向（度），可超过 ±180
 * @note   上板 yaw 在 ±180 边界从 +180 跳到 -180（顺时针递增方向），
 *         若不展开，`yaw - world_direction` 会在边界爆 ±360 假跳变。
 *         与 DM4310_Pos_Unwrap_Deg 同思路，但阈值用 180° 半周期。
 */
float YawImu_Unwrap_Deg(float raw_yaw)
{
    static float yaw_unwrapped = 0.0f;
    static float yaw_last      = 0.0f;
    static uint8_t yaw_first   = 1;

    if (yaw_first) {
        yaw_unwrapped = raw_yaw;
        yaw_last      = raw_yaw;
        yaw_first     = 0;
    } else {
        float delta = raw_yaw - yaw_last;

        /* 回绕检测：相邻两次变化超过 180°，说明跨过 ±180 边界 */
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;

        yaw_unwrapped += delta;
        yaw_last       = raw_yaw;
    }
    return yaw_unwrapped;
}
#define CHASSIS_R        0.231f
#define WHEEL_DIAMETER   0.152f
#define GEAR_RATIO       19.20320856f
/**
 * @brief  4个电机端 RPM → 底盘角速度
 * @param  rpm_LF, rpm_RF, rpm_RR, rpm_LR  4个电机的 RPM（电机端）
 * @return 底盘角速度（rad/s）
 */
float OmniWheel_GetOmega(void)
{
	  M3508_GetBasicData(&wheelJoint_motorHandle[0], 1);
    M3508_GetBasicData(&wheelJoint_motorHandle[1], 1);
    M3508_GetBasicData(&wheelJoint_motorHandle[2], 1);
    M3508_GetBasicData(&wheelJoint_motorHandle[3], 1);

    float wheel_a = CHASSIS_WHEEL_SIGN_A * 
              (float)wheelJoint_motorHandle[0].basic_data.speed_rpm;
    float wheel_b = CHASSIS_WHEEL_SIGN_B * 
              (float)wheelJoint_motorHandle[1].basic_data.speed_rpm;
    float wheel_c = CHASSIS_WHEEL_SIGN_C * 
              (float)wheelJoint_motorHandle[2].basic_data.speed_rpm;
    float wheel_d = CHASSIS_WHEEL_SIGN_D * 
              (float)wheelJoint_motorHandle[3].basic_data.speed_rpm;
    /* 1. 电机端 RPM → 轮子端 RPM */
    float rpm_wheelA = wheel_a / GEAR_RATIO;
    float rpm_wheelB = wheel_b / GEAR_RATIO;
    float rpm_wheelC = wheel_c / GEAR_RATIO;
    float rpm_wheelD = wheel_d / GEAR_RATIO;

    /* 2. 轮子端 RPM → 轮子线速度 (m/s) */
    float k = PI * WHEEL_DIAMETER / 60.0f;  
    float v_LF = rpm_wheelA * k;
    float v_RF = rpm_wheelB * k;
    float v_RR = rpm_wheelC * k;
    float v_LR = rpm_wheelD * k;

    return (v_LF + v_RF + v_RR + v_LR) / (4.0f * CHASSIS_R);
}  
float wheel_to_rad(float vlaue)
{
	float rpm_lunzi= vlaue/GEAR_RATIO;
	float k = PI * WHEEL_DIAMETER / 60.0f;
	
	float w_rad=rpm_lunzi*k/CHASSIS_R;
	
	return w_rad;
}
float wheel_to_omega(float value)
{
    float wheel_rpm = value / GEAR_RATIO;
    float omega = (wheel_rpm / 60.0f)*2*PI;
    return omega;
}

static float yaw_target = 0.0f;   // 云台要锁定的世界方向
static uint8_t init_flag = 0;     // 上电初始化标志

/* 切换到非 holdYAW 模式时调用，下次切回时重新初始化 yaw_target */
void chassis_holdYAW_reset(void)
{
    init_flag = 0;
}

/**
 * @brief  模式切换时的 yaw 复位：发零命令停电机 + 清 PID + 清各模式锁存
 * @note   在 Chassis_Task 检测到 s1/s2 挡位变化时调用一次。
 *         旧 chassis_holdYAW_reset 只清 init_flag、不发命令，电机停不下来；
 *         本函数额外发 vel=0/tor=0 停止帧，并清 PID 与各模式锁存。
 */
void chassis_yaw_mode_reset(void)
{
    /* 1. 发零命令帧：vel=0、tor=0，保留 kd 阻尼刹停 */
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, 0.0f, 0.0f, YAW_KD, 0.0f);

    /* 2. 清位置环 PID 历史（积分/微分/误差），避免带上次残留跳变 */
    {
        fp32 yaw_pos_params[3] = {YAW_POS_KP, YAW_POS_KI, YAW_POS_KD};
        PID_init(&yaw_pos_pid, PID_POSITION, yaw_pos_params, 1000, 1000);
    }

    /* 3. 清速度环 PID 历史 */
    PID_Init(&YAW_IMU_SPEED_PID, YAW_IMU_KP, YAW_IMU_KI, YAW_IMU_KD, 1000.0f, 1000.0f);

    /* 4. 清各模式的锁存标志，下次进入重新锁存当前角度 */
    chassis_holdYAW_reset();
    gimbal_lock_reset();
    yaw_rc_hold(0, 0, YAW_HOLD_MODE_VEL);   /* enable=0 清 yaw_hold_latched */
}
float yaw_target_speed = 0.0f;
void chassis_holdYAW_GO(int16_t chassis_selfZ_vel, float x_speed, float y_speed,float rc_yaw_vel)
{

    if (!init_flag) {
        yaw_target = YAWImuReceiveData.yaw;
        init_flag = 1;
    }  
		yaw_target = Normalize_Angle360(yaw_target);
		float get_speed=-OmniWheel_GetOmega();
    
		static float f_speed=0 ;
		
    f_speed = DM_PositionLoopPIDController(
        &yaw_pos_pid,
        YAWImuReceiveData.yaw,
        yaw_target
    ); 
		
		float YAW_VEL = get_speed +f_speed+rc_yaw_vel;

    float cmd = friction_compensation_calc_torque(&YAW_fric, YAW_VEL);
		
		if (YAW_VEL >  YAW_VELMAX) YAW_VEL =  YAW_VELMAX;
    if (YAW_VEL < -YAW_VELMAX) YAW_VEL = -YAW_VELMAX;
		
		yaw_target_speed = YAW_VEL;
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, YAW_VEL, 0.0f, YAW_KD,0.0f);

    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
		
    float c_angle_deg = yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel;

  ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_selfZ_vel, c_angle_deg);
}

void chassis_yaw_control(float chassis_yaw)
{
    float vel_target =chassis_yaw;   
	
		float cmd = friction_compensation_calc_torque(&YAW_fric,vel_target );
	
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
             0.0f, vel_target, 0.0f, YAW_RC_KD,cmd);
}

/* =================================================================== */
/*    遥控增量式 Yaw 位置锁定（转到哪，锁在哪）                           */
/*                                                                     */
/*    摇杆 → 角速度 → 积分成目标角度 → MIT 位置环锁住                    */
/*    松杆后角速度为 0，目标停住，kp 持续把电机顶在该角度                */
/*                                                                     */
/*    target += rc_omega * dt                                          */
/* =================================================================== */

#define YAW_RC_HOLD_SPEED_GAIN   0.007f 
#define YAW_RC_HOLD_KP           10.0f     /* MIT 位置环 Kp (N·m/rad)，越大锁得越硬 */
#define YAW_RC_HOLD_KD           1.0f      /* MIT 位置环 Kd 阻尼，抑制到位振荡 */
#define YAW_RC_HOLD_DT           0.001f    /* 调用周期，与任务 1ms 循环保持一致 */

static float   yaw_rc_hold_target    = 0.0f;   /* 目标角度 (rad) */
static uint8_t yaw_rc_hold_latched   = 0;      /* 是否已成功锁存起点(电机使能后才锁) */

/**
 * @brief  遥控增量式 Yaw 位置锁定：摇杆转动、松杆锁住（转到哪锁在哪）
 * @param  rc_ch   遥控 yaw 通道原始值（12bit 有符号，内部做死区）
 * @param  enable  是否进入该模式（1=进入，0=退出）
 * @note   需以 1ms 周期调用；仅当电机已使能(state==0x01)时才锁存起点，
 *         避免上电初期位置反馈未就绪时把目标锁成 0 导致电机跳变
 */
void yaw_rc_pos_hold(int16_t rc_ch, uint8_t enable)
{
    if (enable) {
        /* 电机已使能、反馈已就绪才锁存起点，否则保持等待 */
        if (!yaw_rc_hold_latched && DM_4310.para.state == 0x01) {
            yaw_rc_hold_target = DM_4310.para.pos;   /* rad */
            yaw_rc_hold_latched = 1;
        }

        /* 锁存成功后才开始积分与发送，避免未就绪时误发命令 */
        if (yaw_rc_hold_latched) {
            /* 摇杆(死区) → 角速度 → 积分成目标角度 */
            float rc_omega = (float)apply_deadzone(rc_ch) * YAW_RC_HOLD_SPEED_GAIN;
            yaw_rc_hold_target += rc_omega * YAW_RC_HOLD_DT;

            /* 位置回绕：跨过 ±P_MAX 边界时搬移一个周期(2π) */
            if (yaw_rc_hold_target >   P_MAX) yaw_rc_hold_target -= DM4310_POS_RANGE;
            if (yaw_rc_hold_target <  -P_MAX) yaw_rc_hold_target += DM4310_POS_RANGE;

            /* MIT 位置环：kp 锁位置、kd 阻尼 */
            mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                     yaw_rc_hold_target, 0.0f, YAW_RC_HOLD_KP, YAW_RC_HOLD_KD, 0.0f);
        }
    } else {
        /* 退出模式：复位锁存标志，下次切回重新锁存当前位置 */
        yaw_rc_hold_latched = 0;
    }
}

/* =================================================================== */
/*    遥控 Yaw 锁定（位置式/速度式 可切换，独立于 yaw_rc_pos_hold）        */
/*                                                                     */
/*    mode == YAW_HOLD_MODE_POS：MIT 位置式，发 pos，行程限 ±半圈         */
/*    mode == YAW_HOLD_MODE_VEL：MIT 速度式，外环位置 P 输出速度，发 vel   */
/*                               行程不限，可转多圈（角度连续展开）        */
/* =================================================================== */

#define YAW_RC_HOLD_VEL_KP        3.0f     /* 速度式：位置误差→速度 的 P 增益 (1/s) */

static float   yaw_hold_target       = 0.0f;  /* 连续目标角度 (rad) */
static uint8_t yaw_hold_latched      = 0;     /* 是否已成功锁存起点 */
static uint8_t yaw_hold_last_mode    = 0xFF;  /* 上次模式，用于切换检测 */

/* 速度式：电机位置的连续展开（独立 static，不干扰 DM4310_Pos_Unwrap_Deg） */
static float   yaw_hold_fb_unwrapped = 0.0f;
static float   yaw_hold_fb_last      = 0.0f;
static uint8_t yaw_hold_fb_first     = 1;

/**
 * @brief  遥控增量式 Yaw 锁定：摇杆转、松杆锁（转到哪锁在哪）
 * @param  rc_ch   遥控 yaw 通道原始值（12bit 有符号，内部做死区）
 * @param  enable  是否进入该模式（1=进入，0=退出）
 * @param  mode    YAW_HOLD_MODE_POS(发 pos 位置式) 或 YAW_HOLD_MODE_VEL(发 vel 速度式)
 * @note   需以 1ms 周期调用；仅当电机已使能(state==0x01)时才锁存起点
 */
void yaw_rc_hold(int16_t rc_ch, uint8_t enable, uint8_t mode)
{
    if (enable) {
        /* 模式切换时重新锁存，避免两种坐标系错位 */
        if (yaw_hold_last_mode != mode) {
            yaw_hold_last_mode = mode;
            yaw_hold_latched   = 0;
        }

        /* 电机已使能、反馈已就绪才锁存起点 */
        if (!yaw_hold_latched && DM_4310.para.state == 0x01) {
            yaw_hold_target       = DM_4310.para.pos;
            yaw_hold_fb_unwrapped = DM_4310.para.pos;
            yaw_hold_fb_last      = DM_4310.para.pos;
            yaw_hold_fb_first     = 0;
            yaw_hold_latched      = 1;
        }

        if (!yaw_hold_latched) {
            return;   /* 还没锁存成功，不发命令 */
        }

        /* 摇杆(死区) → 角速度 → 积分成连续目标角度 */
        float rc_omega = (float)apply_deadzone(rc_ch) * YAW_RC_HOLD_SPEED_GAIN;
        yaw_hold_target += rc_omega * YAW_RC_HOLD_DT;

        if (mode == YAW_HOLD_MODE_POS) {
            /* ---- MIT 位置式：发 pos，钳位在 ±P_MAX（不跨回绕，避免疯转） ---- */
            float pos_cmd = yaw_hold_target;
            if (pos_cmd >   P_MAX) pos_cmd =  P_MAX;
            if (pos_cmd <  -P_MAX) pos_cmd = -P_MAX;
            mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                     pos_cmd, 0.0f, YAW_RC_HOLD_KP, YAW_RC_HOLD_KD, 0.0f);
        } else {
            /* ---- MIT 速度式：外环位置 P 输出速度，发 vel（可转多圈） ---- */
            /* 展开电机位置为连续角度（增量回绕检测，同 DM4310_Pos_Unwrap_Deg 思路） */
            if (yaw_hold_fb_first) {
                yaw_hold_fb_unwrapped = DM_4310.para.pos;
                yaw_hold_fb_last      = DM_4310.para.pos;
                yaw_hold_fb_first     = 0;
            } else {
                float delta = DM_4310.para.pos - yaw_hold_fb_last;
                if (delta >   DM4310_POS_HALF) delta -= DM4310_POS_RANGE;
                if (delta <  -DM4310_POS_HALF) delta += DM4310_POS_RANGE;
                yaw_hold_fb_unwrapped += delta;
                yaw_hold_fb_last = DM_4310.para.pos;
            }

            /* 位置误差 → 速度指令 */
            float vel_cmd = YAW_RC_HOLD_VEL_KP * (yaw_hold_target - yaw_hold_fb_unwrapped);
            if (vel_cmd >   YAW_VELMAX) vel_cmd =  YAW_VELMAX;
            if (vel_cmd <  -YAW_VELMAX) vel_cmd = -YAW_VELMAX;

            /* 摩擦前馈 */
            float tor = friction_compensation_calc_torque(&YAW_fric, vel_cmd);

            /* 发速度：kp=0，只靠 kd 阻尼 */
            mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                     0.0f, vel_cmd, 0.0f, YAW_RC_HOLD_KD, tor);
        }
    } else {
        yaw_hold_latched = 0;
    }
}

void get_groyfromimu(void)
{
	float yaw_rate=BMI088.Gyro[2];

    float cmd = friction_compensation_calc_torque(&YAW_fric, yaw_rate);
	
	mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, yaw_rate, 0.0f,YAW_KD , cmd);
}

/* =================================================================== */
/*    双IMU云台世界锁定 (Dual-IMU Gimbal World Lock)                     */
/*                                                                     */
/*    底盘BMI088 → 角速度前馈 (主力, 1000Hz低延迟)                        */
/*    云台IMU    → 绝对角度反馈 (慢速纠偏, 消除累积漂移)                  */
/*                                                                     */
/*    vel_cmd = BMI088.Gyro[2] + angle_corr + rc_yaw_vel               */
/*                ↑前馈主力      ↑角度修正    ↑遥控微调                   */
/* =================================================================== */

/* 角度环 Kp: 单位「rad/s per 度」。等效带宽(1/s) = Kp × 180/π。
 * 此处取等效 5.0 /s → 5.0×π/180 ≈ 0.0873。
 * ⚠ 量纲坑: YAWImuReceiveData.yaw 是「度」, PID 内部误差也是「度」, 而它的输出被直接当 rad/s 与 vel_cmd 相加。
 *   所以 Kp 必须自带 π/180 折算, 不能按 rad 域增益的直觉填。
 *   (旧值 2.0f 是按 rad 域填的 → 等效增益 ≈2.0×57.3≈114/s, 零点几度误差就顶满 MAX_OUT, 与前馈叠加后云台狂转。)
 *   注: 不能改成在调用处乘 π/180 —— PID 内部的 max_out 限幅(5.0)会退化成度域限幅, 实际只剩 5°/s 权限。
 * 建议等效带宽 3.0~10.0 /s: 偏小则抵消前馈误差的稳态角偏大, 偏大有振荡风险(受上板 γ 的传输/融合延迟限制)。 */
#define GIMBAL_LOCK_ANGLE_KP       (5.0f * PI / 180.0f)
#define GIMBAL_LOCK_ANGLE_KI       0.0f   /* 先不开积分, 观察有无稳态偏置 */
#define GIMBAL_LOCK_ANGLE_KD       0.0f
#define GIMBAL_LOCK_ANGLE_MAX_OUT  5.0f   /* 角度修正最大输出 (rad/s) */
#define GIMBAL_LOCK_ANGLE_MAX_IOUT 2.0f

static pid_type_def gimbal_lock_angle_pid;      /* 角度环PID */
static float        gimbal_lock_target_yaw = 0.0f;  /* 锁定的世界目标角度 */
static uint8_t      gimbal_lock_inited = 0;         /* 初始化标志 */

/**
 * @brief  重置云台锁定模式 (切换模式时调用, 下次进入重新锁定当前角度)
 */
void gimbal_lock_reset(void)
{
    gimbal_lock_inited = 0;
}

void gimbal_world_lock(float x_speed, float y_speed, float rc_yaw_vel,
                       int16_t chassis_spin_speed)
{
    /* ---- 首次调用: 锁定当前云台世界角度 + 初始化PID ---- */
    if (!gimbal_lock_inited) {
        fp32 pid_params[3] = {
            GIMBAL_LOCK_ANGLE_KP,
            GIMBAL_LOCK_ANGLE_KI,
            GIMBAL_LOCK_ANGLE_KD
        };
        PID_init(&gimbal_lock_angle_pid, PID_POSITION, pid_params,
                 GIMBAL_LOCK_ANGLE_MAX_OUT, GIMBAL_LOCK_ANGLE_MAX_IOUT);
        gimbal_lock_target_yaw = YAWImuReceiveData.yaw;
        gimbal_lock_inited = 1;
    }
    gimbal_lock_target_yaw = Normalize_Angle360(gimbal_lock_target_yaw);

    float angle_corr = DM_PositionLoopPIDController(
        &gimbal_lock_angle_pid,
        YAWImuReceiveData.yaw,        /* 反馈: 云台当前世界角度 (度) */
        gimbal_lock_target_yaw         /* 目标: 锁定的世界方向 (度) */
    );

    /* ---- 速度指令 = 前馈主力 + 角度修正 + 遥控微调 ---- */
		float get_speed=-OmniWheel_GetOmega();
		
		if(get_speed<wheel_to_rad(1000))
      {
				get_speed=0;
			}

    float vel_cmd = get_speed     /* ★ 底盘角速度前馈 (主力) */
                  + rc_yaw_vel-angle_corr;    /* 遥控微调 */

    if (vel_cmd >  YAW_VELMAX) vel_cmd =  YAW_VELMAX;
    if (vel_cmd < -YAW_VELMAX) vel_cmd = -YAW_VELMAX;

    /* ---- 摩擦力前馈 ---- */
    float ff_tor = friction_compensation_calc_torque(&YAW_fric, vel_cmd);

    /* ---- 发送DM4310 Yaw电机 (MIT模式) ---- */
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
             0.0f, vel_cmd, 0.0f, YAW_IMUKD,ff_tor);

    /* ---- 底盘麦轮运动 (世界→底盘坐标系变换) ---- */
    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
    float c_angle_deg = yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel;
    ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_spin_speed, c_angle_deg);
}
#define YAW_VELMAX_COUNTER   14.0f

void chassis_spin_counter(float x_speed, float y_speed, float rc_yaw_vel,
                          int16_t chassis_spin_speed)
{
    if (!YAWImu_IsValid()) {
        (void)DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);   /* 仅跟踪编码器, 不使用 */

        mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                 0.0f, 0.0f, 0.0f, YAW_KD, 0.0f);
        Chassis_WheatWheel_Solution(0, 0, 0);
        return;
    }

    /* ---- 1. 云台: 纯速度开环, 目标相对底盘角速度 = -2ω ---- */
    float omega_chassis = BMI088.Gyro[2];              /* 底盘 z 轴角速度 rad/s (SPI 1kHz) */
    float vel_cmd = 2.0f * omega_chassis + rc_yaw_vel;

    if (vel_cmd >  YAW_VELMAX_COUNTER) vel_cmd =  YAW_VELMAX_COUNTER;
    if (vel_cmd < -YAW_VELMAX_COUNTER) vel_cmd = -YAW_VELMAX_COUNTER;

    /* 摩擦力前馈 */
    float ff_tor = friction_compensation_calc_torque(&YAW_fric, vel_cmd);

    /* kp 传 0、不发位置 —— 只靠 YAW_IMUKD 做速度阻尼 */
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
             0.0f,vel_cmd, 0.0f, YAW_KD, ff_tor);

    /* ---- 2. 底盘: 世界速度矢量 → 车体系 + 自旋 ---- */
    float gimbal_rel  = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap  = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
    float c_angle_deg = Normalize_Angle180(yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel);
		
		 rotion_jisuan.world2chassis_angle = c_angle_deg;
    ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_spin_speed, c_angle_deg);
}
void rotation_with_YAW(float x_speed, float y_speed, float rc_yaw_vel,
                          int16_t chassis_spin_speed)
{
    /* 上电锚点: static 保证只在上电第一次调用时采样一次 */
    static float world_direction    = 0.0f;
    static uint8_t world_anchor_set = 0;
    if (!world_anchor_set) {
        world_direction    = YAWImuReceiveData.yaw;
        world_anchor_set   = 1;
    }

    /* 编码器: 展开 + 转度 + 方向(内部已带负号) */
    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);

    /* 上板 yaw: 展开 ±180 回绕成连续值 */
    float yaw_unwrapped = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);

    /* 解算: 底盘相对世界角 (度), 再归一化到 ±180 喂给旋转变换 */
    /* 云台世界朝向(相对锚点) = yaw_unwrapped - world_direction (同 yaw 快轴, 直接减) */
    /* 云台相对底盘角          = RATIO * gimbal_rel (慢轴放大对齐) */
    float c_angle = Normalize_Angle180((yaw_unwrapped - world_direction) - GIMBAL_GEAR_RATIO * gimbal_rel);

    /* 观测量: 复用本次展开结果, 避免再调 unwrap 争抢 static 状态 */
    rotion_jisuan.DM4310_angle        = gimbal_rel;
    rotion_jisuan.world2chassis_angle = c_angle;

    ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_spin_speed, c_angle);
}

void gimbal_spin_world_drive(float x_speed, float y_speed,int16_t chassis_spin_speed)
{
    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
    float c_angle = Normalize_Angle180(yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel);

    ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_spin_speed, c_angle);
}
