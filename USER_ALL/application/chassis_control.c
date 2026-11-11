#include "main.h"
#include "chassis_control.h"
#include "control_3508.h"
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
#include "chassis_angle_calculation.h"

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
	fp32 pid_yaw[3]={
	 YAW_ANGLE_KP,
		YAW_ANGLE_KI,
		YAW_ANGLE_KD
	};

	PID_init(&chassis_angle_pid,PID_POSITION,pid_angle, 5000, 5000);
	 PID_init(&DM4310_pid,PID_POSITION,pid_yaw,1000,1000);
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
//	taskENTER_CRITICAL();
//	for(uint8_t dbg_i = 0; dbg_i < 4; dbg_i++)
//	{
//		g_motor_snap.wheel_t[dbg_i] = (float)wheelJoint_motorHandle[dbg_i].basic_data.given_current * Wheel_T_Coeff;
//		g_motor_snap.wheel_w[dbg_i] = (float)wheelJoint_motorHandle[dbg_i].basic_data.speed_rpm * Velocity_Constant;
//	}
//	g_motor_snap.pre_chassis_power = total_power;
//	
//	g_motor_snap.tick = xTaskGetTickCount();
//	g_motor_snap.seq++;
//	taskEXIT_CRITICAL();

	CAN_cmd_chassis((int16_t )set_current[0], (int16_t )set_current[1], (int16_t )set_current[2], (int16_t )set_current[3]);
}
/**
  * @brief          底盘小陀螺任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      worldZrotate：小陀螺转速
  * @param[in]      c_angle：世界/云台坐标系与底盘坐标系的夹角 
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
#define YAW_RC_HOLD_SPEED_GAIN   0.01f 
#define YAW_RC_HOLD_KP           10.0f     /* MIT 位置环 Kp (N·m/rad)，越大锁得越硬 */
#define YAW_RC_HOLD_KD           1.0f      /* MIT 位置环 Kd 阻尼，抑制到位振荡 */
#define YAW_RC_HOLD_DT           0.001f    /* 调用周期，与任务 1ms 循环保持一致 */

#define YAW_RC_HOLD_VEL_KP        3.0f     /* 速度式：位置误差→速度 的 P 增益 (1/s) */
#define YAW_RC_HOLD_RC_DEADZONE   0.05f    /* 摇杆回中判零阈值 (rad/s)，小于它视为回中 */

static float   yaw_hold_target       = 0.0f;  /* 连续目标角度 (rad) */
static uint8_t yaw_hold_latched      = 0;     /* 是否已成功锁存起点 */
static uint8_t yaw_hold_last_mode    = 0xFF;  /* 上次模式，用于切换检测 */

/* 速度式：电机位置的连续展开（独立 static，不干扰 DM4310_Pos_Unwrap_Deg） */
static float   yaw_hold_fb_unwrapped = 0.0f;
static float   yaw_hold_fb_last      = 0.0f;
static uint8_t yaw_hold_fb_first     = 1;

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

        /* ---- 先把电机位置展开成连续值（两个模式、回中追平共用） ---- */
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

        /* 摇杆(死区) → 角速度 → 积分成连续目标角度 */
        float rc_omega = (float)apply_deadzone(rc_ch) * YAW_RC_HOLD_SPEED_GAIN;
        if (fabsf(rc_omega) >= YAW_RC_HOLD_RC_DEADZONE) {
            yaw_hold_target += rc_omega * YAW_RC_HOLD_DT;
        } else {
            /* 摇杆回中：目标瞬间追平当前实际角，误差归零，电机停在当前位置 */
            yaw_hold_target = yaw_hold_fb_unwrapped;
        }

        if (mode == YAW_HOLD_MODE_POS) {
            /* ---- MIT 位置式：发 pos，钳位在 ±P_MAX（不跨回绕，避免疯转） ---- */
            float pos_cmd = yaw_hold_target;
            if (pos_cmd >   P_MAX) pos_cmd =  P_MAX;
            if (pos_cmd <  -P_MAX) pos_cmd = -P_MAX;
            mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                     pos_cmd, 0.0f, YAW_RC_HOLD_KP, YAW_RC_HOLD_KD, 0.0f);
        } else {
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
		
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, YAW_VEL, 0.0f, YAW_KD,0.0f);

    float gimbal_rel = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap = YawImu_Unwrap_Deg(YAWImuReceiveData.yaw);
		
    float c_angle_deg = yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel;

  ChassisGyro_Task(-x_speed * 6, y_speed * 6, chassis_selfZ_vel, c_angle_deg);
}

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

void chassis_spin_counter(float x_speed, float y_speed,int16_t chassis_spin_speed)
{
    if (!YAWImu_IsValid()) {
        (void)DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);

        mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                 0.0f, 0.0f, 0.0f, YAW_KD, 0.0f);
        Chassis_WheatWheel_Solution(0, 0, 0);
        return;
    }

    float omega_chassis = BMI088.Gyro[2];           
    float vel_cmd = 1.5f * omega_chassis;

    if (vel_cmd >  YAW_VELMAX_COUNTER) vel_cmd =  YAW_VELMAX_COUNTER;
    if (vel_cmd < -YAW_VELMAX_COUNTER) vel_cmd = -YAW_VELMAX_COUNTER;

    /* 摩擦力前馈 */
    float ff_tor = friction_compensation_calc_torque(&YAW_fric, vel_cmd);

    mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
             0.0f,vel_cmd, 0.0f, YAW_KD, ff_tor);
		
    /* ---- 2. 底盘: 世界速度矢量 → 车体系 + 自旋 ---- */
    float gimbal_rel  = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap  =YawImu_Unwrap_Deg(yaw_angle);
    float c_angle_deg = Normalize_Angle180(yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel+deviation_angle);
		
    ChassisGyro_Task(-y_speed * 6, x_speed * 6, chassis_spin_speed, c_angle_deg);
}
static uint8_t prev_in_spin_mode = 0;   /* 上一次调用是否处于该模式 */
float DMget_ref_deg;                     /* 进入本模式瞬间记录的 DM4310 位置角(度) */

void chassis_spin_counter_test(float x_speed, float y_speed, float rc_yaw_vel,
                               int16_t chassis_spin_speed)
{
    if (!YAWImu_IsValid()) {
        (void)DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);   /* 仅跟踪编码器, 不使用 */
        mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, 0.0f, 0.0f, YAW_KD, 0.0f);
        Chassis_WheatWheel_Solution(0, 0, 0);
        prev_in_spin_mode = 0;   /* 本帧不算"在模式内"，下次进来会重记 */
        return;
    }

    if (!prev_in_spin_mode) {
        /* 上升沿: 刚进入本模式, 重记参考角 */
        DMget_ref_deg = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    }
    prev_in_spin_mode = 1;

    /* ---- 1. 云台: 纯速度开环 ---- */
    float omega_chassis = BMI088.Gyro[2];
    float vel_cmd = 1.5f * omega_chassis + rc_yaw_vel;
    if (vel_cmd >  YAW_VELMAX_COUNTER) vel_cmd =  YAW_VELMAX_COUNTER;
    if (vel_cmd < -YAW_VELMAX_COUNTER) vel_cmd = -YAW_VELMAX_COUNTER;
    float ff_tor = friction_compensation_calc_torque(&YAW_fric, vel_cmd);
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id, 0.0f, vel_cmd, 0.0f, YAW_KD, ff_tor);

    /* ---- 2. 底盘 ---- */
    float gimbal_rel  = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos) - DMget_ref_deg;
    float yaw_unwrap  = YawImu_Unwrap_Deg(yaw_angle);
    float c_angle_deg = Normalize_Angle180(yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel);

    ChassisGyro_Task(-y_speed * 6, x_speed * 6, chassis_spin_speed, c_angle_deg);
}

void chassis_nav_auto(float x_speed,float y_speed,float spin_speed,float aim_get)
{
	    if (!YAWImu_IsValid()) {
        (void)DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);  

        mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
                 0.0f, 0.0f, 0.0f, YAW_KD, 0.0f);
        Chassis_WheatWheel_Solution(0, 0, 0);
        return;
    }
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
        YAWImuReceiveData.yaw,      
        gimbal_lock_target_yaw       
    );

    float omega_chassis = BMI088.Gyro[2];            
		float get_speed=-OmniWheel_GetOmega();
		float vel_cmd=0;
		if(aim_get==1){
			vel_cmd = 1.5f * omega_chassis ;
		}
		else if(aim_get==2)
		{
	 vel_cmd = get_speed -angle_corr;   
		}  
		else {vel_cmd=0;}

    if (vel_cmd >  YAW_VELMAX_COUNTER) vel_cmd =  YAW_VELMAX_COUNTER;
    if (vel_cmd < -YAW_VELMAX_COUNTER) vel_cmd = -YAW_VELMAX_COUNTER;

    float ff_tor = friction_compensation_calc_torque(&YAW_fric, vel_cmd);
		
    mit_ctrl(&hcan1, &DM_4310, DM_4310.id,
             0.0f,vel_cmd, 0.0f, YAW_KD, ff_tor); 

    float gimbal_rel  = DM4310_Pos_Unwrap_Deg(DM_4310.para.pos);
    float yaw_unwrap  =YawImu_Unwrap_Deg(yaw_angle);
    float c_angle_deg = Normalize_Angle180(yaw_unwrap - GIMBAL_GEAR_RATIO * gimbal_rel+deviation_angle);
		
    ChassisGyro_Task(-y_speed * 6, x_speed * 6, spin_speed, c_angle_deg);
	
	}
void chassisYAW_RC(float ref,float set)
{
	float speed=DM_PositionLoopPIDController(&DM4310_pid,ref,set);
	mit_ctrl(&hcan1,&DM_4310,DM_4310.id,0.0f,speed,0.0f,YAW_KD,0.0f);
}

