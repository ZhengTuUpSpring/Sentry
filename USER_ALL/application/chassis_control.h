#ifndef CH_CON_H
#define CH_CON_H

#include "control_3508.h"
#include "main.h"
#include "arm_math.h"

#define CHASSIS_WHEEL_RADIUS 0.07625
#define CHASSIS_WHEEL_MOTRO_REDUCTION_RATIO 19
#define CHASSIS_WHEEL_L 0.3
#define CHASSIS_WHEEL_W 0.4
#define LOW_FILTER_A 0.9

#define CHASSIS_ANGLEPID_KP 1.0f 
#define CHASSIS_ANGLEPID_KI 0.0001f 
#define CHASSIS_ANGLEPID_KD 0.00f 

#define YAW_ANGLE_KP 1.0f
#define YAW_ANGLE_KI  0.0f
#define YAW_ANGLE_KD  0.0f

#define CHASSIS_WHEEL_SIGN_A 1.0f
#define CHASSIS_WHEEL_SIGN_B 1.0f
#define CHASSIS_WHEEL_SIGN_C 1.0f
#define CHASSIS_WHEEL_SIGN_D 1.0f

#define CHASSIS_CMD_TO_YAW_RADPS  0.001813f
#define YAW_KP 2.0f  
#define YAW_IMUKD 1.8f
#define YAW_KD  1.85f
#define RAD2DEG 57.29578f

#define GIMBAL_GEAR_RATIO   3.822f

#define deviation_angle 19.0f

#define YAW_VELMAX 7.0f

#define YAW_RC_KD 0.3f
#define rpm_to_rad 0.10472
#define WORLD_BASE_OFFSET_DEG  (-24.13f)

#define yaw_sensity 1
#define rad2degree 0.0174533f

/* 遥控 Yaw 锁定模式（供 yaw_rc_hold 使用） */
typedef enum {
    YAW_HOLD_MODE_POS = 0,   /* MIT 位置式：发 pos，行程 ±半圈 */
    YAW_HOLD_MODE_VEL = 1,   /* MIT 速度式：外环位置P→发 vel，可转多圈 */
} yaw_hold_mode_e;

typedef struct{

	int16_t c_x;
	int16_t c_y;

	int16_t c_rotate;
	float32_t beganangle_cwtw;
  float32_t angle_cmtw;
	float32_t cos;
	float32_t sin;

}Chassis_CoordinateSystem_Typedef;

typedef struct{

  float DM4310_angle;
	float world2chassis_angle;
	float yaw_unwarp;
}chassis_rotation;
extern  chassis_rotation rotion_jisuan;

extern void sin_control(float x_speed,float y_speed,float rc_yaw);
extern void Chassis_Init(void);
extern M3508_HandleTypeDef wheelJoint_motorHandle[4];	
extern void Chassis_WheatWheel_Solution(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector);
extern void ChassisGyro_Task(int16_t worldXvector, int16_t worldYvector, int16_t worldZrotate, float c_angle_deg);
																			
extern void chassis_holdYAW_GO(int16_t chassis_selfZ_vel, float x_speed, float y_speed,float rc_yaw_vel);
extern void chassis_holdYAW_reset(void);
extern void chassis_yaw_mode_reset(void);
extern float OmniWheel_GetOmega(void);	

extern void gimbal_world_lock(float x_speed, float y_speed, float rc_yaw_vel,
                               int16_t chassis_spin_speed);
extern void gimbal_lock_reset(void);

extern void yaw_rc_hold(int16_t rc_ch, uint8_t enable, uint8_t mode);

extern void chassis_spin_counter(float x_speed, float y_speed,int16_t chassis_spin_speed);

extern void chassis_spin_counter_test(float x_speed, float y_speed, float rc_yaw_vel,
                          int16_t chassis_spin_speed);

extern void chassis_nav_auto(float x_speed,float y_speed,float spin_speed,float aim_get);

extern void chassis_nav_auto_v2(float x_speed,float y_speed,float spin_speed,float aim_get);

extern void chassisYAW_RC(float ref,float set);

#endif
