#ifndef __PIDCON_H__
#define __PIDCON_H__
#include "main.h"
#define SPEED_RECORD_NUM 10
#include "can_receive.h"



enum PID_mode 
{
    PID_POSITION_SPEED,    // 位置控制 - 速度
    PID_POSITION_ANGLE,    // 位置控制 - 角度
    PID_DELTA_SPEED        // 增量式-速度
};

//微分先行的枚举常量
typedef enum
{
    PIDC_D_FIRST_DISABLE = 0,
    PIDC_D_FIRST_ENABLE,
  
} pidc_d_first_e;


extern float Speed_record_A[10];//记录10次速度
extern float Speed_record_B[10];//记录10次速度
extern float Speed_record_C[10];
extern float Speed_record_D[10];
extern int test_Speed;


typedef struct
{
  //可调参数
	enum PID_mode mode;		
	float Kp;
	float Ki;
	float Kd;
  float Kf;                /* 前馈增益：Out_F = Kf * (Target - Pre_Target) */
	float Max_iout;	
	float Max_out;
  float Dt;                /* 控制周期，单位：秒 */
  float I_Variable_A;      /* 变速积分：线性衰减区宽度（从 B 到 A+B 线性减至 0） */
  float I_Variable_B;      /* 变速积分：中心稳态区阈值（|error| <= B 时全速积分） */
  float DeadBand;
  float I_Separate_Thresh; /* 积分分离阈值（绝对值），0 关闭；超出则禁止积分 */
  pidc_d_first_e D_First;
  
	float measure;
	float target;
	float Pout;
	float Iout;
  float Iout_last;
	float Dout;
	float D_item;
	float error[3];
	float OUT;
  float Integral_Error;//积分误差的累计
  float CurrenT_Output;//用于pid计算贮存
  float CurrenT_Output_Last;  
  float Speed_record[10];//用于滤波记录
  float Pre_Now;           /* 上一周期测量值（保留以备扩展） */
  float Pre_Target;        /* 上一周期目标值（用于计算前馈 ΔTarget） */
  float Pre_Out;           /* 上一周期控制输出（D_First=ENABLE 时用于 D 项） */
  float Pre_Error;         /* 上一周期误差（D_First=DISABLE 时用于 D 项） */
  float Pre_Measure;       /* 用于记录上一次电机的转速，便于给下一次的计算使用*/     
  
  
}PID_typedef;

extern PID_typedef Motor[14];


                                                                                                                            
void ADVpid_init(PID_typedef *PID,
              uint8_t mode,
              float Kp,
              float Ki,
              float Kd,
              float Kf,
              float Max_out,
              float max_iout,
              float DeadBand,
              float I_Variable_A,
              float I_Variable_B,
              float Dt,
              float I_Separate_Thresh, 
              pidc_d_first_e D_First);
              
              
float ADVPID_calc(PID_typedef *PID, float measure, float target);
              
float Speed_Low_Filter(float new_Spe,float *speed_Record);
                            
void LimitMax(float *value, float max_limit);
              

#endif
