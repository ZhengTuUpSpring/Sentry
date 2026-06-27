#include "pid_control.h"
#include "main.h"
#include "can_receive.h"


//PID的相关数据的结构体，在.h文件中声明了，在这里进行定义
PID_typedef Motor[14];

float Speed_record_A[10];
float Speed_record_B[10];
float Speed_record_C[10];
float Speed_record_D[10];

int test_Speed;

//这是一个用于对积分进行限幅的函数
static inline void pidc_constrain(float *x, float min_v, float max_v)
{
    if (*x < min_v)
    {
        *x = min_v;
    }
    else if (*x > max_v)
    {
        *x = max_v;
    }
}

//自制的取绝对值的函数，适用于频繁的调用的地方
static inline float pid_absf (float x)
{
  return (x >= 0.0f) ? x : -x;

}

void LimitMax(float *value, float max_limit) 
{
if (*value > max_limit) 
{
   *value = max_limit;  // 如果值大于最大值，设置为最大值
} 
else if (*value < -max_limit) 
{
   *value = -max_limit; // 如果值小于负最大值，设置为负最大值
}

}

//PID的初始化
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
              pidc_d_first_e D_First)
{
        PID->Kp=Kp;
        PID->Ki=Ki;       
        PID->Kd=Kd;
        PID->Kf=Kf;  
        PID->Max_out=Max_out;
        PID->Max_iout=max_iout;
        PID->DeadBand = DeadBand;
        PID->Dt = Dt;
        PID->I_Variable_A = I_Variable_A;
        PID->I_Variable_B = I_Variable_B;
        PID->I_Separate_Thresh = I_Separate_Thresh;
        PID->D_First = D_First;
        PID->Dout=PID->Iout=PID->OUT=PID->Pout=0;
//      PID->error[0]=PID->error[1]=PID->error[2]=PID->D_item=0;
        PID->Integral_Error = 0.0f;
        PID->Pre_Error = 0.0f;
        PID->Pre_Target = 0.0f;
        PID->Pre_Out = 0.0f;
        PID->Pre_Measure = 0.0f;
        PID->Iout_last = 0.0f;
}





//PID的计算函数

//PID的优化

//1.加入死区 2.变速积分 3.积分限幅 4.微分先行 5.前馈控制
float ADVPID_calc(PID_typedef *PID, float measure, float target)
{
  if(PID == NULL)
  {
     return 0;
    
  }
  //函数内部用到的变量
  float p_out = 0.0f;
  float i_out = 0.0f;
  float d_out = 0.0f;
  float f_out = 0.0f;
  float now_error = 0.0f;//用于获取当前误差的变量
  float abs_error = 0.0f;//对当前误差进行绝对值求和
  float speed_ratio = 0.0f;
  
//用于位置式位置环和速度环
if(PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE)
{

//获取误差与死区处理,now_error就是当前的误差,measure就是当前读取到的电机转速
  now_error = target - measure;//当前的误差
  abs_error = pid_absf(now_error);//对误差进行绝对值的获取

  if(abs_error <= PID->DeadBand)
  {
     target = measure;
     now_error = 0.0f;
     abs_error = 0.0f;
     
  }

  /* 比例项 */
  p_out = PID->Kp * now_error;

/*
* 积分项（变速积分）
* - |error| <= B：全速积分（speed_ratio = 1）
* - B < |error| < A+B：线性衰减到 0
* - |error| >= A+B：不积分
* 若 A=B=0：关闭变速积分，恒 1
  
变速积分的作用：
小误差：加强积分，消除静差
大误差：减弱或停止积分，防止积分饱和和超调
*/
  if (PID->I_Variable_A == 0.0f && PID->I_Variable_B == 0.0f)
  {
    speed_ratio = 1.0f;
  }
  else
  {
    if (abs_error <= PID->I_Variable_B)//误差的绝对值小于B边界的阈值，进行全速积分
    {
        speed_ratio = 1.0f;
    }
    else if ((PID->I_Variable_B < abs_error) && (abs_error < PID->I_Variable_A + PID->I_Variable_B))//误差的绝对值处于B与A的区间进行动态的积分，线性衰减
    {
        speed_ratio = (PID->I_Variable_A + PID->I_Variable_B - abs_error) / PID->I_Variable_A;
    }
    else /* abs_error >= A+B，误差过大，不进行积分*/
    {
        speed_ratio = 0.0f;
    }
  }

/* 积分限幅：以 I_Out_Max（对 I 输出的限幅）推导 Integral_Error（积分累积）的界限 */
  if(PID->Max_iout != 0.0f && PID->Ki != 0.0f)
  {
    //动态的获取两个边界
    float limit = PID->Max_iout / PID->Ki;
    if (limit < 0.0f) 
    {
       limit = -limit;
    }
    pidc_constrain(&PID->Integral_Error, -limit, limit);//对积分的总值进行限幅
  }

/* 积分分离阈值（一段式），0 关闭；超出则禁止积分 */
  if (PID->I_Separate_Thresh == 0.0f)//不使用积分阈值分离
  {
    PID->Integral_Error += speed_ratio * PID->Dt * now_error;
    i_out = PID->Ki * PID->Integral_Error;
    PID->Iout = i_out;
  }
  else
  {
    if (abs_error < PID->I_Separate_Thresh)//当前误差的绝对值小于阈值时
    {
        PID->Integral_Error += speed_ratio * PID->Dt * now_error;
        i_out = PID->Ki * PID->Integral_Error;
        PID->Iout = i_out;
    }
    else//若当前的误差过大，则直接丢弃
    {
        PID->Integral_Error = 0.0f;
        i_out = 0.0f;
        PID->Iout = i_out;
    }
    
   }
//对Iout进行限制
  LimitMax(&PID->Iout,PID->Max_iout);
  PID->Iout_last=PID->Iout;

//微分先行PID控制是只对输出量进行微分，而对给定指令不起微分作用，因此它适合于给定指令频繁升降的场合，可以避免指令的改变导致超调过大。
  if (PID->D_First == PIDC_D_FIRST_DISABLE)//不使用微分先行
  {
    d_out = PID->Kd * (now_error - PID->Pre_Error) / PID->Dt;
  }
  else
  {
    d_out = PID->Kd * (PID->measure - PID->Pre_Measure) / PID->Dt;
  }

/* 前馈项：目标增量的比例补偿 */
  f_out = (PID->target - PID->Pre_Target) * PID->Kf;

/* 输出合成与总限幅 */
  PID->OUT = p_out + i_out + d_out + f_out;

  if(PID->Max_out != 0.0f)
  {
    pidc_constrain(&PID->OUT, - PID->Max_out, PID->Max_out);
  }
   /* 保存状态以供下周期使用 */
    PID->Pre_Measure = PID->measure;
    PID->Pre_Target = PID->target;
    PID->Pre_Out = PID->OUT;
    PID->Pre_Error = now_error;    
}

//else if(PID->mode == PID_DELTA_SPEED)
//{
//    PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);
//    PID->Iout = PID->Ki * PID->error[0];
//    PID->D_item = (PID->error[0] - 2.0f*PID->error[1] + PID->error[2]);
//    PID->Dout = PID->Kd * PID->D_item;
//    PID->OUT += PID->Pout + PID->Iout + PID->Dout;
//    LimitMax(&PID->OUT, PID->Max_out);
//}

//else
//{
//      PID->OUT=0;
//}
      return PID->OUT;
}


//位移滤波
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;
    return sum/SPEED_RECORD_NUM;//返回均值
}