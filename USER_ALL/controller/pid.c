			/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "math.h"

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
	
	pid->integral_valve=0;
}

static inline float Normalization_func(float exp, float real) 
{
    float error = exp - real;
    if (error > 180.0f)
	{
        error -= 360.0f;
    } 
	else if (error < -180.0f)
	{
        error += 360.0f;
    }
    return error;
}
/**
 * @brief  PID 初始化（PID_t 版本）
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_max, float output_max)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral     = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max   = output_max;
    pid->prev_error   = 0.0f;
    pid->prev_measure = 0.0f;
    pid->first_run    = 1;
}

/**
 * @brief  速度环 PID 计算（微分先行 + 条件积分抗饱和）
 *         用于级联内环，适合正弦变速等连续变化目标场景。
 * @param  pid       PID 结构体
 * @param  target    目标速度
 * @param  measure   当前测量速度（陀螺仪）
 * @param  dt        控制周期（秒）
 * @return PID 输出（速度指令）
 */
float PID_Calc(PID_t *pid, float target, float measure, float dt)
{
    float error = target - measure;

    /* P 项 */
    float p_out = pid->kp * error;

    /* D 项：微分先行 — 对测量值(陀螺仪)求导，避免目标跳变产生尖峰 */
    float d_out = 0.0f;
    if (pid->first_run) {
        pid->prev_measure = measure;
        pid->first_run    = 0;
    } else {
        float d_measure = (measure - pid->prev_measure) / dt;
        d_out = -pid->kd * d_measure;
        pid->prev_measure = measure;
    }

    /* I 项：条件积分抗饱和 — 当输出饱和且误差同向时才冻结积分 */
    float output_no_i = p_out + d_out;
    int    frozen = 0;

    if ((output_no_i + pid->ki * pid->integral >  pid->output_max && error > 0) ||
        (output_no_i + pid->ki * pid->integral < -pid->output_max && error < 0))
    {
        frozen = 1;  // 积分饱和方向与误差同向，冻结积分
    }

    if (!frozen) {
        pid->integral += error * dt;
    }

    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    float i_out = pid->ki * pid->integral;

    /* 合成输出并限幅 */
    float output = p_out + i_out + d_out;

    if (output >  pid->output_max) output =  pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    return output;
}
 float DM_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = Normalization_func(set,ref); //过零处理
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
		//积分限幅
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
		//输出限幅
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    
	else
	{
		return 0;
	}
}
