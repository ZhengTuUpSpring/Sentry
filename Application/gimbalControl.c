#include "gimbalControl.h"
#include "driver_4310.h"
#include "driver_6020.h"
#include "arm_math.h"
#include "Angle_limit.h"
#include "can.h"
#include "4310_apply.h"

M6020_HandleTypeDef   yaw_gimbal_motor;
M6020_HandleTypeDef   pitch_gimbal_motor;
/* define smc handle of yaw */
cSMC YawSmcController;

int compensation_factor;
float pitch_cos;
float pitch_sin;

float yaw_beganangle;
int16_t yaw_beganecd;  
float yaw_median_expangle;

int16_t d_ecd;
float d_angle;

int16_t pitch_beganecd;

float target_first = 0;
float rev_first = 0;
float target_second = 0;
float rev_second = 0;

uint8_t state_down=1;
uint8_t state_up=0;
uint8_t pitch_state_down=1;
uint8_t pitch_state_up=0;
float pitch_set = -30.0f;
float yaw_set = 85.0f;
pid_type_def DM4310_pid;
 AngleLimit_TypeDef yaw_limit;

void Gimbal_Init(void)
{

	M6020_Init(0x01, &yaw_gimbal_motor,PID_POSITION, CAN_CMD_VOLTAGE);


	yaw_beganangle=0.0f;
	d_ecd=0;
	d_angle=0;

	AngleLimit_Init(&yaw_limit, YAW_MIN_ANGLE_DEG, YAW_MAX_ANGLE_DEG, YAW_SOFT_LIMIT_BAND_DEG);
//	mit_motor_init();
	pos_motor_init();
	simple_delay(1);
//	pos_motor_enable();
// mit_motor_init();
// mit_motor_enable();

}
int16_t current_test(void)
{
    int16_t target_current;
    int16_t safe_current;
    int16_t current_angle;

    if (Yaw_UpdateRefAngle() == 0U) {
        PID_clear(&yaw_gimbal_motor.current_control_data);
        CAN_cmd_gimbal_yaw(0);
        return 0;
    }

    current_angle = yaw_gimbal_motor.basic_data.ref_angle;

    target_current = PID_Calc(
        &yaw_gimbal_motor.current_control_data,
        yaw_gimbal_motor.basic_data.given_current,
        target_current
    );

    safe_current = AngleLimit_Protect(&yaw_limit, current_angle, target_current);

    CAN_cmd_gimbal_yaw(safe_current);

    return safe_current;
}
int16_t Yaw_Gimbal_ControlOfECD(int16_t targetpostion)
{

	int16_t target_rpm, output;

	if (Yaw_UpdateRefAngle() == 0U) {
		PID_clear(&yaw_gimbal_motor.position_control_data);
		PID_clear(&yaw_gimbal_motor.speed_control_data);
		return 0;
	}
  target_rpm=PID_Calc(&yaw_gimbal_motor.position_control_data, yaw_gimbal_motor.basic_data.ref_angle, targetpostion);
	  output=PID_Calc(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);

		return output;

}
 float Float_Limit(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
	return value;
}

uint8_t Yaw_UpdateRefAngle(void)
{
    M6020_GetBasicData(&yaw_gimbal_motor);

//    if (yaw_gimbal_motor.basic_data.is_online == 0U) {
//        return 0U;
//    }

    yaw_gimbal_motor.basic_data.ref_angle =
        ((yaw_gimbal_motor.basic_data.ecd / 8192.0f) * 360.0f) - 180.0f;

    return 1U;
}

void Yaw_Motor_Control(float target_position)
{
    float current_angle;
    float target_angle;
    float target_rpm;
    float target_current;
    float angle_error;
    float feedforward_current = 0.0f;
    int16_t desired_current;
    int16_t safe_current;

//    if (Yaw_UpdateRefAngle() == 0U) {
//        PID_clear(&yaw_gimbal_motor.position_control_data);
//        PID_clear(&yaw_gimbal_motor.speed_control_data);
//        CAN_cmd_gimbal_yaw(0);
//        return;
//    }

    current_angle = yaw_gimbal_motor.basic_data.ref_angle;
    target_angle = Float_Limit(target_position,
                               yaw_limit.angle_min,
                               yaw_limit.angle_max);

    angle_error = target_angle - current_angle;

    target_rpm = PID_Calc(&yaw_gimbal_motor.position_control_data,
                          current_angle,
                          target_angle);

    target_rpm = Float_Limit(target_rpm, -YAW_MAX_RPM, YAW_MAX_RPM);

    target_current = PID_Calc(&yaw_gimbal_motor.speed_control_data,
                              yaw_gimbal_motor.basic_data.speed_rpm,
                              target_rpm);

    if (angle_error > YAW_POS_DEADBAND_DEG) {
        feedforward_current = YAW_FF_STATIC_CURRENT + YAW_FF_SPEED_CURRENT * target_rpm;
    } else if (angle_error < -YAW_POS_DEADBAND_DEG) {
        feedforward_current = -YAW_FF_STATIC_CURRENT + YAW_FF_SPEED_CURRENT * target_rpm;
    }

    target_current += feedforward_current;
    target_current = Float_Limit(target_current, -YAW_MAX_CURRENT, YAW_MAX_CURRENT);

    desired_current = (int16_t)target_current;

    safe_current = AngleLimit_Protect(&yaw_limit,
                                      current_angle,
                                      desired_current);

    CAN_cmd_gimbal_yaw(safe_current);
}
//void Yaw_Motor_Control(float target_position)
//{
//    int16_t target_rpm;      
//    int16_t target_current;  
//    int16_t safe_current;    
//    int16_t current_angle;
//    
//    M6020_GetBasicData(&yaw_gimbal_motor);
//	  yaw_gimbal_motor.basic_data.ref_angle = ((yaw_gimbal_motor.basic_data.ecd / 8192.0f) * 360.0f) - 180.0f;
//    current_angle = yaw_gimbal_motor.basic_data.ref_angle;
//    
//	  float current_angle_turns = current_angle / 360.0f;

//    target_rpm = PID_Calc(&yaw_gimbal_motor.position_control_data, 
//                          yaw_gimbal_motor.basic_data.ref_angle, 
//                          target_position);
//    

//    target_current = PID_Calc(&yaw_gimbal_motor.speed_control_data, 
//                              yaw_gimbal_motor.basic_data.speed_rpm, 
//                              target_rpm);
//	
//    if (target_current > 16384) target_current = 16384;
//    if (target_current < -16384) target_current = -16384;

//    safe_current = AngleLimit_Protect(&yaw_limit, current_angle_turns, target_current);
  
//    CAN_cmd_gimbal_yaw(safe_current);
//}



/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          杈惧pitch杞翠覆绾id鎺у埗浠诲姟
  * @param[in]      targetpostion锛歱itch杞寸洰鏍囩粷瀵圭紪鐮佸櫒鍊?
  * @retval         pitch杞翠覆绾id杈撳嚭
  */
int16_t Pitch_Gimbal_ControlOfECD(int16_t targetpostion)
{
	/*涓茬骇pid*/
	M6020_GetBasicData(&pitch_gimbal_motor);//鑾峰彇鏁版嵁
	int16_t target_rpm=PID_Calc(&pitch_gimbal_motor.position_control_data,pitch_gimbal_motor.basic_data.ecd , targetpostion);//浣嶇疆鐜?
	int16_t output=PID_Calc(&pitch_gimbal_motor.speed_control_data, pitch_gimbal_motor.basic_data.speed_rpm, target_rpm);       //瑙掑害鐜?
	
	return output;
}
int16_t Pitch_Gimbal_Controlspeed(int16_t targetspeed)
{
	/*涓茬骇pid*/
	M6020_GetBasicData(&pitch_gimbal_motor);//鑾峰彇鏁版嵁
	int16_t output=PID_Calc(&pitch_gimbal_motor.speed_control_data,pitch_gimbal_motor.basic_data.speed_rpm , targetspeed);//浣嶇疆鐜?
	
	return output;
}
/**
  * @func			int16_t Yaw_Gimbal_Task(int16_t targetpostion)
  * @brief          yaw杞翠覆绾id鎺у埗浠诲姟
  * @brief          浠awh杞寸粷瀵圭紪鐮佸櫒涓哄弬鑰?
  * @param[in]      targetpostion锛歽aw杞寸洰鏍囪搴?
  * @retval         yaw杞翠覆绾id杈撳嚭
  */
int16_t pos_cnt;


int16_t Yaw_speedControl(int16_t target_rpm)
{
	M6020_GetBasicData(&yaw_gimbal_motor);//鑾峰彇鏁版嵁
	int16_t output=PID_Calc(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//瑙掑害鐜?
	return output;
}

int16_t Yaw_Gimbal_ControlOfVision(float target, float rev)
{
	int16_t target_rpm=PID_Calc(&yaw_gimbal_motor.position_control_data,rev, target);//浣嶇疆鐜?
	int16_t output=PID_Calc(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//閫熷害鐜?
	
	return output;
}

/**
  * @func			int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
  * @brief          yawh杞翠覆绾id鎺у埗浠诲姟2
  * @brief          浠ラ檧铻轰华瑙掑害(涓栫晫鍧愭爣)涓哄弬鑰?
  * @param[in]      realangle锛歽aw杞村疄鏃惰搴︼紝鑼冨洿[-180,180]
  * @param[in]      targetpostion锛歽aw杞寸洰鏍囪搴?鑼冨洿[-180,180]
  * @retval         yaw杞翠覆绾id杈撳嚭
  */
int16_t Yaw_Gimbal_ControlOfIMU(float realangle ,float targetangle)
{
	M6020_GetBasicData(&yaw_gimbal_motor);

	target_first = targetangle;
	rev_first = realangle;

	int num=0;
	if(targetangle>0)
	{
		num=(int)targetangle/180;
		if(num%2==0)
		{
			targetangle=targetangle-num*180;
		}
		else
		{
			targetangle=-180+(targetangle-num*180);
		}
	}
	else
	{
		num=(int)targetangle/180;
		if(num%2==0)
		{
			targetangle=targetangle-num*180;
		}
		else
		{
			targetangle=180+(targetangle-num*180);
		}
	}
	//printf("%f,%f\n",realangle,targetangle);	
	float rev=0.0f;
	float set=0.0f;

	if(targetangle>=0)
	{
		if(realangle>=0)
		{
			rev=realangle;
			set=targetangle;
		}
		else
		{
			if(realangle>=targetangle-180)
			{
				rev=realangle;//rev=-realangle; //rev=realangle
				set=targetangle;//set=targetangle;//set=targetangle
			}
			else
			{
				rev=realangle+180;//rev=(180+realangle);   
				set=targetangle-180;//set=-(180-targetangle);set=targetangle-180
			}
		}
	}
	else
	{
		if(realangle<0)
		{
			rev=realangle;
			set=targetangle;
		}
		else
		{
			if(realangle<=targetangle+180)
			{
				rev=realangle;//rev=-realangle;//rev=realangle
				set=targetangle;//targetangle
			}
			else
			{
				rev=-(180-realangle);
				set=180+targetangle;
			}
		}
	}

	set=set*8191/360;
	rev=rev*8191/360;
	
	target_second = set;
	rev_second = rev;
	
//	vofa_send_data(0, set*360/8192);
//	vofa_send_data(0, rev*360/8192);
//	vofa_sendframetail();
	
	/*涓茬骇pid*/
	int16_t target_rpm=PID_Calc(&yaw_gimbal_motor.position_control_data,rev, set);//浣嶇疆鐜?
	int16_t output=PID_Calc(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//閫熷害鐜?
  
	return output;
}

/**
  * @func			float Update_ReferenceBeganAngle(User_USART* jy901_data)
  * @brief          鏇存柊鍙傝€冭搴?
  * @brief          浠ラ檧铻轰华瑙掑害(涓栫晫鍧愭爣)涓哄弬鑰?
  * @param[in]      jy901_data锛歫y901鍙ユ焺
  * @retval         yaw杞存洿鏂板悗鐨勫弬鑰冭搴?
  */
float Update_ReferenceBeganAngle(float imuYaw)
{
	// yaw_beganangle=jy901_data->angle.angle[2];
	yaw_beganangle=imuYaw;
	return yaw_beganangle;
}

/**
  * @func			float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
  * @brief          璁＄畻yaw杞村湪鏈烘涓€间綅缃殑瑙掑害
  * @brief          浠ラ檧铻轰华瑙掑害(涓栫晫鍧愭爣)涓哄弬鑰?
  * @param[in]      jy901_data锛歫y901鍙ユ焺
  * @param[in]      device锛?020鐢垫満鍙ユ焺
  * @retval         yaw杞村湪鏈烘涓€间綅缃殑瑙掑害
  */
float Get_ReferenceMedianAngle(float imuYaw, M6020_HandleTypeDef* device)
{
	/*鑾峰彇鍒濆瑙掑害*/
	yaw_beganangle=imuYaw;
	/*璁＄畻鍋忓樊瑙掑害*/
	d_ecd=device->basic_data.ecd-YAW_MEDIAN;
	d_angle=d_ecd*360/8191;
	/*璁＄畻鏈烘涓€艰搴*/
	yaw_median_expangle=  -d_angle;
	
	return yaw_median_expangle;
}

/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          鏇存柊鍙傝€冭搴?
  * @brief          浠ョ粷瀵圭紪鐮佸€间负鍙傝€?
  * @param[in]      device锛?020鐢垫満鍙ユ焺
  * @retval         yaw杞存洿鏂板悗鐨勫弬鑰冭搴?缁濆缂栫爜鍣ㄥ€?
  */
int16_t Update_ReferenceBeganECD(M6020_HandleTypeDef* device)
{
	int16_t ecd=device->basic_data.ecd;
	if(device->motor_id == 1)
	{
		yaw_beganecd=ecd;
	}
	else if(device->motor_id == 2)
	{
		pitch_beganecd=ecd;
	}
	
	return ecd;
}
void simple_delay(uint32_t ms)
{
	uint32_t start=HAL_GetTick();
	while(HAL_GetTick()-start<ms);
}
/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch杞撮噸鍔涜ˉ鍋?
  * @param[in]      device锛?020鐢垫満鍙ユ焺
  * @retval         琛ュ伩鍊?
  */
int16_t GravityCompensation(const M6020_HandleTypeDef* device)
{
	float angle=(device->basic_data.ecd-PITCH_MEDIAN)*360/8191;//鏈烘涓€兼槸鏍规嵁pitch杞存按骞虫椂鐨勭紪鐮佸櫒鍊艰瀹氱殑
	float cos=0.0f;
	float sin=0.0f;
	arm_sin_cos_f32(angle, &sin, &cos);
	pitch_cos=cos;
	pitch_sin=sin;
	//printf("%f,%f,%f\n",angle,cos,compensation_factor*pitch_cos);
	return compensation_factor*pitch_cos;
}

void pitch_con(void)
{
    float pitch_real = DM_4310.para.pos;

    if (pitch_state_up == 0 && pitch_state_down == 1) {
        pitch_set -= 0.28f;

        if (pitch_set < PITCH_MIN) {
            pitch_set = PITCH_MIN;
        }

        if (pitch_real <= PITCH_MIN + PITCH_SCAN_REVERSE_BAND_DEG) {
            pitch_state_up = 1;
            pitch_state_down = 0;
            pitch_set = PITCH_MIN + PITCH_SCAN_REVERSE_BAND_DEG;
        }
    } else if (pitch_state_up == 1 && pitch_state_down == 0) {
        pitch_set += 0.28f;

        if (pitch_set > PITCH_MAX) {
            pitch_set = PITCH_MAX;
        }

        if (pitch_real >= PITCH_MAX - PITCH_SCAN_REVERSE_BAND_DEG) {
            pitch_state_up = 0;
            pitch_state_down = 1;
            pitch_set = PITCH_MAX - PITCH_SCAN_REVERSE_BAND_DEG;
        }
    } else {
        pitch_state_up = 0;
        pitch_state_down = 1;
        pitch_set = pitch_real;
    }

    pitch_set = Float_Limit(pitch_set, PITCH_MIN, PITCH_MAX);
//  pitch_mit_control(pitch_set);
    pitch_spepos_control(pitch_set);
}
void yaw_con(void)
{
    float yaw_real;

//    if (Yaw_UpdateRefAngle() == 0U) {
//        return;
//    }
    yaw_real = yaw_gimbal_motor.basic_data.ref_angle;

    if (state_up == 0 && state_down == 1) {
        yaw_set -= 0.15f;

        if (yaw_set < YAW_MIN_ANGLE_DEG) {
            yaw_set = YAW_MIN_ANGLE_DEG;
        }

        if (yaw_real <= YAW_MIN_ANGLE_DEG + error_admit) {
            state_up = 1;
            state_down = 0;
            yaw_set = YAW_MIN_ANGLE_DEG + error_admit;
        }
    } else if (state_up == 1 && state_down == 0) {
        yaw_set += 0.15f;

        if (yaw_set > YAW_MAX_ANGLE_DEG) {
            yaw_set = YAW_MAX_ANGLE_DEG;
        }

        if (yaw_real >= YAW_MAX_ANGLE_DEG - error_admit) {
            state_up = 0;
            state_down = 1;
            yaw_set = YAW_MAX_ANGLE_DEG - error_admit;
        }
    } else {
        state_up = 0;
        state_down = 1;
        yaw_set = yaw_real;
    }

    yaw_set = Float_Limit(yaw_set, YAW_MIN_ANGLE_DEG, YAW_MAX_ANGLE_DEG);
    Yaw_Motor_Control(yaw_set);
}
//void yaw_con(void)
//{
//	  M6020_GetBasicData(&yaw_gimbal_motor);
//	  yaw_gimbal_motor.basic_data.ref_angle = ((yaw_gimbal_motor.basic_data.ecd/8192.0f)*360.0f)-180.0f;
//	

//			if(state_up==0||state_down==1)
//      {
//	     yaw_set-=0.3f;
//					 if(yaw_set<YAW_MIN)
//	        {
//		     state_up =1;
//		     state_down =0;
//	        }

//				Yaw_Motor_Control(yaw_set);
//					
//      }
//			else if(state_up==1||state_down==0)
//    { 
//		     yaw_set+=0.3f;

//        if(yaw_set>YAW_MAX)
//	          {
//	               state_up =0;
//		           state_down =1;
//	          }

//				Yaw_Motor_Control(yaw_set);
//				
//    }    

//}
gun_gdata gundata;
float Get_GunAngle(void)
{
    float motor_pos = DM_4310.para.pos;
    float gun_angle = motor_pos - pitch_offset;  
	
	if(gun_angle>20)gun_angle=20;
	if(gun_angle<-20)gun_angle=-20;
	
    return gun_angle;
}

float Get_GravityTorque(void)
{
    gundata.gun_angle_deg = Get_GunAngle();  
    gundata.gun_angle_rad = gundata.gun_angle_deg * PAI / 180.0f;
    gundata.torque = MAX_TOUR * cosf(gundata.gun_angle_rad);
	  
    return gundata.torque;
}
