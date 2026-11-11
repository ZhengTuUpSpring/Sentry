#ifndef __CHASSIS_CAL
#define __CHASSIS_CAL

#define RAD2DEG            57.29578f
#define DM4310_POS_RANGE   6.28318531f  
#define DM4310_POS_HALF    3.14159265f   

#define GEAR_RATIO       19.20320856f
#define CHASSIS_R        0.231f
#define WHEEL_DIAMETER   0.152f

extern float Limit_Float(float value, float limit);
extern float Normalize_Angle360(float angle);
extern float Normalize_Angle180(float angle);
extern float wheel_to_rad(float vlaue);
extern float wheel_to_omega(float value);
extern float DM4310_Pos_Unwrap_Deg(float raw_pos);
extern float YawImu_Unwrap_Deg(float raw_yaw);
#endif
