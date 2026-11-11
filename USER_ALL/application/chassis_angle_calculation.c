#include "chassis_angle_calculation.h"
#include "main.h"
#include "arm_math.h"
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

