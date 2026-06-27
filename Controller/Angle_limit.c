#include "angle_limit.h"
#include "gimbalControl.h"
void AngleLimit_Init(AngleLimit_TypeDef* limit, float min_angle, float max_angle, float safe_margin)
{
    limit->angle_min = min_angle;
    limit->angle_max = max_angle;
    limit->safe_margin = safe_margin;
    limit->is_blocked = 0;
    limit->blocked_dir = 0;
}

uint8_t AngleLimit_CheckMove(AngleLimit_TypeDef* limit, float current_angle, float direction)
{
    // direction: >0 表示正方向运动，<0 表示负方向运动
    
    if (direction > 0) {
        // 正方向运动：检查是否超过上限
        if (current_angle >= limit->angle_max) {
            return 0;  // 禁止
        }
    } else if (direction < 0) {
        // 负方向运动：检查是否超过下限
        if (current_angle <= limit->angle_min) {
            return 0;  // 禁止
        }
    }
    
    return 1;  // 允许
}

int16_t AngleLimit_Protect(AngleLimit_TypeDef* limit, float current_angle, int16_t desired_current)
{
    float angle_high;
    float angle_low;
    int16_t safe_current = desired_current;

    if (limit->safe_margin <= 0.0f) {
        limit->safe_margin = 1.0f;
    }

    angle_high = limit->angle_max - limit->safe_margin;
    angle_low  = limit->angle_min + limit->safe_margin;
    
    // 情况1：超过上限
    if (current_angle >= limit->angle_max) {
        limit->is_blocked = 1;
        limit->blocked_dir = 1;
        // 只能向负方向运动（远离限位），禁止正方向电流
        if (desired_current > 0) {
            safe_current = 0;
        }
        return safe_current;
    }
    
    // 情况2：低于下限
    if (current_angle <= limit->angle_min) {
        limit->is_blocked = 1;
        limit->blocked_dir = -1;
        // 只能向正方向运动（远离限位），禁止负方向电流
        if (desired_current < 0) {
            safe_current = 0;
        }
        return safe_current;
    }
    
    // 情况3：在安全区域内
    limit->is_blocked = 0;
    limit->blocked_dir = 0;
    
    // 情况4：进入警戒区域（接近限位），线性减小电流
    if (current_angle > angle_high && current_angle < limit->angle_max) {
        // 正方向运动时减小输出，负方向不受影响
        if (desired_current > 0) {
            float factor = (limit->angle_max - current_angle) / limit->safe_margin;
            if (factor < 0) factor = 0;
            if (factor > 1) factor = 1;
            safe_current = (int16_t)(desired_current * factor);
        }
    }
    else if (current_angle < angle_low && current_angle > limit->angle_min) {
        // 负方向运动时减小输出，正方向不受影响
        if (desired_current < 0) {
            float factor = (current_angle - limit->angle_min) / limit->safe_margin;
            if (factor < 0) factor = 0;
            if (factor > 1) factor = 1;
            safe_current = (int16_t)(desired_current * factor);
        }
    }
    
    return safe_current;
}



