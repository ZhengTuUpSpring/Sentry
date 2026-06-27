#ifndef _ANGLE_LIMIT_H_
#define _ANGLE_LIMIT_H_

#include <stdint.h>

// 角度限位配置
typedef struct {
    float angle_min;        // 最小角度（圈数）
    float angle_max;        // 最大角度（圈数）
    float safe_margin;      // 安全边界（圈数），进入此区域开始减速
    
    // 内部状态
    uint8_t is_blocked;     // 是否触发限位
     int8_t blocked_dir;    // 被阻挡的方向: 1=正限位, -1=负限位
} AngleLimit_TypeDef;

// 初始化限位
void AngleLimit_Init(AngleLimit_TypeDef* limit, float min_angle, float max_angle, float safe_margin);

// 角度保护：根据目标电流和当前角度，返回安全的电流值
int16_t AngleLimit_Protect(AngleLimit_TypeDef* limit, float current_angle, int16_t desired_current);

// 检查是否允许向某方向运动
uint8_t AngleLimit_CheckMove(AngleLimit_TypeDef* limit, float current_angle, float direction);

#endif
