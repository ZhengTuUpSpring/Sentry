#ifndef __FILITER_H
#define __FILITER_H

/**
 * @brief 一阶低通滤波器 (First Order Low Pass Filter)
 * @param input      当前输入值 (新的传感器数据)
 * @param output_last 上一次的输出值 (滤波后的历史值)
 * @param alpha      滤波系数 (0.0 ~ 1.0)
 *                    建议范围: 0.2 ~ 0.6
 * @return float     本次滤波后的输出值
 */
float LPF_FirstOrder(float input, float output_last, float alpha);
#endif
