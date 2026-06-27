#include "Filiter.h"
#include "math.h"

float LPF_FirstOrder(float input, float output_last, float alpha)
{
    /* 参数保护，防止 alpha 越界导致系统不稳定 */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    return (1.0f - alpha) * input + alpha * output_last;
}
