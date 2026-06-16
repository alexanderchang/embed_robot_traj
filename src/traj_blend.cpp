#include "traj_blend.h"

AxisState TrajBlend::blend(const AxisState& pre, const AxisState& next, double blend_ratio)
{
    blend_ratio = std::clamp(blend_ratio, 0.0, 1.0);
    AxisState res;

    // 抛物线平滑过渡
    res.pos = pre.pos * (1 - blend_ratio) + next.pos * blend_ratio;
    res.vel = pre.vel * (1 - blend_ratio) + next.vel * blend_ratio;
    // 加速度: 基于控制周期的速度变化率 (EC_PERIOD_US=1000us = 0.001s)
    double dt = EC_PERIOD_US * 1e-6;
    res.acc = (next.vel - pre.vel) / dt;
    res.jerk = 0.0;
    return res;
}