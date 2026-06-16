#ifndef TRAJ_BLEND_H
#define TRAJ_BLEND_H

#include "trajectory.h"

// 轨迹平滑过渡（抛物线过渡，论文直线/圆弧衔接方案）
class TrajBlend
{
public:
    // 两段轨迹过渡插值
    AxisState blend(const AxisState& pre, const AxisState& next, double blend_ratio);
};

#endif // TRAJ_BLEND_H