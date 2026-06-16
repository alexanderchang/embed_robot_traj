#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <algorithm>
#include "common.h"

// 运动状态：位置/速度/加速度/加加速度(冲击Jerk)
struct AxisState
{
    double pos, vel, acc, jerk;
    AxisState() : pos(0), vel(0), acc(0), jerk(0) {}
};

// 1. 三次多项式（点位，加速度不连续）
class CubicPoly
{
private:
    double a0, a1, a2, a3;
    double t_total;
public:
    void init(double s0, double s1, double v0, double v1, double t);
    AxisState get_state(double t) const;
};

// 2. 五次多项式（论文主推，加速度连续、低冲击）
class QuinticPoly
{
private:
    double a0, a1, a2, a3, a4, a5;
    double t_total;
public:
    void init(double s0, double s1, double v0, double v1, double a0_, double a1_, double t);
    AxisState get_state(double t) const;
    double get_t() const { return t_total; }
};

// 3. 3-4-5分段多项式（论文参考文献码垛专用）
class Poly345
{
private:
    double t_total;
    double s_start, s_end;  // 起始和目标位置
public:
    void init(double s0, double s1, double t);
    AxisState get_state(double t) const;
};

// 4. S型加减速曲线（抑振减冲击）
class SCurve
{
private:
    double v_max, a_max, j_max;
    double s_total;
    double t_acc, t_const, t_dec, t_total;
public:
    void init(double s0, double s1, double vmax, double amax, double jmax);
    AxisState get_state(double t) const;
};

// 5. 多轴直线插补（笛卡尔点对点）
class LineInterp
{
private:
    std::vector<double> start, end;
    double t_total;
public:
    void init(const std::vector<double>& s, const std::vector<double>& e, double t);
    std::vector<double> get_pos(double t) const;
};

// 6. 平面圆弧插补（连续路径：焊接/切割）
class CircleInterp
{
private:
    std::vector<double> start, center;
    double radius, angle_total, t_total;
    bool clockwise;
public:
    void init(const std::vector<double>& s, const std::vector<double>& c, double r, double ang, double t, bool cw = false);
    std::vector<double> get_pos(double t) const;
};

// 7. 三阶B样条（复杂曲线：喷涂/轮廓加工）
class BSpline3
{
private:
    std::vector<std::vector<double>> ctrl_pts;
    std::vector<double> knot;
    int dim;
    double t_total;
    double base(int i, int k, double u) const;
public:
    void init(const std::vector<std::vector<double>>& ctrl, double t);
    std::vector<double> get_pos(double t) const;
};

#endif // TRAJECTORY_H