#include "trajectory.h"

// ===================== 三次多项式 =====================
void CubicPoly::init(double s0, double s1, double v0, double v1, double t)
{
    t_total = t;
    double T = t;
    double T2 = T * T;
    double T3 = T2 * T;

    a0 = s0;
    a1 = v0;
    a2 = (3 * (s1 - s0) - (2 * v0 + v1) * T) / T2;
    a3 = (-2 * (s1 - s0) + (v0 + v1) * T) / T3;
}

AxisState CubicPoly::get_state(double t) const
{
    t = std::clamp(t, 0.0, t_total);
    double t2 = t * t;
    double t3 = t2 * t;

    AxisState st;
    st.pos = a0 + a1 * t + a2 * t2 + a3 * t3;
    st.vel = a1 + 2 * a2 * t + 3 * a3 * t2;
    st.acc = 2 * a2 + 6 * a3 * t;
    st.jerk = 6 * a3;
    return st;
}

// ===================== 五次多项式 =====================
void QuinticPoly::init(double s0, double s1, double v0, double v1, double a0_, double a1_, double t)
{
    t_total = t;
    double T = t;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    a0 = s0;
    a1 = v0;
    a2 = a0_ / 2.0;

    a3 = (20 * (s1 - s0) - (8 * v1 + 12 * v0) * T - (3 * a1_ - a0_) * T2) / (2 * T3);
    a4 = (-30 * (s1 - s0) + (14 * v1 + 16 * v0) * T + (3 * a1_ - 2 * a0_) * T2) / (2 * T4);
    a5 = (12 * (s1 - s0) - (6 * v1 + 6 * v0) * T - (a1_ - a0_) * T2) / (2 * T5);
}

AxisState QuinticPoly::get_state(double t) const
{
    t = std::clamp(t, 0.0, t_total);
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    AxisState st;
    st.pos = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
    st.vel = a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4;
    st.acc = 2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3;
    st.jerk = 6*a3 + 24*a4*t + 60*a5*t2;
    return st;
}

// ===================== 3-4-5 分段多项式 =====================
void Poly345::init(double s0, double s1, double t)
{
    t_total = t;
    s_start = s0;
    s_end   = s1;
}

AxisState Poly345::get_state(double t) const
{
    t = std::clamp(t, 0.0, t_total);
    double tau = t / t_total;
    double ds = s_end - s_start;
    AxisState st;

    // 3-4-5多项式标准形式
    st.pos = s_start + ds * (10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5));
    st.vel = ds * (30*pow(tau,2) - 60*pow(tau,3) + 30*pow(tau,4)) / t_total;
    st.acc = ds * (60*tau - 180*pow(tau,2) + 120*pow(tau,3)) / (t_total*t_total);
    st.jerk = ds * (60 - 360*tau + 360*pow(tau,2)) / (t_total*t_total*t_total);
    return st;
}

// ===================== S型曲线 =====================
void SCurve::init(double s0, double s1, double vmax, double amax, double jmax)
{
    v_max = vmax;
    a_max = amax;
    j_max = jmax;
    s_total = fabs(s1 - s0);
    if (s_total < EPS)
    {
        t_total = 0;
        return;
    }

    // 防止除零
    if(j_max < EPS)
    {
        // 退化为梯形速度曲线
        double t_a = v_max / a_max;
        double s_a = 0.5 * a_max * t_a * t_a;
        if(2 * s_a >= s_total)
        {
            // 三角形速度曲线
            t_acc = sqrt(s_total / a_max);
            t_const = 0;
        }
        else
        {
            t_acc = t_a;
            t_const = (s_total - 2 * s_a) / v_max;
        }
        t_dec = t_acc;
        t_total = t_acc + t_const + t_dec;
        return;
    }

    double t_j = a_max / j_max;
    double s_half = 0.5 * a_max * t_j * t_j;
    // 加速段+减速段总位移 = 4 * s_half (每个半段加速包含s_half位移)
    if (4 * s_half >= s_total)
    {
        // 未达到最大加速度即开始减速
        t_acc = sqrt(s_total / a_max);
        t_const = 0;
    }
    else
    {
        t_acc = 2 * t_j;
        double s_acc = 2 * s_half;
        double s_const = s_total - 2 * s_acc;
        t_const = s_const / v_max;
    }
    t_dec = t_acc;
    t_total = t_acc + t_const + t_dec;
}

AxisState SCurve::get_state(double t) const
{
    t = std::clamp(t, 0.0, t_total);
    AxisState st;

    if (t < t_acc)
    {
        st.acc = a_max;
        st.vel = a_max * t;
        st.pos = 0.5 * a_max * t * t;
        st.jerk = j_max;
    }
    else if (t < t_acc + t_const)
    {
        st.acc = 0;
        st.jerk = 0;
        st.vel = v_max;
        st.pos = 0.5 * a_max * t_acc * t_acc + v_max * (t - t_acc);
    }
    else
    {
        double td = t_total - t;
        st.acc = -a_max;
        st.vel = a_max * td;
        st.pos = s_total - 0.5 * a_max * td * td;
        st.jerk = -j_max;
    }
    return st;
}

// ===================== 直线插补 =====================
void LineInterp::init(const std::vector<double>& s, const std::vector<double>& e, double t)
{
    start = s;
    end = e;
    t_total = t;
}

std::vector<double> LineInterp::get_pos(double t) const
{
    double r = std::clamp(t / t_total, 0.0, 1.0);
    std::vector<double> res;
    for (size_t i = 0; i < start.size(); ++i)
        res.push_back(start[i] + (end[i] - start[i]) * r);
    return res;
}

// ===================== 圆弧插补 =====================
void CircleInterp::init(const std::vector<double>& s, const std::vector<double>& c, double r, double ang, double t, bool cw)
{
    start = s;
    center = c;
    radius = r;
    angle_total = ang;
    t_total = t;
    clockwise = cw;
}

std::vector<double> CircleInterp::get_pos(double t) const
{
    double r = std::clamp(t / t_total, 0.0, 1.0);
    double ang = angle_total * (clockwise ? -1.0 : 1.0) * r;

    double x0 = start[0] - center[0];
    double y0 = start[1] - center[1];
    double x = x0 * cos(ang) - y0 * sin(ang);
    double y = x0 * sin(ang) + y0 * cos(ang);

    std::vector<double> res = center;
    res[0] += x;
    res[1] += y;
    for (size_t i = 2; i < start.size(); ++i)
        res[i] = start[i];
    return res;
}

// ===================== 三阶B样条 =====================
double BSpline3::base(int i, int k, double u) const
{
    if (k == 1)
        return (u >= knot[i] && u < knot[i + 1]) ? 1.0 : 0.0;

    double d1 = knot[i + k - 1] - knot[i];
    double d2 = knot[i + k] - knot[i + 1];
    double b1 = (d1 > EPS) ? (u - knot[i]) / d1 * base(i, k - 1, u) : 0.0;
    double b2 = (d2 > EPS) ? (knot[i + k] - u) / d2 * base(i + 1, k - 1, u) : 0.0;
    return b1 + b2;
}

void BSpline3::init(const std::vector<std::vector<double>>& ctrl, double t)
{
    ctrl_pts = ctrl;
    dim = ctrl[0].size();
    t_total = t;
    int n = ctrl.size();
    int p = 3;
    knot.resize(n + p + 1, 0.0);

    for (int i = 0; i <= p; ++i) knot[i] = 0.0;
    for (int i = p+1; i < n; ++i) knot[i] = (double)(i - p) / (n - p);
    for (int i = n; i <= n+p; ++i) knot[i] = 1.0;
}

std::vector<double> BSpline3::get_pos(double t) const
{
    double u = std::clamp(t / t_total, 0.0, 1.0);
    std::vector<double> pt(dim, 0.0);
    int n = ctrl_pts.size();
    int p = 3;

    for (int i = 0; i < n; ++i)
    {
        double b = base(i, p+1, u);
        for (int d = 0; d < dim; ++d)
            pt[d] += ctrl_pts[i] * b;
    }
    return pt;
}