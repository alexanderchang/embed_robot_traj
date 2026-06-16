#include "traj_opt.h"

// ===================== 时间最优 =====================
void TimeOpt::set_data(const std::vector<double>& s, const std::vector<double>& e,
                       const std::vector<double>& vmax, const std::vector<double>& amax)
{
    q0 = s;
    q1 = e;
    vlim = vmax;
    alim = amax;
}

double TimeOpt::cost(double T) const
{
    double penalty = 0.0;
    for (size_t i = 0; i < q0.size(); ++i)
    {
        QuinticPoly traj;
        traj.init(q0[i], q1[i], 0, 0, 0, 0, T);
        double v_peak = 0.0, a_peak = 0.0;

        for (double t = 0; t <= T; t += T / 100.0)
        {
            AxisState st = traj.get_state(t);
            v_peak = std::max(v_peak, fabs(st.vel));
            a_peak = std::max(a_peak, fabs(st.acc));
        }
        if (v_peak > vlim[i]) penalty += (v_peak - vlim[i]) * 1000.0;
        if (a_peak > alim[i]) penalty += (a_peak - alim[i]) * 1000.0;
    }
    return T + penalty;
}

double TimeOpt::solve()
{
    using namespace boost::math::tools;
    auto f = [this](double T) { return cost(T); };
    uintmax_t iter = 200;
    auto res = brent_find_minima(f, 0.01, 10.0, iter);
    return res.first;
}

QuinticPoly TimeOpt::get_traj(int axis, double T)
{
    QuinticPoly traj;
    traj.init(q0[axis], q1[axis], 0,0,0,0,T);
    return traj;
}

// ===================== 冲击最优 =====================
void JerkOpt::set_data(const std::vector<double>& s, const std::vector<double>& e)
{
    q0 = s;
    q1 = e;
}

double JerkOpt::cost(double T) const
{
    double total_jerk = 0.0;
    for (size_t i = 0; i < q0.size(); ++i)
    {
        QuinticPoly traj;
        traj.init(q0[i], q1[i], 0,0,0,0,T);
        double j_sum = 0.0;
        for (double t = 0; t <= T; t += T / 200.0)
        {
            AxisState st = traj.get_state(t);
            j_sum += st.jerk * st.jerk;
        }
        total_jerk += j_sum;
    }
    return total_jerk;
}

double JerkOpt::solve()
{
    using namespace boost::math::tools;
    auto f = [this](double T) { return cost(T); };
    uintmax_t iter = 200;
    auto res = brent_find_minima(f, 0.01, 8.0, iter);
    return res.first;
}

// ===================== 能耗最优 =====================
void EnergyOpt::set_dyn(const std::vector<DynParam>& p)
{
    dyn = p;
}

void EnergyOpt::set_data(const std::vector<double>& s, const std::vector<double>& e)
{
    q0 = s;
    q1 = e;
}

double EnergyOpt::cost(double T) const
{
    double total_energy = 0.0;
    double dt = T / 200.0;
    for (size_t i = 0; i < q0.size(); ++i)
    {
        QuinticPoly traj;
        traj.init(q0[i], q1[i], 0,0,0,0,T);
        DynParam p = dyn[i];
        double e = 0.0;

        for (double t = 0; t <= T; t += dt)
        {
            AxisState st = traj.get_state(t);
            double tau = p.J * st.acc + p.B * st.vel;
            double I = tau / p.Kt;
            e += I * I * p.R * dt;
        }
        total_energy += e;
    }
    return total_energy;
}

double EnergyOpt::solve()
{
    using namespace boost::math::tools;
    auto f = [this](double T) { return cost(T); };
    uintmax_t iter = 200;
    auto res = brent_find_minima(f, 0.01, 10.0, iter);
    return res.first;
}

// ===================== 多目标优化 =====================
void MultiOpt::set_weight(double t, double j, double e)
{
    wt = t;
    wj = j;
    we = e;
}

void MultiOpt::set_all(const std::vector<double>& s, const std::vector<double>& e,
                       const std::vector<double>& v, const std::vector<double>& a,
                       const std::vector<DynParam>& dyn)
{
    t_opt.set_data(s, e, v, a);
    j_opt.set_data(s, e);
    e_opt.set_data(s, e);
    e_opt.set_dyn(dyn);
}

double MultiOpt::total_cost(double T) const
{
    return wt * t_opt.cost(T) + wj * j_opt.cost(T) + we * e_opt.cost(T);
}

double MultiOpt::solve()
{
    using namespace boost::math::tools;
    auto f = [this](double T) { return total_cost(T); };
    uintmax_t iter = 300;
    auto res = brent_find_minima(f, 0.01, 12.0, iter);
    return res.first;
}