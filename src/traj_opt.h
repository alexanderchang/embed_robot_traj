#ifndef TRAJ_OPT_H
#define TRAJ_OPT_H

#include <vector>
#include <boost/math/tools/minima.hpp>
#include "trajectory.h"
#include "common.h"

// 电机动力学参数（能耗计算）
struct DynParam
{
    double J;    // 转动惯量
    double B;    // 阻尼系数
    double Kt;   // 力矩系数
    double R;    // 绕组电阻
};

// 1. 时间最优轨迹
class TimeOpt
{
private:
    std::vector<double> q0, q1;
    std::vector<double> vlim, alim;
    double cost(double T) const;
public:
    void set_data(const std::vector<double>& s, const std::vector<double>& e,
                  const std::vector<double>& vmax, const std::vector<double>& amax);
    double solve();
    QuinticPoly get_traj(int axis, double T);
};

// 2. 冲击(Jerk)最优
class JerkOpt
{
private:
    std::vector<double> q0, q1;
    double cost(double T) const;
public:
    void set_data(const std::vector<double>& s, const std::vector<double>& e);
    double solve();
};

// 3. 能耗最优
class EnergyOpt
{
private:
    std::vector<double> q0, q1;
    std::vector<DynParam> dyn;
    double cost(double T) const;
public:
    void set_dyn(const std::vector<DynParam>& p);
    void set_data(const std::vector<double>& s, const std::vector<double>& e);
    double solve();
};

// 4. 多目标加权优化（时间+冲击+能耗）
class MultiOpt
{
private:
    TimeOpt t_opt;
    JerkOpt j_opt;
    EnergyOpt e_opt;
    double wt, wj, we;
    double total_cost(double T) const;
public:
    void set_weight(double t, double j, double e);
    void set_all(const std::vector<double>& s, const std::vector<double>& e,
                 const std::vector<double>& v, const std::vector<double>& a,
                 const std::vector<DynParam>& dyn);
    double solve();
};

#endif // TRAJ_OPT_H