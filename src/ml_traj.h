#ifndef ML_TRAJ_H
#define ML_TRAJ_H

#include <vector>
#include <random>
#include <numeric>
#include "trajectory.h"
#include "common.h"

// 强化学习状态
struct RLState
{
    std::vector<double> cur_pos;
    std::vector<double> tar_pos;
    RLState() = default;
    RLState(std::vector<double> c, std::vector<double> t)
        : cur_pos(c), tar_pos(t) {}
};

// 1. Q-Learning 强化学习
class QLearnTraj
{
private:
    std::vector<double> act_space;
    std::vector<std::pair<double, double>> q_table;
    double alpha, gamma, epsilon;
    int train_epoch;

    int state_encode(const RLState& s) const;
    double calc_reward(const RLState& s, double act);
    double select_action(int state_id);
public:
    void init(const std::vector<double>& acts, double lr, double g, double e, int ep);
    void train(const RLState& s);
    double predict(const RLState& s);
    QuinticPoly get_traj(const RLState& s);
};

// 2. 监督学习（线性拟合）
class SuperTraj
{
private:
    std::vector<std::vector<double>> feat_x;
    std::vector<double> label_y;
    double k, b;
public:
    void load_sample(const std::vector<std::vector<double>>& x, const std::vector<double>& y);
    void train();
    double predict(const std::vector<double>& cur, const std::vector<double>& tar);
    QuinticPoly get_traj(const std::vector<double>& cur, const std::vector<double>& tar);
};

#endif // ML_TRAJ_H