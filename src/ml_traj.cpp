#include "ml_traj.h"

// ===================== Q-Learning =====================
void QLearnTraj::init(const std::vector<double>& acts, double lr, double g, double e, int ep)
{
    act_space = acts;
    alpha = lr;
    gamma = g;
    epsilon = e;
    train_epoch = ep;
    q_table.clear();
    for (double a : act_space)
        q_table.emplace_back(a, 0.0);
}

int QLearnTraj::state_encode(const RLState& s) const
{
    double dist = 0.0;
    for (size_t i = 0; i < s.cur_pos.size(); ++i)
        dist += fabs(s.tar_pos[i] - s.cur_pos[i]);
    return static_cast<int>(dist * 10) % 100;
}

double QLearnTraj::calc_reward(const RLState& s, double act)
{
    double j_sum = 0.0;
    for (size_t i = 0; i < s.cur_pos.size(); ++i)
    {
        QuinticPoly traj;
        traj.init(s.cur_pos[i], s.tar_pos[i], 0,0,0,0, act);
        for (double t = 0; t <= act; t += act/100.0)
            j_sum += fabs(traj.get_state(t).jerk);
    }
    return -(act + 0.8 * j_sum);
}

double QLearnTraj::select_action(int state_id)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    if (dis(gen) < epsilon)
    {
        std::uniform_int_distribution<> r(0, (int)act_space.size()-1);
        return act_space[r(gen)];
    }

    double max_q = -1e9;
    double best_act = act_space[0];
    for (auto& p : q_table)
    {
        if (p.second > max_q)
        {
            max_q = p.second;
            best_act = p.first;
        }
    }
    return best_act;
}

void QLearnTraj::train(const RLState& s)
{
    int sid = state_encode(s);
    for (int e = 0; e < train_epoch; ++e)
    {
        double act = select_action(sid);
        double r = calc_reward(s, act);
        for (auto& p : q_table)
        {
            if (fabs(p.first - act) < EPS)
            {
                p.second += alpha * (r + gamma * 0 - p.second);
                break;
            }
        }
    }
}

double QLearnTraj::predict(const RLState& s)
{
    int sid = state_encode(s);
    double max_q = -1e9;
    double best = act_space[0];
    for (auto& p : q_table)
    {
        if (p.second > max_q)
        {
            max_q = p.second;
            best = p.first;
        }
    }
    return best;
}

QuinticPoly QLearnTraj::get_traj(const RLState& s)
{
    double t = predict(s);
    QuinticPoly traj;
    // 取位移最大的轴作为基准轨迹 (多轴同步以最慢轴为准)
    double max_dist = 0.0;
    size_t max_idx = 0;
    for (size_t i = 0; i < s.cur_pos.size(); ++i)
    {
        double d = fabs(s.tar_pos[i] - s.cur_pos[i]);
        if(d > max_dist) { max_dist = d; max_idx = i; }
    }
    traj.init(s.cur_pos[max_idx], s.tar_pos[max_idx], 0, 0, 0, 0, t);
    return traj;
}

// ===================== 监督学习 =====================
void SuperTraj::load_sample(const std::vector<std::vector<double>>& x, const std::vector<double>& y)
{
    feat_x = x;
    label_y = y;
}

void SuperTraj::train()
{
    int n = feat_x.size();
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (int i = 0; i < n; ++i)
    {
        double dx = std::accumulate(feat_x[i].begin(), feat_x[i].end(), 0.0);
        sum_x += dx;
        sum_y += label_y[i];
        sum_xy += dx * label_y[i];
        sum_x2 += dx * dx;
    }
    k = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    b = (sum_y - k * sum_x) / n;
}

double SuperTraj::predict(const std::vector<double>& cur, const std::vector<double>& tar)
{
    double dx = 0.0;
    for (size_t i = 0; i < cur.size(); ++i)
        dx += fabs(tar[i] - cur[i]);
    double res = k * dx + b;
    return std::max(0.1, res);
}

QuinticPoly SuperTraj::get_traj(const std::vector<double>& cur, const std::vector<double>& tar)
{
    double t = predict(cur, tar);
    QuinticPoly traj;
    // 取位移最大的轴作为基准轨迹
    double max_dist = 0.0;
    size_t max_idx = 0;
    for (size_t i = 0; i < cur.size(); ++i)
    {
        double d = fabs(tar[i] - cur[i]);
        if(d > max_dist) { max_dist = d; max_idx = i; }
    }
    traj.init(cur[max_idx], tar[max_idx], 0, 0, 0, 0, t);
    return traj;
}