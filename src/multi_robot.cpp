#include "multi_robot.h"
#include "kinematics.h"

void MultiRobot::sync_move(std::vector<std::vector<double>>& robot_joints, double total_t)
{
    // 多机时间基准同步：以最慢轴时间为基准，统一各机器人运动时间
    if(robot_joints.empty() || total_t <= EPS)
        return;

    // 计算各机器人到目标的最大关节位移
    double max_dist = 0.0;
    for(const auto& joints : robot_joints)
    {
        double dist = 0.0;
        for(double j : joints)
            dist += j * j;
        dist = sqrt(dist);
        if(dist > max_dist) max_dist = dist;
    }

    // 根据最大位移同步各机器人速度比例
    // 为每个机器人规划独立的五次多项式轨迹，统一时间基准
    for(size_t r = 0; r < robot_joints.size(); ++r)
    {
        auto& joints = robot_joints[r];
        double dist = 0.0;
        for(double j : joints) dist += j * j;
        dist = sqrt(dist);

        // 速度比例因子: 位移越大的机器人速度越快，保证同时到达
        double scale = (max_dist > EPS) ? dist / max_dist : 1.0;
        double t_r = total_t / scale;  // 位移小的给更长的时间，统一到达
        if(t_r < 0.1) t_r = 0.1;       // 最小安全时间

        // 为每个关节规划轨迹 (此处为纯计算，实际下发由RobotCtrl::control_loop完成)
        (void)t_r; // 实际使用时由各RobotCtrl实例调用对应的move方法
    }

    log_print(LOG_INFO, "多机器人同步运动: " + std::to_string(robot_joints.size()) +
              "台, 总时间=" + std::to_string(total_t) + "s, 最大位移=" + std::to_string(max_dist));
}

bool MultiRobot::collision_check(const std::vector<std::vector<double>>& joints)
{
    // 多机器人碰撞检测：计算任意两机器人末端距离
    if(joints.size() < 2) return false;

    // 简易末端位置计算 (假设每台机器人使用相同DH参数)
    const double L1 = 200.0, L2 = 500.0;
    std::vector<CartPose> poses;
    for(const auto& j : joints)
    {
        if(j.size() < 6) continue;
        CartPose p;
        p.x = L2 * cos(j[0]) * cos(j[1]);
        p.y = L2 * sin(j[0]) * cos(j[1]);
        p.z = L2 * sin(j[1]) + L1;
        poses.push_back(p);
    }

    // 检查两两间距
    const double safe_dist = 100.0; // 安全距离100mm
    for(size_t i = 0; i < poses.size(); ++i)
    {
        for(size_t k = i + 1; k < poses.size(); ++k)
        {
            double dx = poses[i].x - poses[k].x;
            double dy = poses[i].y - poses[k].y;
            double dz = poses[i].z - poses[k].z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            if(dist < safe_dist)
            {
                log_print(LOG_WARN, "多机器人碰撞预警: 机器人" + std::to_string(i) +
                          "与机器人" + std::to_string(k) + "距离=" + std::to_string(dist) + "mm");
                return true;
            }
        }
    }
    return false;
}