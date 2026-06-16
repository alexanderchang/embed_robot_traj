#ifndef MULTI_ROBOT_H
#define MULTI_ROBOT_H

#include <vector>
#include "common.h"
#include "trajectory.h"

// 多机器人协同同步控制
class MultiRobot
{
public:
    // 多机同步点位运动
    void sync_move(std::vector<std::vector<double>>& robot_joints, double total_t);
    // 多机互斥防碰撞检测
    bool collision_check(const std::vector<std::vector<double>>& joints);
};

#endif // MULTI_ROBOT_H