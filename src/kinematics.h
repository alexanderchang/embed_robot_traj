#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include "common.h"

// 笛卡尔位姿 (X/Y/Z mm + 欧拉角 rad)
struct CartPose
{
    double x, y, z;
    double rx, ry, rz;
    CartPose() : x(0), y(0), z(0), rx(0), ry(0), rz(0) {}
};

// D-H参数
struct DHParam
{
    double a, alpha, d, theta;
    DHParam() : a(0), alpha(0), d(0), theta(0) {}
};

// 正运动学：关节角 → 笛卡尔位姿
CartPose forward_kin(const std::vector<double>& joint, const std::vector<DHParam>& dh);

// 逆运动学：笛卡尔位姿 → 关节角 (返回false=奇异点/无解)
bool inverse_kin(const CartPose& cart, const std::vector<DHParam>& dh, std::vector<double>& joint_out);

// 奇异点检测
bool is_singularity(const std::vector<double>& joint, const std::vector<DHParam>& dh);

#endif // KINEMATICS_H