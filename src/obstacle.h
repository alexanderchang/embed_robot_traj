#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include "kinematics.h"
#include "common.h"

// 球形障碍物（工业常用简化模型）
struct Obstacle
{
    double x, y, z;
    double radius;
    Obstacle() : x(0), y(0), z(0), radius(0) {}
};

// 虚拟3D禁区类型
enum ZoneType
{
    ZONE_BOX,       // 长方体禁区 (x/y/z 范围)
    ZONE_CYLINDER,  // 圆柱禁区 (中心轴+z, 半径+高度)
    ZONE_SPHERE     // 球禁区 (中心+半径)
};

// 虚拟3D禁区定义 (不可运行空间)
struct ForbiddenZone
{
    ZoneType type;
    double min_x, max_x;   // 长方体X范围
    double min_y, max_y;   // 长方体Y范围
    double min_z, max_z;   // 长方体Z范围 / 圆柱Z范围
    double center_x, center_y, center_z; // 球心/圆柱轴心
    double radius;         // 球半径 / 圆柱半径
    double height;         // 圆柱高度

    ForbiddenZone()
        : type(ZONE_BOX)
        , min_x(0), max_x(0), min_y(0), max_y(0), min_z(0), max_z(0)
        , center_x(0), center_y(0), center_z(0)
        , radius(0), height(0) {}

    // 工厂方法：创建长方体禁区
    static ForbiddenZone box(double x1, double x2, double y1, double y2, double z1, double z2)
    {
        ForbiddenZone z;
        z.type = ZONE_BOX;
        z.min_x = std::min(x1, x2); z.max_x = std::max(x1, x2);
        z.min_y = std::min(y1, y2); z.max_y = std::max(y1, y2);
        z.min_z = std::min(z1, z2); z.max_z = std::max(z1, z2);
        return z;
    }

    // 工厂方法：创建圆柱禁区
    static ForbiddenZone cylinder(double cx, double cy, double z1, double z2, double r)
    {
        ForbiddenZone z;
        z.type = ZONE_CYLINDER;
        z.center_x = cx; z.center_y = cy;
        z.min_z = std::min(z1, z2); z.max_z = std::max(z1, z2);
        z.radius = r; z.height = std::fabs(z2 - z1);
        return z;
    }

    // 工厂方法：创建球禁区
    static ForbiddenZone sphere(double cx, double cy, double cz, double r)
    {
        ForbiddenZone z;
        z.type = ZONE_SPHERE;
        z.center_x = cx; z.center_y = cy; z.center_z = cz;
        z.radius = r;
        return z;
    }
};

// 禁区避碰 — 虚拟3D空间不可运行，贴边跳过
class ZoneAvoid
{
public:
    // 检测笛卡尔位姿是否落入禁区内部
    bool check_zone_collision(const CartPose& pose, const std::vector<ForbiddenZone>& zones) const;

    // 计算点到禁区的最近边界安全点 (贴边跳过)
    // 返回 true 表示姿态被修正（落入禁区内部）
    bool slide_along_boundary(const CartPose& pose_in, const std::vector<ForbiddenZone>& zones,
                              CartPose& pose_out, double safety_margin = 5.0) const;

private:
    // 点是否在长方体禁区内部
    bool inside_box(const CartPose& p, const ForbiddenZone& z, double margin = 0.0) const;

    // 点是否在圆柱禁区内部
    bool inside_cylinder(const CartPose& p, const ForbiddenZone& z, double margin = 0.0) const;

    // 点是否在球禁区内部
    bool inside_sphere(const CartPose& p, const ForbiddenZone& z, double margin = 0.0) const;

    // 将落入长方体禁区的点推到最近边界外侧
    void push_out_box(const CartPose& p_in, const ForbiddenZone& z, double margin, CartPose& p_out) const;

    // 将落入圆柱禁区的点推到最近边界外侧
    void push_out_cylinder(const CartPose& p_in, const ForbiddenZone& z, double margin, CartPose& p_out) const;

    // 将落入球禁区的点推到最近边界外侧
    void push_out_sphere(const CartPose& p_in, const ForbiddenZone& z, double margin, CartPose& p_out) const;
};

// RRT* 动态避障 + 路径重规划
class DynAvoid
{
public:
    bool check_collision(const CartPose& pose, const std::vector<Obstacle>& obs);
    bool replan_path(const CartPose& start, const CartPose& tar,
                     const std::vector<Obstacle>& obs,
                     std::vector<CartPose>& new_path);
};

#endif // OBSTACLE_H