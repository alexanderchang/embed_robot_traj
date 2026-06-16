#include "obstacle.h"
#include <random>
#include <cmath>

// ===================== ZoneAvoid 实现 =====================

bool ZoneAvoid::check_zone_collision(const CartPose& pose, const std::vector<ForbiddenZone>& zones) const
{
    for (const auto& z : zones)
    {
        switch (z.type)
        {
        case ZONE_BOX:
            if (inside_box(pose, z)) return true;
            break;
        case ZONE_CYLINDER:
            if (inside_cylinder(pose, z)) return true;
            break;
        case ZONE_SPHERE:
            if (inside_sphere(pose, z)) return true;
            break;
        }
    }
    return false;
}

bool ZoneAvoid::slide_along_boundary(const CartPose& pose_in, const std::vector<ForbiddenZone>& zones,
                                      CartPose& pose_out, double safety_margin) const
{
    pose_out = pose_in;
    bool modified = false;

    for (const auto& z : zones)
    {
        switch (z.type)
        {
        case ZONE_BOX:
            if (inside_box(pose_out, z))
            {
                push_out_box(pose_out, z, safety_margin, pose_out);
                modified = true;
            }
            break;
        case ZONE_CYLINDER:
            if (inside_cylinder(pose_out, z))
            {
                push_out_cylinder(pose_out, z, safety_margin, pose_out);
                modified = true;
            }
            break;
        case ZONE_SPHERE:
            if (inside_sphere(pose_out, z))
            {
                push_out_sphere(pose_out, z, safety_margin, pose_out);
                modified = true;
            }
            break;
        }
    }
    return modified;
}

// --- 长方体禁区 ---

bool ZoneAvoid::inside_box(const CartPose& p, const ForbiddenZone& z, double margin) const
{
    return (p.x >= z.min_x - margin && p.x <= z.max_x + margin &&
            p.y >= z.min_y - margin && p.y <= z.max_y + margin &&
            p.z >= z.min_z - margin && p.z <= z.max_z + margin);
}

void ZoneAvoid::push_out_box(const CartPose& p_in, const ForbiddenZone& z, double margin, CartPose& p_out) const
{
    // 计算点到长方体各面的距离，沿最近面法向推出到边界 + margin 外侧
    double dx_low = z.min_x - margin - p_in.x;  // 推离下表面的偏移 (负值 = 需要往外)
    double dx_high = p_in.x - z.max_x - margin;  // 推离上表面的偏移 (负值 = 需要往外)
    double dy_low = z.min_y - margin - p_in.y;
    double dy_high = p_in.y - z.max_y - margin;
    double dz_low = z.min_z - margin - p_in.z;
    double dz_high = p_in.z - z.max_z - margin;

    // 找出需要推出最多的轴和方向（最浅穿入的边界）
    double push_x = 0, push_y = 0, push_z = 0;
    bool push_on_x = false, push_on_y = false, push_on_z = false;

    if (dx_low < 0) { push_x = dx_low; push_on_x = true; }
    if (dx_high > 0 && dx_high < std::fabs(push_x)) { push_x = -dx_high; push_on_x = true; }

    if (dy_low < 0) { push_y = dy_low; push_on_y = true; }
    if (dy_high > 0 && dy_high < std::fabs(push_y)) { push_y = -dy_high; push_on_y = true; }

    if (dz_low < 0) { push_z = dz_low; push_on_z = true; }
    if (dz_high > 0 && dz_high < std::fabs(push_z)) { push_z = -dz_high; push_on_z = true; }

    // 选最小推离距离的轴（贴最近边跳过）
    double ax = std::fabs(push_x);
    double ay = std::fabs(push_y);
    double az = std::fabs(push_z);

    if (push_on_x && ax <= ay && ax <= az)
    {
        p_out.x += push_x;
        // 沿X推出后，确保Y/Z在合法边界内
        p_out.y = clamp_val(p_out.y, z.min_y + margin, z.max_y - margin);
        p_out.z = clamp_val(p_out.z, z.min_z + margin, z.max_z - margin);
    }
    else if (push_on_y && ay <= ax && ay <= az)
    {
        p_out.y += push_y;
        p_out.x = clamp_val(p_out.x, z.min_x + margin, z.max_x - margin);
        p_out.z = clamp_val(p_out.z, z.min_z + margin, z.max_z - margin);
    }
    else if (push_on_z)
    {
        p_out.z += push_z;
        p_out.x = clamp_val(p_out.x, z.min_x + margin, z.max_x - margin);
        p_out.y = clamp_val(p_out.y, z.min_y + margin, z.max_y - margin);
    }
}

// --- 圆柱禁区 ---

bool ZoneAvoid::inside_cylinder(const CartPose& p, const ForbiddenZone& z, double margin) const
{
    if (p.z < z.min_z - margin || p.z > z.max_z + margin)
        return false;
    double dx = p.x - z.center_x;
    double dy = p.y - z.center_y;
    double dist_sq = dx * dx + dy * dy;
    double r_eff = z.radius + margin;
    return dist_sq < r_eff * r_eff;
}

void ZoneAvoid::push_out_cylinder(const CartPose& p_in, const ForbiddenZone& z, double margin, CartPose& p_out) const
{
    // 圆柱禁区内：径向推出到圆柱表面外
    double dx = p_in.x - z.center_x;
    double dy = p_in.y - z.center_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double r_safe = z.radius + margin;

    if (dist < EPS)
    {
        // 点在轴心线上，沿X方向推出
        p_out.x = z.center_x + r_safe;
        p_out.y = z.center_y;
    }
    else
    {
        double scale = r_safe / dist;
        p_out.x = z.center_x + dx * scale;
        p_out.y = z.center_y + dy * scale;
    }

    // Z方向也确保在边界外
    if (p_in.z < z.min_z + margin)
        p_out.z = z.min_z - margin;
    else if (p_in.z > z.max_z - margin)
        p_out.z = z.max_z + margin;
    else
        p_out.z = p_in.z;  // Z在范围内保持，仅径向推出
}

// --- 球禁区 ---

bool ZoneAvoid::inside_sphere(const CartPose& p, const ForbiddenZone& z, double margin) const
{
    double dx = p.x - z.center_x;
    double dy = p.y - z.center_y;
    double dz = p.z - z.center_z;
    double dist_sq = dx * dx + dy * dy + dz * dz;
    double r_eff = z.radius + margin;
    return dist_sq < r_eff * r_eff;
}

void ZoneAvoid::push_out_sphere(const CartPose& p_in, const ForbiddenZone& z, double margin, CartPose& p_out) const
{
    double dx = p_in.x - z.center_x;
    double dy = p_in.y - z.center_y;
    double dz = p_in.z - z.center_z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double r_safe = z.radius + margin;

    if (dist < EPS)
    {
        // 点在球心，沿X方向推出
        p_out.x = z.center_x + r_safe;
        p_out.y = z.center_y;
        p_out.z = z.center_z;
    }
    else
    {
        double scale = r_safe / dist;
        p_out.x = z.center_x + dx * scale;
        p_out.y = z.center_y + dy * scale;
        p_out.z = z.center_z + dz * scale;
    }
}

// ===================== DynAvoid 实现 =====================

bool DynAvoid::check_collision(const CartPose& pose, const std::vector<Obstacle>& obs)
{
    for (const auto& o : obs)
    {
        double dx = pose.x - o.x;
        double dy = pose.y - o.y;
        double dz = pose.z - o.z;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < o.radius + 20.0) // 安全余量20mm
            return true;
    }
    return false;
}

bool DynAvoid::replan_path(const CartPose& start, const CartPose& tar,
                           const std::vector<Obstacle>& obs,
                           std::vector<CartPose>& new_path)
{
    new_path.clear();

    // 简易RRT*: 直线采样 + 碰撞检测 + 绕行策略
    int seg = 20;
    bool collision_found = false;

    for (int i = 0; i <= seg; ++i)
    {
        double r = (double)i / seg;
        CartPose p;
        p.x = start.x + (tar.x - start.x) * r;
        p.y = start.y + (tar.y - start.y) * r;
        p.z = start.z + (tar.z - start.z) * r;
        p.rx = start.rx + (tar.rx - start.rx) * r;
        p.ry = start.ry + (tar.ry - start.ry) * r;
        p.rz = start.rz + (tar.rz - start.rz) * r;

        if (check_collision(p, obs))
        {
            collision_found = true;
            // 尝试绕行: 对每个障碍物，在垂直方向偏移避让
            for(const auto& o : obs)
            {
                double dx = p.x - o.x;
                double dy = p.y - o.y;
                double dz = p.z - o.z;
                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if(dist < o.radius + 20.0 && dist > EPS)
                {
                    // 沿障碍物表面外推
                    double safe_dist = o.radius + 25.0;
                    double scale = safe_dist / dist;
                    p.x = o.x + dx * scale;
                    p.y = o.y + dy * scale;
                    p.z = o.z + dz * scale;
                }
            }
        }
        new_path.push_back(p);
    }
    return !collision_found;  // true表示路径无碰撞
}