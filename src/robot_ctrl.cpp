#include "robot_ctrl.h"
#include <iostream>

RobotCtrl::RobotCtrl()
    : axis_num(0), thread_run(false), cur_mode(MODE_QUINTIC)
    , zone_check_enabled(true), zone_safety_margin(5.0)
{}

RobotCtrl::~RobotCtrl()
{
    stop();
}

bool RobotCtrl::init(const std::string& eth, int axis)
{
    axis_num = axis;
    cur_pos.resize(axis, 0.0);
    if (!ec_dev.init(eth, axis))
        return false;
    ec_dev.dc_start();

    // 全部轴使能 + 读取初始位置
    for (int i = 0; i < axis_num; ++i)
    {
        if (!ec_dev.servo_enable(i))
            return false;
        int32_t pulse = ec_dev.get_act_pulse(i);
        cur_pos[i] = pulse * PULSE_TO_RAD;
    }
    log_print(LOG_INFO, "机器人总控初始化完成");
    return true;
}

void RobotCtrl::start()
{
    if (!thread_run)
        ctrl_thread = boost::thread(&RobotCtrl::control_loop, this);
}

void RobotCtrl::stop()
{
    thread_run = false;
    if (ctrl_thread.joinable())
        ctrl_thread.join();
    ec_dev.close();
}

void RobotCtrl::control_loop()
{
    using namespace boost::chrono;
    thread_run = true;
    auto base_time = steady_clock::now();
    const microseconds period(EC_PERIOD_US);

    while (thread_run)
    {
        auto loop_start = steady_clock::now();
        double t = duration_cast<duration<double>>(loop_start - base_time).count();
        std::vector<double> cmd(axis_num, 0.0);

        // 故障检测
        for (int i = 0; i < axis_num; ++i)
        {
            if (ec_dev.check_fault(i))
                ec_dev.servo_fault_reset(i);
        }

        // 轨迹分支
        switch (cur_mode)
        {
            case MODE_CUBIC:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = cubic.get_state(t).pos;
                break;
            case MODE_QUINTIC:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = quintic.get_state(t).pos;
                break;
            case MODE_POLY345:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = poly345.get_state(t).pos;
                break;
            case MODE_SCURVE:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = scurve.get_state(t).pos;
                break;
            case MODE_LINE:
                cmd = line.get_pos(t);
                break;
            case MODE_CIRCLE:
                cmd = circle.get_pos(t);
                break;
            case MODE_BSPLINE:
                cmd = bspline.get_pos(t);
                break;
            case MODE_TIME_OPT:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = quintic.get_state(t).pos;
                break;
            case MODE_JERK_OPT:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = quintic.get_state(t).pos;
                break;
            case MODE_ENERGY_OPT:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = quintic.get_state(t).pos;
                break;
            case MODE_MULTI_OPT:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = quintic.get_state(t).pos;
                break;
            case MODE_QLEARN:
            case MODE_SUPER:
                for (int i = 0; i < axis_num; ++i)
                    cmd[i] = quintic.get_state(t).pos;
                break;
            case MODE_CART_LINE:
                {
                    // 笛卡尔空间直线插补 -> 逆运动学 -> 关节空间
                    std::vector<double> cart_pt = line.get_pos(t);
                    CartPose cp;
                    cp.x = cart_pt[0]; cp.y = cart_pt[1]; cp.z = cart_pt[2];
                    cp.rx = cart_start.rx; cp.ry = cart_start.ry; cp.rz = cart_start.rz;
                    std::vector<double> jnt;
                    if(inverse_kin(cp, dh_table, jnt))
                        cmd = jnt;
                    else
                        cmd = cur_pos;
                }
                break;
            case MODE_CART_CIRCLE:
                {
                    // 笛卡尔空间圆弧插补 -> 逆运动学 -> 关节空间
                    std::vector<double> cart_pt = cart_circle.get_pos(t);
                    CartPose cp;
                    cp.x = cart_pt[0]; cp.y = cart_pt[1]; cp.z = cart_pt[2];
                    cp.rx = cart_start.rx; cp.ry = cart_start.ry; cp.rz = cart_start.rz;
                    std::vector<double> jnt;
                    if(inverse_kin(cp, dh_table, jnt))
                        cmd = jnt;
                    else
                        cmd = cur_pos;
                }
                break;
            default:
                cmd = cur_pos;
                break;
        }

        // 虚拟3D禁区检测与贴边跳过
        if (zone_check_enabled && !forbidden_zones.empty())
        {
            // 将关节空间命令转为笛卡尔位姿，检测是否落入禁区
            CartPose cart_cmd = forward_kin(cmd, dh_table);
            CartPose cart_safe;
            if (zone_avoid.slide_along_boundary(cart_cmd, forbidden_zones, cart_safe, zone_safety_margin))
            {
                // 落入禁区，贴边修正后逆解回关节空间
                log_print(LOG_WARN, "轨迹落入禁区，贴边跳过: ("
                          + std::to_string(cart_cmd.x) + "," + std::to_string(cart_cmd.y) + "," + std::to_string(cart_cmd.z) + ")"
                          + " -> (" + std::to_string(cart_safe.x) + "," + std::to_string(cart_safe.y) + "," + std::to_string(cart_safe.z) + ")");
                std::vector<double> jnt_safe;
                if (inverse_kin(cart_safe, dh_table, jnt_safe))
                    cmd = jnt_safe;
                // 逆解失败则保留原cmd（受软限位保护）
            }
        }

        // 工业软限位保护
        for (int i = 0; i < axis_num; ++i)
            cmd[i] = clamp_val(cmd[i], JOINT_MIN_LIMIT, JOINT_MAX_LIMIT);

        // 下发伺服脉冲
        for (int i = 0; i < axis_num; ++i)
        {
            cur_pos[i] = cmd[i];
            int32_t pulse = static_cast<int32_t>(cmd[i] * RAD_TO_PULSE);
            ec_dev.set_target_pulse(i, pulse);
        }
        ec_dev.cycle_io();

        // 精准周期休眠
        auto cost = steady_clock::now() - loop_start;
        if (cost < period)
            this_thread::sleep_for(period - cost);
    }
}

// 基础轨迹接口
void RobotCtrl::move_cubic(const std::vector<double>& tar, double t)
{
    cur_mode = MODE_CUBIC;
    for (int i = 0; i < axis_num; ++i)
        cubic.init(cur_pos[i], tar[i], 0, 0, t);
}

void RobotCtrl::move_quintic(const std::vector<double>& tar, double t)
{
    cur_mode = MODE_QUINTIC;
    for (int i = 0; i < axis_num; ++i)
        quintic.init(cur_pos[i], tar[i], 0,0,0,0,t);
}

void RobotCtrl::move_poly345(const std::vector<double>& tar, double t)
{
    cur_mode = MODE_POLY345;
    for (int i = 0; i < axis_num; ++i)
        poly345.init(cur_pos[i], tar[i], t);
}

void RobotCtrl::move_scurve(const std::vector<double>& tar, double v, double a, double j)
{
    cur_mode = MODE_SCURVE;
    // S曲线以第一轴为基准规划
    scurve.init(cur_pos[0], tar[0], v, a, j);
}

void RobotCtrl::move_line(const std::vector<double>& tar, double t)
{
    cur_mode = MODE_LINE;
    line.init(cur_pos, tar, t);
}

void RobotCtrl::move_circle(const std::vector<double>& c, double r, double ang, double t)
{
    cur_mode = MODE_CIRCLE;
    circle.init(cur_pos, c, r, ang, t);
}

void RobotCtrl::move_bspline(const std::vector<std::vector<double>>& ctrl, double t)
{
    cur_mode = MODE_BSPLINE;
    bspline.init(ctrl, t);
}

// 最优轨迹
void RobotCtrl::move_time_opt(const std::vector<double>& tar, const std::vector<double>& v, const std::vector<double>& a)
{
    cur_mode = MODE_TIME_OPT;
    t_opt.set_data(cur_pos, tar, v, a);
    double T = t_opt.solve();
    log_print(LOG_INFO, "时间最优周期: " + std::to_string(T) + "s");
    // 以第一轴轨迹为基准统一运动时间
    quintic = t_opt.get_traj(0, T);
}

void RobotCtrl::move_jerk_opt(const std::vector<double>& tar)
{
    j_opt.set_data(cur_pos, tar);
    double t = j_opt.solve();
    move_quintic(tar, t);
}

void RobotCtrl::move_energy_opt(const std::vector<double>& tar, const std::vector<DynParam>& dyn)
{
    e_opt.set_dyn(dyn);
    e_opt.set_data(cur_pos, tar);
    double t = e_opt.solve();
    move_quintic(tar, t);
}

void RobotCtrl::move_multi_opt(const std::vector<double>& tar, const std::vector<double>& v, const std::vector<double>& a,
                                const std::vector<DynParam>& dyn, double wt, double wj, double we)
{
    m_opt.set_weight(wt, wj, we);
    m_opt.set_all(cur_pos, tar, v, a, dyn);
    double t = m_opt.solve();
    move_quintic(tar, t);
}

// 机器学习
void RobotCtrl::init_ql(const std::vector<double>& acts, double lr, double g, double e, int ep)
{
    ql.init(acts, lr, g, e, ep);
}

void RobotCtrl::move_ql(const std::vector<double>& tar)
{
    ql.train(RLState(cur_pos, tar));
    quintic = ql.get_traj(RLState(cur_pos, tar));
    cur_mode = MODE_QLEARN;
}

void RobotCtrl::init_ml(const std::vector<std::vector<double>>& x, const std::vector<double>& y,
                         double lr, double g, double e, int ep)
{
    (void)lr; (void)g; (void)e; (void)ep;
    sup.load_sample(x, y);
    sup.train();
}

void RobotCtrl::move_ml(const std::vector<double>& tar)
{
    move_sup(tar);
}

void RobotCtrl::load_sup_sample(const std::vector<std::vector<double>>& x, const std::vector<double>& y)
{
    sup.load_sample(x, y);
}

void RobotCtrl::train_sup()
{
    sup.train();
}

void RobotCtrl::move_sup(const std::vector<double>& tar)
{
    double t = sup.predict(cur_pos, tar);
    move_quintic(tar, t);
}

// 禁区管理
void RobotCtrl::add_forbidden_zone(const ForbiddenZone& zone)
{
    forbidden_zones.push_back(zone);
    log_print(LOG_INFO, "添加虚拟3D禁区: type=" + std::to_string(zone.type));
}

void RobotCtrl::set_forbidden_zones(const std::vector<ForbiddenZone>& zones)
{
    forbidden_zones = zones;
    log_print(LOG_INFO, "设置虚拟3D禁区: count=" + std::to_string(zones.size()));
}

// 禁区检测与贴边修正（单次调用版本）
bool RobotCtrl::check_and_slide(CartPose& cp) const
{
    if (!zone_check_enabled || forbidden_zones.empty())
        return false;
    return zone_avoid.slide_along_boundary(cp, forbidden_zones, cp, zone_safety_margin);
}

// 笛卡尔
void RobotCtrl::init_dh(const std::vector<DHParam>& dh)
{
    dh_table = dh;
}

void RobotCtrl::move_cart_line(const CartPose& tar, double t)
{
    cur_mode = MODE_CART_LINE;
    cart_end = tar;
    // 先从当前关节位置反解出当前笛卡尔位姿，再规划直线
    cart_start = forward_kin(cur_pos, dh_table);
    std::vector<double> s = {cart_start.x, cart_start.y, cart_start.z};
    std::vector<double> e = {tar.x, tar.y, tar.z};
    line.init(s, e, t);
}

void RobotCtrl::move_cart_circle(const CartPose& c, double r, double ang, double t)
{
    cur_mode = MODE_CART_CIRCLE;
    cart_start = forward_kin(cur_pos, dh_table);
    std::vector<double> s = {cart_start.x, cart_start.y, cart_start.z};
    std::vector<double> ct = {c.x, c.y, c.z};
    cart_circle.init(s, ct, r, ang, t);
}