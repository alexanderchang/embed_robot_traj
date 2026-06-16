#ifndef ROBOT_CTRL_H
#define ROBOT_CTRL_H

#include <vector>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "common.h"
#include "ec_dev.h"
#include "kinematics.h"
#include "trajectory.h"
#include "traj_opt.h"
#include "traj_blend.h"
#include "obstacle.h"
#include "ml_traj.h"
#include "multi_robot.h"

// 轨迹模式枚举
enum TrajMode
{
    MODE_CUBIC,
    MODE_QUINTIC,
    MODE_POLY345,
    MODE_SCURVE,
    MODE_LINE,
    MODE_CIRCLE,
    MODE_BSPLINE,
    MODE_TIME_OPT,
    MODE_JERK_OPT,
    MODE_ENERGY_OPT,
    MODE_MULTI_OPT,
    MODE_QLEARN,
    MODE_SUPER,
    MODE_CART_LINE,
    MODE_CART_CIRCLE
};

class RobotCtrl
{
private:
    EcDev ec_dev;
    int axis_num;
    std::vector<double> cur_pos;
    boost::thread ctrl_thread;
    bool thread_run;
    TrajMode cur_mode;

    // 基础轨迹
    CubicPoly cubic;
    QuinticPoly quintic;
    Poly345 poly345;
    SCurve scurve;
    LineInterp line;
    CircleInterp circle;
    BSpline3 bspline;

    // 最优轨迹
    TimeOpt t_opt;
    JerkOpt j_opt;
    EnergyOpt e_opt;
    MultiOpt m_opt;

    // 机器学习
    QLearnTraj ql;
    SuperTraj sup;

    // 运动学
    std::vector<DHParam> dh_table;
    CartPose cart_start, cart_end;
    CircleInterp cart_circle;   // 笛卡尔空间圆弧插补器

    // 虚拟3D禁区（不可运行空间）
    std::vector<ForbiddenZone> forbidden_zones;
    ZoneAvoid zone_avoid;
    bool zone_check_enabled;    // 禁区检测开关
    double zone_safety_margin;  // 贴边安全余量(mm)

    // 禁区检测与贴边修正（控制循环内调用）
    bool check_and_slide(CartPose& cp) const;

    void control_loop();
public:
    RobotCtrl();
    ~RobotCtrl();

    bool init(const std::string& eth, int axis);
    void start();
    void stop();

    // 基础轨迹接口
    void move_cubic(const std::vector<double>& tar, double t);
    void move_quintic(const std::vector<double>& tar, double t);
    void move_poly345(const std::vector<double>& tar, double t);
    void move_scurve(const std::vector<double>& tar, double v, double a, double j);
    void move_line(const std::vector<double>& tar, double t);
    void move_circle(const std::vector<double>& c, double r, double ang, double t);
    void move_bspline(const std::vector<std::vector<double>>& ctrl, double t);

    // 最优轨迹
    void move_time_opt(const std::vector<double>& tar, const std::vector<double>& v, const std::vector<double>& a);
    void move_jerk_opt(const std::vector<double>& tar);
    void move_energy_opt(const std::vector<double>& tar, const std::vector<DynParam>& dyn);
    void move_multi_opt(const std::vector<double>& tar, const std::vector<double>& v, const std::vector<double>& a,
                        const std::vector<DynParam>& dyn, double wt, double wj, double we);

    // 机器学习
    void init_ql(const std::vector<double>& acts, double lr, double g, double e, int ep);
    void move_ql(const std::vector<double>& tar);
    void init_ml(const std::vector<std::vector<double>>& x, const std::vector<double>& y,
                 double lr, double g, double e, int ep);
    void move_ml(const std::vector<double>& tar);
    void load_sup_sample(const std::vector<std::vector<double>>& x, const std::vector<double>& y);
    void train_sup();
    void move_sup(const std::vector<double>& tar);

    // 笛卡尔空间
    void init_dh(const std::vector<DHParam>& dh);
    void move_cart_line(const CartPose& tar, double t);
    void move_cart_circle(const CartPose& c, double r, double ang, double t);

    // 虚拟3D禁区管理
    void add_forbidden_zone(const ForbiddenZone& zone);
    void set_forbidden_zones(const std::vector<ForbiddenZone>& zones);
    void enable_zone_check(bool enable) { zone_check_enabled = enable; }
    void set_zone_safety_margin(double margin) { zone_safety_margin = margin; }
    bool is_zone_check_enabled() const { return zone_check_enabled; }
    const std::vector<ForbiddenZone>& get_forbidden_zones() const { return forbidden_zones; }

    std::vector<double> get_cur_pos() const { return cur_pos; }
};

#endif // ROBOT_CTRL_H