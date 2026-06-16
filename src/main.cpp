#include "robot_ctrl.h"
#include "common.h"
#include <unistd.h>

int main(int argc, char** argv)
{
    // 读取配置文件
    std::string eth_name;
    int axis_cnt = 0;
    if (!read_config("../config/robot_config.ini", eth_name, axis_cnt))
    {
        log_print(LOG_ERROR, "读取配置失败");
        return -1;
    }

    log_print(LOG_INFO, "嵌入式工业机器人启动");
    RobotCtrl robot;
    if (!robot.init(eth_name, axis_cnt))
    {
        log_print(LOG_ERROR, "机器人初始化失败");
        return -1;
    }
    robot.start();
    sleep(1);

    // 测试点位 (rad) — 根据实际轴数动态生成
    std::vector<double> p0(axis_cnt, 0.0);
    std::vector<double> p1(axis_cnt, 0.0);
    std::vector<double> p2(axis_cnt, 0.0);
    if(axis_cnt >= 3)
    {
        p1 = {1.2, 0.9, -0.3};
        p2 = {0.8, -0.6, 0.4};
        for(int i = 3; i < axis_cnt; ++i) { p1[i]=0; p2[i]=0; }
    }
    else
    {
        for(int i = 0; i < axis_cnt; ++i) { p1[i]=0.5; p2[i]=-0.3; }
    }

    // 1. 三次多项式
    log_print(LOG_INFO, "=== 三次多项式轨迹 ===");
    robot.move_cubic(p1, 2.0);
    sleep(3);

    // 2. 五次多项式（论文主推）
    log_print(LOG_INFO, "=== 五次多项式轨迹 ===");
    robot.move_quintic(p0, 2.0);
    sleep(3);

    // 3. 3-4-5多项式
    log_print(LOG_INFO, "=== 3-4-5分段多项式 ===");
    robot.move_poly345(p1, 1.8);
    sleep(3);

    // 4. S曲线
    log_print(LOG_INFO, "=== S型加减速轨迹 ===");
    robot.move_scurve(p0, 1.0, 2.0, 10.0);
    sleep(3);

    // 5. 直线插补
    log_print(LOG_INFO, "=== 直线插补 ===");
    robot.move_line(p2, 1.5);
    sleep(3);

    // 6. 圆弧插补
    log_print(LOG_INFO, "=== 圆弧插补 ===");
    std::vector<double> circle_c(axis_cnt, 0.0);
    circle_c[0] = 0.6; circle_c[1] = 0.3;
    robot.move_circle(circle_c, 0.5, 3.14, 2.0);
    sleep(4);

    // 7. B样条
    log_print(LOG_INFO, "=== B样条曲线 ===");
    {
        std::vector<double> mid(axis_cnt, 0.0); mid[0]=0.5; mid[1]=0.4;
        std::vector<std::vector<double>> ctrl = {p0, mid, p1, p2};
        robot.move_bspline(ctrl, 3.0);
    }
    sleep(4);

    // 8. 时间最优
    log_print(LOG_INFO, "=== 时间最优轨迹 ===");
    std::vector<double> v_max(axis_cnt, 1.5);
    std::vector<double> a_max(axis_cnt, 3.0);
    robot.move_time_opt(p0, v_max, a_max);
    sleep(5);

    // 9. 冲击最优
    log_print(LOG_INFO, "=== 冲击最优轨迹 ===");
    robot.move_jerk_opt(p1);
    sleep(4);

    // 10. 能耗最优
    log_print(LOG_INFO, "=== 能耗最优轨迹 ===");
    std::vector<DynParam> dyn(axis_cnt);
    for (auto& d) { d.J=0.01; d.B=0.001; d.Kt=1.2; d.R=2.5; }
    robot.move_energy_opt(p2, dyn);
    sleep(4);

    // 11. 多目标优化
    log_print(LOG_INFO, "=== 多目标优化轨迹 ===");
    robot.move_multi_opt(p0, v_max, a_max, dyn, 1.0, 0.8, 0.5);
    sleep(5);

    // 12. Q-Learning
    log_print(LOG_INFO, "=== Q-Learning强化学习 ===");
    std::vector<double> acts = {0.5, 1.0, 1.5, 2.0, 2.5};
    robot.init_ql(acts, 0.1, 0.9, 0.2, 50);
    robot.move_ql(p1);
    sleep(4);

    // 13. 监督学习
    log_print(LOG_INFO, "=== 监督学习 ===");
    {
        std::vector<double> x1(axis_cnt, 0.2); x1[1]=0.1;
        std::vector<double> x2(axis_cnt, 0.6); x2[1]=0.3;
        std::vector<double> x3(axis_cnt, 1.0); x3[1]=0.7;
        std::vector<std::vector<double>> samples_x = {x1, x2, x3};
        std::vector<double> samples_y = {0.1, 0.3, 0.5};
        robot.init_ml(samples_x, samples_y, 0.01, 0.9, 0.2, 50);
    }
    robot.move_ml(p2);
    sleep(4);

    // 14. 虚拟3D禁区 — 贴边跳过测试
    log_print(LOG_INFO, "=== 虚拟3D禁区 贴边跳过 ===");
    {
        // 初始化DH参数（以3轴为例，实际应根据机器人型号配置）
        std::vector<DHParam> dh(3);
        dh[0] = {200, M_PI/2, 150, 0};  // a=200, alpha=90°, d=150
        dh[1] = {500, 0,      0,   0};  // a=500, alpha=0°,   d=0
        dh[2] = {100, 0,      0,   0};  // a=100, alpha=0°,   d=0
        robot.init_dh(dh);

        // 设置长方体禁区 (x:300~400, y:-100~100, z:0~300)
        // 模拟工作空间中的虚拟墙/设备区域
        robot.add_forbidden_zone(
            ForbiddenZone::box(300, 400, -100, 100, 0, 300));

        // 设置圆柱禁区 (中心x=200, y=150, z=0~250, 半径=80)
        // 模拟立柱/管道等不可进入区域
        robot.add_forbidden_zone(
            ForbiddenZone::cylinder(200, 150, 0, 250, 80));

        // 设置球形禁区 (中心x=150, y=-120, z=180, 半径=60)
        // 模拟设备球体保护区域
        robot.add_forbidden_zone(
            ForbiddenZone::sphere(150, -120, 180, 60));

        robot.enable_zone_check(true);
        robot.set_zone_safety_margin(5.0);  // 5mm安全余量贴边

        // 规划一条穿过禁区的笛卡尔直线轨迹
        CartPose target;
        target.x = 450; target.y = -50; target.z = 200;
        target.rx = 0; target.ry = 0; target.rz = 0;
        robot.move_cart_line(target, 3.0);

        log_print(LOG_INFO, "禁区贴边跳过运行中...");
    }
    sleep(5);

    robot.stop();
    return 0;
}