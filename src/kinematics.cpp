#include "kinematics.h"

CartPose forward_kin(const std::vector<double>& joint, const std::vector<DHParam>& dh)
{
    CartPose pose;
    // 使用DH参数表做正运动学递推 (标准DH: T = Rz(θ)*Tz(d)*Tx(a)*Rx(α))
    // 从基座到末端的齐次变换累乘
    double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    for(size_t i = 0; i < dh.size() && i < joint.size(); ++i)
    {
        double th = dh[i].theta + joint[i];  // 关节角叠加DH基准角
        double d  = dh[i].d;
        double a  = dh[i].a;
        double al = dh[i].alpha;

        double cth = cos(th), sth = sin(th);
        double ca  = cos(al), sa  = sin(al);

        double Ti[4][4] = {
            {cth, -sth*ca,  sth*sa, a*cth},
            {sth,  cth*ca, -cth*sa, a*sth},
            {0,    sa,      ca,     d    },
            {0,    0,       0,      1    }
        };

        // T = T * Ti
        double n[4][4];
        for(int r = 0; r < 4; ++r)
            for(int c = 0; c < 4; ++c)
                n[r][c] = T[r][0]*Ti[0][c] + T[r][1]*Ti[1][c]
                        + T[r][2]*Ti[2][c] + T[r][3]*Ti[3][c];
        for(int r = 0; r < 4; ++r)
            for(int c = 0; c < 4; ++c)
                T[r][c] = n[r][c];
    }

    pose.x  = T[0][3];
    pose.y  = T[1][3];
    pose.z  = T[2][3];

    // 姿态提取 (ZYX欧拉角近似)
    pose.ry = asin(clamp_val(-T[2][0], -1.0, 1.0));
    if(fabs(cos(pose.ry)) > EPS)
    {
        pose.rx = atan2(T[2][1], T[2][2]);
        pose.rz = atan2(T[1][0], T[0][0]);
    }
    else
    {
        pose.rx = 0.0;
        pose.rz = atan2(-T[0][1], T[1][1]);
    }
    return pose;
}

bool inverse_kin(const CartPose& cart, const std::vector<DHParam>& dh, std::vector<double>& joint_out)
{
    joint_out.resize(6, 0.0);

    // 基于DH参数表的解析/数值逆运动学
    // 对于6轴工业机器人，优先使用DH参数反解
    double x = cart.x, y = cart.y, z = cart.z;

    // 提取DH参数中前两连杆长度 (a_i, d_i)
    double L1 = (dh.size() > 0) ? dh[0].a : 200.0;
    double L2 = (dh.size() > 1) ? dh[1].a : 500.0;

    double r = sqrt(x*x + y*y);
    double d_base = (dh.size() > 0) ? dh[0].d : 0.0;

    // 奇异点判断
    if(fabs(r) < EPS && fabs(z - d_base) < EPS)
        return false;

    // 几何逆解 (2R平面简化 + 腕部解耦)
    joint_out[0] = atan2(y, x);

    double D = (r*r + (z - d_base)*(z - d_base) - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    D = clamp_val(D, -1.0, 1.0);
    double theta2 = atan2(sqrt(1.0 - D*D), D);   // elbow-up
    double theta1 = atan2(z - d_base, r) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));

    joint_out[1] = theta1;
    joint_out[2] = theta2;
    joint_out[3] = cart.rx;
    joint_out[4] = cart.ry;
    joint_out[5] = cart.rz;
    return true;
}

bool is_singularity(const std::vector<double>& joint, const std::vector<DHParam>& dh)
{
    if(joint.size() < 2) return false;
    // 基于DH参数的第二关节奇异点检测
    if(!dh.empty())
    {
        double L1 = dh[0].a;
        double L2 = (dh.size() > 1) ? dh[1].a : L1;
        double x = L2 * cos(joint[0]) * cos(joint[1]);
        double y = L2 * sin(joint[0]) * cos(joint[1]);
        if(sqrt(x*x + y*y) < 1e-3) return true;   // 腕部奇异
    }
    // 肘部奇异: 第二关节接近0或±180度
    return fabs(fabs(joint[1]) - M_PI/2.0) < 0.01;
}