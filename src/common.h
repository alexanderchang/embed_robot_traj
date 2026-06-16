#ifndef COMMON_H
#define COMMON_H

#include <cstdint>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

// ===================== 嵌入式全局常量 =====================
// EtherCAT 配置
constexpr int EC_PERIOD_US = 1000;    // 1ms 硬实时周期
constexpr int AXIS_RX_LEN  = 8;       // 单轴Rx:2B控制字+4B目标脉冲
constexpr int AXIS_TX_LEN  = 8;       // 单轴Tx:2B状态字+4B实际脉冲

// 单位换算（汇川伺服 1圈=10000脉冲 = 2π rad）
constexpr double RAD_TO_PULSE = 10000.0 / (2 * M_PI);
constexpr double PULSE_TO_RAD = (2 * M_PI) / 10000.0;

// 关节安全限位（rad，工业软限位）
constexpr double JOINT_MIN_LIMIT = -M_PI / 2.0;
constexpr double JOINT_MAX_LIMIT =  M_PI / 2.0;

// 浮点精度
constexpr double EPS = 1e-6;

// 日志等级
enum LogLevel { LOG_INFO, LOG_WARN, LOG_ERROR };

// ===================== 工具函数 =====================
// 日志打印（嵌入式串口/本地文件双输出）
void log_print(LogLevel level, const std::string& msg);

// 数值限幅（限位保护）
double clamp_val(double val, double min_v, double max_v);

// 读取ini配置文件（嵌入式参数配置）
bool read_config(const std::string& file, std::string& eth_name, int& axis_cnt);

#endif // COMMON_H