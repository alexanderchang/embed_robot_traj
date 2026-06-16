#include "common.h"
#include <ctime>
#include <algorithm>
#include <sstream>

void log_print(LogLevel level, const std::string& msg)
{
    std::time_t now = std::time(nullptr);
    char time_buf[32];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

    const char* prefix;
    switch(level)
    {
        case LOG_INFO:  prefix = "[INFO] "; break;
        case LOG_WARN:  prefix = "[WARN] "; break;
        case LOG_ERROR: prefix = "[ERROR] "; break;
        default:        prefix = "[INFO] "; break;
    }
    // 控制台输出
    std::cout << time_buf << " " << prefix << msg << std::endl;

    // 双输出：同时写入本地日志文件 (嵌入式场景: 掉电不丢失)
    static std::ofstream log_file("/var/log/robot_ctrl.log", std::ios::app);
    if(log_file.is_open())
    {
        log_file << time_buf << " " << prefix << msg << std::endl;
        log_file.flush();  // 实时落盘
    }
}

double clamp_val(double val, double min_v, double max_v)
{
    return std::clamp(val, min_v, max_v);
}

bool read_config(const std::string& file, std::string& eth_name, int& axis_cnt)
{
    std::ifstream f(file);
    if(!f.is_open())
    {
        log_print(LOG_ERROR, "无法打开配置文件: " + file);
        return false;
    }
    std::string line;
    while(std::getline(f, line))
    {
        // 去除前后空格
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        // 跳过空行和注释
        if(line.empty() || line[0] == '#' || line[0] == ';')
            continue;

        auto eq_pos = line.find('=');
        if(eq_pos == std::string::npos)
            continue;

        std::string key = line.substr(0, eq_pos);
        std::string val = line.substr(eq_pos + 1);
        // 去除key和val的空白
        key.erase(key.find_last_not_of(" \t") + 1);
        val.erase(0, val.find_first_not_of(" \t"));

        if(key == "eth")
            eth_name = val;
        else if(key == "axis_cnt")
            axis_cnt = std::stoi(val);
    }
    f.close();

    if(eth_name.empty() || axis_cnt <= 0)
    {
        log_print(LOG_ERROR, "配置文件参数不完整: eth=" + eth_name + " axis_cnt=" + std::to_string(axis_cnt));
        return false;
    }
    log_print(LOG_INFO, "配置读取成功: eth=" + eth_name + " axis_cnt=" + std::to_string(axis_cnt));
    return true;
}