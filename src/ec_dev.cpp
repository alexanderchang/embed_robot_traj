#include "ec_dev.h"
#include <unistd.h>
#include <cstring>

EcDev::EcDev() : init_flag(false), slave_cnt(0) {}
EcDev::~EcDev() { close(); }

bool EcDev::init(const std::string& eth, int axis_num)
{
    eth_name = eth;
    slave_cnt = axis_num;
    if(ec_init(eth.c_str()) <= 0)
    {
        log_print(LOG_ERROR, "EtherCAT网卡初始化失败: " + eth);
        return false;
    }
    log_print(LOG_INFO, "EtherCAT网卡 " + eth + " 初始化成功");

    // 扫描从站并配置
    if(ec_config_init(FALSE) <= 0)
    {
        log_print(LOG_ERROR, "从站扫描失败，未发现EtherCAT设备");
        ec_close();
        return false;
    }
    log_print(LOG_INFO, "发现从站数量: " + std::to_string(ec_slavecount));

    // 验证从站数量
    if(ec_slavecount < slave_cnt)
    {
        log_print(LOG_ERROR, "实际从站数量(" + std::to_string(ec_slavecount) + 
                  ")少于配置数量(" + std::to_string(slave_cnt) + ")");
        ec_close();
        return false;
    }

    // 缓冲区分配 (使用SOEM标准IOmap)
    int total_rx = slave_cnt * AXIS_RX_LEN;
    int total_tx = slave_cnt * AXIS_TX_LEN;
    rx_buf.resize(total_rx, 0);
    tx_buf.resize(total_tx, 0);

    // 配置PDO映射 (将缓冲区指针赋值给每个从站的inputs/outputs)
    for(int i = 1; i <= slave_cnt; i++)
    {
        ec_slave[i].outputs = &tx_buf[(i - 1) * AXIS_RX_LEN];
        ec_slave[i].inputs  = &rx_buf[(i - 1) * AXIS_TX_LEN];
    }

    // 状态机切换：Init -> PreOp -> SafeOp -> Op
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    if(ec_slave[0].state != EC_STATE_PRE_OP)
    {
        log_print(LOG_ERROR, "无法进入PreOp状态");
        ec_close();
        return false;
    }

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    if(ec_slave[0].state != EC_STATE_SAFE_OP)
    {
        log_print(LOG_ERROR, "无法进入SafeOp状态");
        ec_close();
        return false;
    }

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if(ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        log_print(LOG_ERROR, "伺服无法进入Operational运行状态");
        ec_close();
        return false;
    }

    init_flag = true;
    log_print(LOG_INFO, "汇川EtherCAT初始化完成，轴数: " + std::to_string(slave_cnt));
    return true;
}

bool EcDev::dc_start()
{
    if(!init_flag) return false;
    // 配置DC分布式时钟，周期单位ns
    ec_configdc();
    ec_dcsync0(1, TRUE, EC_PERIOD_US * 1000, 0);
    log_print(LOG_INFO, "DC分布式时钟启动(" + std::to_string(EC_PERIOD_US) + "us)");
    return true;
}

void EcDev::cycle_io()
{
    if(!init_flag) return;
    ec_send_processdata();
    ec_receive_processdata(EC_PERIOD_US);
}

void EcDev::set_ctrl_word(int axis, uint16_t cw)
{
    if(axis < 0 || axis >= slave_cnt)
    {
        log_print(LOG_ERROR, "set_ctrl_word: 轴号越界 " + std::to_string(axis) + "/" + std::to_string(slave_cnt));
        return;
    }
    int off = axis * AXIS_RX_LEN;
    memcpy(&tx_buf[off], &cw, 2);
}

uint16_t EcDev::get_status_word(int axis)
{
    if(axis < 0 || axis >= slave_cnt)
    {
        log_print(LOG_ERROR, "get_status_word: 轴号越界 " + std::to_string(axis) + "/" + std::to_string(slave_cnt));
        return 0;
    }
    int off = axis * AXIS_TX_LEN;
    uint16_t sw = 0;
    memcpy(&sw, &rx_buf[off], 2);
    return sw;
}

void EcDev::set_target_pulse(int axis, int32_t pulse)
{
    if(axis < 0 || axis >= slave_cnt)
    {
        log_print(LOG_ERROR, "set_target_pulse: 轴号越界 " + std::to_string(axis) + "/" + std::to_string(slave_cnt));
        return;
    }
    int off = axis * AXIS_RX_LEN + 2;
    memcpy(&tx_buf[off], &pulse, 4);
}

int32_t EcDev::get_act_pulse(int axis)
{
    if(axis < 0 || axis >= slave_cnt)
    {
        log_print(LOG_ERROR, "get_act_pulse: 轴号越界 " + std::to_string(axis) + "/" + std::to_string(slave_cnt));
        return 0;
    }
    int off = axis * AXIS_TX_LEN + 2;
    int32_t pulse = 0;
    memcpy(&pulse, &rx_buf[off], 4);
    return pulse;
}

bool EcDev::servo_enable(int axis)
{
    set_ctrl_word(axis, CW_RESET); cycle_io(); usleep(20000);
    set_ctrl_word(axis, CW_READY); cycle_io(); usleep(20000);
    set_ctrl_word(axis, CW_ENABLE); cycle_io(); usleep(20000);
    set_ctrl_word(axis, CW_RUN); cycle_io(); usleep(20000);

    uint16_t sw = get_status_word(axis);
    if(sw & 0x0040)
    {
        log_print(LOG_INFO, "轴" + std::to_string(axis) + " 伺服使能成功");
        return true;
    }
    log_print(LOG_ERROR, "轴" + std::to_string(axis) + " 伺服使能失败");
    return false;
}

void EcDev::servo_disable(int axis)
{
    set_ctrl_word(axis, CW_ENABLE); cycle_io(); usleep(10000);
    set_ctrl_word(axis, CW_READY); cycle_io(); usleep(10000);
    set_ctrl_word(axis, CW_RESET); cycle_io();
}

void EcDev::servo_fault_reset(int axis)
{
    set_ctrl_word(axis, CW_FAULT_RST); cycle_io(); usleep(10000);
    set_ctrl_word(axis, CW_RESET); cycle_io();
    log_print(LOG_WARN, "轴" + std::to_string(axis) + " 故障已复位");
}

bool EcDev::check_fault(int axis)
{
    uint16_t sw = get_status_word(axis);
    return (sw & 0x0008) != 0;
}

void EcDev::close()
{
    if(!init_flag) return;
    for(int i = 0; i < slave_cnt; i++) servo_disable(i);
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
    init_flag = false;
    log_print(LOG_INFO, "EtherCAT主站已关闭");
}