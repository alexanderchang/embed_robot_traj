#ifndef EC_DEV_H
#define EC_DEV_H

#include <soem/ethercat.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include "common.h"

// CiA402 标准控制字（汇川伺服通用）
#define CW_RESET     0x0000
#define CW_READY     0x0006
#define CW_ENABLE    0x0007
#define CW_RUN       0x000F
#define CW_FAULT_RST 0x0080

class EcDev
{
private:
    bool init_flag;
    std::string eth_name;
    int slave_cnt;
    std::vector<uint8_t> rx_buf, tx_buf;
public:
    EcDev();
    ~EcDev();

    bool init(const std::string& eth, int axis_num);
    bool dc_start();
    void cycle_io();

    void set_ctrl_word(int axis, uint16_t cw);
    uint16_t get_status_word(int axis);
    void set_target_pulse(int axis, int32_t pulse);
    int32_t get_act_pulse(int axis);

    bool servo_enable(int axis);
    void servo_disable(int axis);
    void servo_fault_reset(int axis);
    bool check_fault(int axis);
    void close();
    bool is_ready() const { return init_flag; }
    int  get_slave_cnt() const { return slave_cnt; }
};

#endif // EC_DEV_H