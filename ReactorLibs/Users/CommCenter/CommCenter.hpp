#pragma once
#include "System.hpp"

// 后续有关通信的内容可放这里，方便管理用的

class CommCenter : public Application
{
    SINGLETON(CommCenter) : Application("CommCenter") {};

    APPLICATION_OVERRIDE

private:
    bool _send_kfs; // 是否发送KFS给上位机
    bool _is_send_sick_correction; // 是否发送SICK纠正指令

    bool _dock_ok = false;  // 是否完成对接

    float _sick_ch0_data; // sick测距值（ch0）


public:
    Vec3 slam_pos;
    uint8_t kfs_values[12];
    bool IsDocked() const { return _dock_ok; }
    void SlamCorrection(); // 触发Sick纠正
    static void SlamRxCallback(uint8_t func_code, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
    static void OpticalRxCallback(uint8_t func_code, const uint8_t* payload, uint8_t payload_len, void* user_ctx);
};

namespace APP
{
    extern CommCenter& comm_center;
}