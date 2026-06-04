#pragma once
#include "System.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "relay.hpp"
#include "infrared_detector.hpp"

class GetBlock : public Application
{
    SINGLETON(GetBlock) : Application("GetBlock") {};

    APPLICATION_OVERRIDE

private:
    void _UpdatePos();
    void _UpdateStore();
    void _SetRelayState();

    int _block_cnt;
    static constexpr int MAX_BLOCK_CNT = 2;
    bool _reached;
    bool _store_flag; // 是否开始存块

    float _debug_pos = 0;

    // 0-3分别对应：
    // 负压泵
    // 负压泵连的电磁阀，快速放块
    // 气缸，垂直
    // 气缸，夹取
    Pin _pin[4];
    Relay _relay[4]; 

    // 小黄，检测物块是否到位
    Pin _pin_xiaohuang;
    InfraredDetector xiaohuang; 

    bool _state[4] = {false, false, false, false};
    
public:
    MotorDJI motor_2006[2]; // 手
    MotorDJI motor_2006_fric; // 上面摩擦带
    MotorDM motor_s2325; // 下面摩擦带
    float targ_pos;

    // 手是否到位，public 但 只读
    bool &reached = _reached;

    void Store(); // 开摩擦带存块
    void Put();

    // 管理四个继电器负载的方法
    void Pump_Open();
    void Pump_Close(); // 气泵

    void PumpValve_Open();
    void PumpValve_Close(); // 气泵的电磁阀

    void Cylinder_v_Open();
    void Cylinder_v_Close(); // 向下取块的气缸

    void Cylinder_j_Open();
    void Cylinder_j_Close(); // 夹物块的气缸

};

namespace APP
{
    extern GetBlock& get_block;
}
