#pragma once
#include "System.hpp"
#include "motor_dji.hpp"
#include "relay.hpp"

class GetBlock : public Application
{
    SINGLETON(GetBlock) : Application("GetBlock") {};

    APPLICATION_OVERRIDE

private:
    void _UpdatePos();
    void _SetRelayState();

    bool _reached;

    float _debug_pos = 0;

    Pin _pin[4];
    // 0-3分别对应：
    // 负压泵
    // 负压泵连的电磁阀，快速放块
    // 气缸，垂直
    // 气缸，夹取
    Relay _relay[4]; 

    bool _state[4] = {false, false, false, false};
    
public:
    MotorDJI motor_2006[2];

    float targ_pos;

    // 手是否到位，public 但 只读
    bool &reached = _reached;

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

extern GetBlock &hand;