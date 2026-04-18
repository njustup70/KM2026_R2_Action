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

    float _debug_pos = 0;
    
public:
    MotorDJI motor_2006[2];
    Relay relay_pump;    // 负压泵
    Relay relay_put;    // 连大气，快速放块
    Relay relay_cylinder_v;   // 气缸，垂直
    Relay relay_cylinder_j;   // 气缸，夹取
};