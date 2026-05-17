#pragma once

#include "System.hpp"

class Lift_Leg : public Application
{
    SINGLETON(Lift_Leg) : Application("Lift_Leg") {}
    APPLICATION_OVERRIDE

public:
    int front_pos = 0; // 前腿位置，单位为mm
    int back_pos = 0;  // 后腿位置，单位为mm

    void ChangeFrontPos(int16_t delta_pos)
    {
        front_pos += delta_pos;
    }

    void ChangeBackPos(int16_t delta_pos)
    {
        back_pos += delta_pos;
    }

private:
    bool enabled_ = false;
};

namespace APP
{
    extern Lift_Leg& leg;
}
