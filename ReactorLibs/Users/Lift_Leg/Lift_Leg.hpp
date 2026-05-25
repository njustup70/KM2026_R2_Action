#pragma once

#include "System.hpp"

class Lift_Leg : public Application
{
    SINGLETON(Lift_Leg) : Application("Lift_Leg") {}
    APPLICATION_OVERRIDE

public:
    int front_pos_bias = 0;
    int back_pos_bias = 0;
    int front_pos = 0; // 前腿位置，单位为mm
    int back_pos = 0;  // 后腿位置，单位为mm

    int _front_max_lim = 200; // 前腿最大位置，单位为mm
    int _front_min_lim = -200;   // 前腿最小位置，单位
    int _back_max_lim = 0; // 后腿最大位置，单位为mm
    int _back_min_lim = -200;   // 后腿最小位置，单位

    void ChangeFrontPos(int16_t delta_pos)
    {
        front_pos += delta_pos;
    }

    void ChangeBackPos(int16_t delta_pos)
    {
        back_pos += delta_pos;
    }

private:
    /// @brief 位置限幅用的
    inline void PosLim();
    void _SendLegPos(int16_t fpos, int16_t bpos);

    bool enabled_ = false;
};

namespace APP
{
    extern Lift_Leg& leg;
}
