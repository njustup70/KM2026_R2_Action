#pragma once

#include "System.hpp"
#include "infrared_detector.hpp"

class Lift_Leg : public Application
{
    SINGLETON(Lift_Leg) : Application("Lift_Leg") {}
    APPLICATION_OVERRIDE

public:
    int front_pos = 0; // 前腿位置，单位为mm
    int back_pos = 0;  // 后腿位置，单位为mm

    int _front_max_lim = 0; // 前腿最大位置，单位为mm
    int _front_min_lim = -400;   // 前腿最小位置，单位
    int _back_max_lim = 0; // 后腿最大位置，单位为mm
    int _back_min_lim = -400;   // 后腿最小位置，单位

    void ChangeFrontPos(int16_t delta_pos)
    {
        front_pos += delta_pos;
    }

    void ChangeBackPos(int16_t delta_pos)
    {
        back_pos += delta_pos;
    }

    void SetFrontPos(int16_t target_pos)
    {
        front_pos = target_pos;
    }

    void SetBackPos(int16_t target_pos)
    {
        back_pos = target_pos;
    }

    // 上台阶用的小黄
    InfraredDetector detector[4];
    bool front_ir;
    bool front_mid_ir;
    bool back_mid_ir;
    bool back_ir;
    

    void Enable();
    void Disable();

    void F_Origin(); // 前腿回到零位
    void B_Origin(); // 后腿回到零位
    void F_Down_200(); // 前腿放低200
    void F_Down_400(); // 前腿放低400
    void B_Down_200(); // 后腿放低200
    void B_Down_400(); // 后腿放低400

    void Up_200(); // 上200台阶

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
