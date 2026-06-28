#pragma once

#include "System.hpp"
#include "infrared_detector.hpp"
#include "relay.hpp"

class Lift_Leg : public Application
{
    SINGLETON(Lift_Leg) : Application("Lift_Leg") {}
    APPLICATION_OVERRIDE

public:
    int front_pos = 0; // 前腿位置，单位为mm
    int back_pos = 0;  // 后腿位置，单位为mm
    int16_t real_front_pos_mm = 0; // 前腿实际位置，单位为mm
    int16_t real_back_pos_mm = 0;   // 后腿实际位置，单位为mm

    int _front_max_lim = 0; // 前腿最大位置，单位为mm
    int _front_min_lim = -420;   // 前腿最小位置，单位
    int _back_max_lim = 0; // 后腿最大位置，单位为mm
    int _back_min_lim = -420;   // 后腿最小位置，单位

    /**
     * @brief C板发回的，关于腿部位置 的 回调
     */
    static void LegActRxCb(uint8_t task_id, const uint8_t* payload, uint8_t payload_len, void* user_ctx);

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
    
    // 后边的妲己
    Relay relay_daji;

    void Enable();
    void Disable();
    void F_AlmostOrigin(); // 前腿回到近零位
    void B_AlmostOrigin(); // 后腿回到近零位
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
    bool is_debug = false; // 用于调试
};

namespace APP
{
    extern Lift_Leg& leg;
}
