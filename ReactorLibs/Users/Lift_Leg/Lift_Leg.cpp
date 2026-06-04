#include "Lift_Leg.hpp"
#include "Interboardcomm.hpp"
#include "board.hpp"

Lift_Leg& APP::leg = Lift_Leg::GetInstance();



void Lift_Leg::_SendLegPos(int16_t fpos, int16_t bpos)
{
    fpos += front_pos_bias;
    uint8_t payload[4];

    payload[0] = (fpos >> 8) & 0xFF;
    payload[1] = fpos & 0xFF;
    payload[2] = (bpos >> 8) & 0xFF;
    payload[3] = bpos & 0xFF;

    MOD::board_can.SendTask(MOD::C_sub_ctrl_board_id, 6, payload, sizeof(payload), false);
}

void Lift_Leg::Start()
{
    front_pos_bias = -15;
}

void Lift_Leg::Update()
{

    // if (!enabled_)
    // {
    //     return;
    // }

    // 进行位置限制
    PosLim();
    
    // 发送腿部位置指令
    _SendLegPos(front_pos, back_pos);
}

inline void Lift_Leg::PosLim()
{
    if (front_pos > _front_max_lim)
    {
        front_pos = _front_max_lim;
    }
    else if (front_pos < _front_min_lim)
    {
        front_pos = _front_min_lim;
    }

    if (back_pos > _back_max_lim)
    {
        back_pos = _back_max_lim;
    }
    else if (back_pos < _back_min_lim)
    {
        back_pos = _back_min_lim;
    }
}