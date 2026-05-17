#include "Lift_Leg.hpp"
#include "Interboardcomm.hpp"

Lift_Leg& APP::leg = Lift_Leg::GetInstance();


void SendLegPos(int16_t fpos, int16_t bpos)
{
    uint8_t payload[4];

    payload[0] = (fpos >> 8) & 0xFF;
    payload[1] = fpos & 0xFF;
    payload[2] = (bpos >> 8) & 0xFF;
    payload[3] = bpos & 0xFF;

    MOD::board_can.SendTask(0x210, 6, payload, 4, false);
}

void Lift_Leg::Start()
{
}



void Lift_Leg::Update()
{

    // if (!enabled_)
    // {
    //     return;
    // }

    SendLegPos(front_pos, back_pos);
}
