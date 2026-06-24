#include "Lift_Leg.hpp"
#include "Interboardcomm.hpp"
#include "board.hpp"
#include "Chassis.hpp"

Lift_Leg& APP::leg = Lift_Leg::GetInstance();

void Lift_Leg::Start()
{
    // 小黄注册：0-3 分别是从前到后四个
    Pin pin[4] = 
    {
        {'I', 5},
        {'A', 1},
        {'A', 2},
        {'A', 3}
    };
    
    for(int i = 0; i < 4; i++)
    {
        detector[i].Init(pin[i]);
    }
}

void Lift_Leg::_SendLegPos(int16_t fpos, int16_t bpos)
{
    uint8_t payload[4];

    payload[0] = (fpos >> 8) & 0xFF;
    payload[1] = fpos & 0xFF;
    payload[2] = (bpos >> 8) & 0xFF;
    payload[3] = bpos & 0xFF;

    MOD::board_can.SendTask(MOD::C_sub_ctrl_board_id, 6, payload, sizeof(payload), false);
}

void Lift_Leg::Update()
{

    if (!enabled_)
    {
        return;
    }

    // 进行位置限制
    PosLim();
    
    // 发送腿部位置指令
    _SendLegPos(front_pos, back_pos);
}

void Lift_Leg::Enable()
{
    enabled_ = true;
}

void Lift_Leg::Disable()
{
    enabled_ = false;
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

/********************************************  越野部分 ********************************************/
// 前腿回到零位
void Lift_Leg::F_Origin()
{
    front_pos = 0;
}

// 后腿回到零位
void Lift_Leg::B_Origin()
{
    back_pos = 0;
}

// 前腿放低200
void Lift_Leg::F_Down_200()
{
    front_pos = -200;
}

// 前腿放低400
void Lift_Leg::F_Down_400()
{
    front_pos = -400;
}

// 后腿放低200
void Lift_Leg::B_Down_200()
{
    back_pos = -200;
}

// 后腿放低400
void Lift_Leg::B_Down_400()
{
    back_pos = -400;
}

void Wait_detected(InfraredDetector& xiaohuang_x)
{
    Seq::WaitUntil([&]()
    {
        return xiaohuang_x.Read();
    });
}

// 上200逻辑
void Lift_Leg::Up_200()
{
    // 登200时，先放低前后腿
    F_Down_200();
    B_Down_200();
    
    // 往前缓慢移动
    APP::chas.Move(Vec2(0.2, 0));

    // 等待小黄0号检测到可以抬前腿
    Wait_detected(detector[0]);
    APP::chas.Move(Vec2(0, 0));

    // 抬升前腿
    F_Origin();

    // 往前移动
    APP::chas.Move(Vec2(0.5, 0));

    // 等待小黄2号检测到可以抬后腿
    Wait_detected(detector[2]);

    // 抬升后腿
    B_Origin();
}