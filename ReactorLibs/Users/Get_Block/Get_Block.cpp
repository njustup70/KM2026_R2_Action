#include "Get_Block.hpp"

#define LEFT_ENDING_CODE 1409674 //左手 正 是前伸
#define RIGHT_ENDING_CODE -1411402 // 右手 负 是前伸

void GetBlock::Start()
{
    motor_2006[0].Init(Hardware::hcan_main, 4, DJI_C610); // 左手电机
    motor_2006[1].Init(Hardware::hcan_main, 5, DJI_C610); // 右手电机

    for (int i = 0; i < 2; i++)
    {
        motor_2006[i].ConfigADRC()
            .AsPosC()
            .ADRC_Womega(42.0f, 9.6f)
            .ADRC_Physic(3.0e-5f, 0.18f)
            .ADRC_Limit(8.0f)
            .SpdLimit(8000.0f)
            .ADRC_MaxPlannedVel(5000.0f)
            .ADRC_SOTF(0.2f)
            .Apply();
        motor_2006[i].driver.Enable();
    }

    Pin relay_pin = {'F', 1};    // 继电器连接的 GPIO 引脚
    relay.Init(relay_pin, Relay::HIGH_ON);
}

void GetBlock::Update()
{
    if(System.out_from_debugmode)
    {
        for (int i = 0; i < 2; i++)
        {
            motor_2006[i].Neutral();
            motor_2006[i].driver.Disable();
        }
    }
    _UpdatePos();
}

void GetBlock::_UpdatePos()
{
    motor_2006[0].SetPos(_debug_pos);
    motor_2006[1].SetPos(-_debug_pos);
}