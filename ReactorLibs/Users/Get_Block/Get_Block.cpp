#include "Get_Block.hpp"

#define LEFT_ENDING_CODE 1409674 //左手 正 是前伸
#define RIGHT_ENDING_CODE -1411402 // 右手 负 是前伸

GetBlock &hand = GetBlock::GetInstance();

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

    // 取块用的四个开关是A板的 POWER_CTRLx 引脚，用的 MOS 和 NPN
    for(int i = 0; i < 4; i++)
    {
        _pin[i].port = 'H'; 
        _pin[i].number = static_cast<uint8_t>(i + 2); // 分别是PH2 PH3 PH4 PH5
        _relay[i].Init(_pin[i], Relay::HIGH_ON);
    }    
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

    // 两只手位置
    _UpdatePos();

    // 管理四个继电器
    _SetRelayState();
}

void GetBlock::_UpdatePos()
{
    float delta_pos[2] = {0};

    motor_2006[0].SetPos(targ_pos);
    motor_2006[1].SetPos(-targ_pos);

    delta_pos[0] = motor_2006[0].targ_pos - motor_2006[0].driver.measure.total_angle;
    delta_pos[1] = motor_2006[1].targ_pos - motor_2006[1].driver.measure.total_angle;
    if(fabs(delta_pos[0]) < 500 && fabs(delta_pos[1]) < 500)
    {
        _reached = true;
    }
    else _reached = false;
}

void GetBlock::_SetRelayState()
{
    for(int i = 0; i < 4; i++)
    {
        if(_state[i])
        {
            _relay[i].On();
        }
        else
        {
            _relay[i].Off();
        }
    }
}

void GetBlock::Pump_Open()
{
    _state[0] = true;
}

void GetBlock::Pump_Close()
{
    _state[0] = false;
}

void GetBlock::PumpValve_Open()
{
    _state[1] = true;
}

void GetBlock::PumpValve_Close()
{
    _state[1] = false;
}

void GetBlock::Cylinder_v_Open()
{
    _state[2] = true;
}

void GetBlock::Cylinder_v_Close()
{
    _state[2] = false;
}

void GetBlock::Cylinder_j_Open()
{
    _state[3] = true;    
}

void GetBlock::Cylinder_j_Close()
{
    _state[3] = false;    
}

