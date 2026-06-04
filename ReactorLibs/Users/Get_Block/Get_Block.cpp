#include "Get_Block.hpp"


#define LEFT_ENDING_CODE 1409674 //左手 正 是前伸
#define RIGHT_ENDING_CODE -1411402 // 右手 负 是前伸

GetBlock& APP::get_block = GetBlock::GetInstance();

void GetBlock::Start()
{
    /********************      下摩擦带DM配置      *******************/

    // 初始化
    motor_s2325.Init(Hardware::hcan_sub, 9, 19, VEL, DM_S2325); 
    // 设置DM参数
    motor_s2325.SetFeedbackRange(
        3.14159f,   // PMAX = π（单圈精度 π/32767 ≈ 0.096 mrad）
        10.0f,      // VMAX = 10 rad/s
        5.0f        // TMAX = 5 Nm
    ); 
    // 限幅
    // motor_s2325.SetPosLimit(50, 0);
    motor_s2325.SetVelLimit(38); // 38是额定最大rad/s
    // 使能
    // motor_s2325.SetAutoEnable(true, 5000);
    motor_s2325.Enable();

    /********************      上摩擦带2006配置      *******************/

    // 初始化
    motor_2006_fric.Init(Hardware::hcan_main, 6, DJI_C610);                   
    // 设置上摩擦带2006电机参数，用速度模式
    motor_2006_fric.ConfigADRC()
        .AsSpeedC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(1.5e-4f, 0.18f)
        .ADRC_Limit(8.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(5000.0f)
        .ADRC_SOTF(0.2f)
        .Apply();
    // 使能
    motor_2006_fric.driver.Enable();

    /********************      手2006配置      *******************/
    // 初始化
    motor_2006[0].Init(Hardware::hcan_main, 4, DJI_C610); // 左手电机
    motor_2006[1].Init(Hardware::hcan_main, 5, DJI_C610); // 右手电机
    // 设置手电机的参数
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
        // 使能
        motor_2006[i].driver.Enable();
    }

    /********************      气缸、气泵等开关量配置      *******************/

    // 取块用的四个开关是A板的 POWER_CTRLx 引脚，用的 MOS 和 NPN
    for(int i = 0; i < 4; i++)
    {
        _pin[i].port = 'H'; 
        _pin[i].number = static_cast<uint8_t>(i + 2); // 分别是PH2 PH3 PH4 PH5
        _relay[i].Init(_pin[i], Relay::HIGH_ON);
    }

    // 小黄（检测物块到位）配置
    _pin_xiaohuang = {'E', 4};
    xiaohuang.Init(_pin_xiaohuang);

    // 初始化标志位、变量
    _reached = false;
    _store_flag = false;
    _block_cnt = 0;
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

    // 存块
    _UpdateStore();

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
    // if(fabs(delta_pos[0]) < 500 && fabs(delta_pos[1]) < 500)//这写的不对，一上电都是0，直接满足条件了
    // {
    //     _reached = true;
    // }
    // else _reached = false;
}

void GetBlock::_UpdateStore()
{
    if(xiaohuang.Read()) // 检测到物块到位了
    {
        // 如果是存块过程中触发小黄才算存入，避免误触发
        if(_store_flag)
        {
            _block_cnt++;
            APP::monit.LogInfo("[GetBlock]: block stored, current cnt: %d", _block_cnt);
        }
        // 停止存块
        _store_flag = false;
        motor_2006_fric.SetSpd(0);
        motor_s2325.SetVel(0);
    }
    if(_store_flag)
    {
        motor_2006_fric.SetSpd(15 * MotorDJIConst::redu_M2006 * 60.0f / (2*PI) );
        motor_s2325.SetVel(15);
    }
}

// 存块
void GetBlock::Store()
{
    // 存块前检查存储是否已满
    if(_block_cnt >= MAX_BLOCK_CNT)
    {
        APP::monit.LogWarning("[GetBlock]: storage full (current: %d, max: %d)", _block_cnt, MAX_BLOCK_CNT);
        return;
    }
    _store_flag = true;
}

// 放块
void GetBlock::Put()
{
    
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

