#include "VacuumClaw.hpp"
#include "StateCore.hpp"

VacuumClaw& APP::vac_claw = VacuumClaw::GetInstance();

void VacuumClaw::Start()
{
    motor_stretch.Init(Hardware::hcan_main, 6, DJI_C620);
    motor_snap.Init(Hardware::hcan_main, 7, DJI_C620);

    // ADRC控制
    motor_stretch.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(1.0e-4f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(2000.0f)
        .ADRC_MaxPlannedVel(2000.0f)
        .ADRC_SOTF(0.32f)
        .Apply();
    motor_stretch.driver.Enable();

    // ADRC控制
    motor_snap.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(6.4e-5f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(8000.0f)
        .ADRC_SOTF(0.42f)
        .Apply();
    // motor_snap.driver.Enable();

    /****  翻转 DM (hcan_sub, ID 10) ，零位：吸盘面竖直，总行程 0 到 -1.6 rad ****/ 
    motor_pitch.Init(Hardware::hcan_sub, 11, 21, POSANDVEL, DM_J4340);
    // motor_pitch.SetAutoEnable(1);
    motor_pitch.SetFeedbackRange(2.0f, 20.0f, 20.0f);
    motor_pitch.SetPosLimit(pitch_lim_max, pitch_lim_min);
    motor_pitch.SetVelLimit(5.0f);
    motor_pitch.Enable();

    // Seq::Wait(1.0f);
    // motor_pitch.SetZeroPosi();
    // Seq::Wait(0.03);
}

void VacuumClaw::Update()
{
    if (!enabled) return;

    if(System.out_from_debugmode)
    {
        motor_stretch.Neutral();
        motor_snap.Neutral();
        motor_pitch.Disable();
    }

    // 控制前伸和收回
    motor_stretch.SetPos(stretch_pos); // 伸出为正

    // 第一次 update 的时候达妙设置一次零点
    if(set_damiao_zero)
    {
        set_damiao_zero = false;
        motor_pitch.SetPosVel(0.0f, 1.5);
    }

    static int damiao_cnt = 0;
    damiao_cnt++;
    {
        if(damiao_cnt >= 4)
        {
            damiao_cnt = 0;
            motor_pitch.SetPosVel(-pitch_pos, 1.5);
        }
    }

    if(is_out_debug)
    {
        is_out_debug = false;
        StretchOut();
    }

    if(is_back_debug)
    {
        is_back_debug = false;
        StretchBack();
    }
}

/**
  * @brief 伸出夹爪，保持开启状态
  * @details 水平快速伸出，同时Pitch变为水平状态，用于夹取的snap电机打开
  */
void VacuumClaw::StretchOut()
{
    pitch_pos = 0.0f;
    motor_pitch.SetPosVel(-pitch_pos, 1.5);

    // 等爪子pitch竖直
    Seq::Wait(1.5f);

    // 伸出
    stretch_pos = 250000;
}

void VacuumClaw::StretchBack()
{
    pitch_pos = 1.7; // 翻转物块
    motor_pitch.SetPosVel(-pitch_pos, 1.5);

    // 等物块被翻转过来
    Seq::Wait(1.5f);

    // 收回
    stretch_pos = 1000.0; // 防堵转，给个略大于0的值
}

void VacuumClaw::Enable()
{
    enabled = true;
}

void VacuumClaw::Disable()
{
    enabled = false;
}