#include "VacuumClaw.hpp"

VacuumClaw& APP::vac_claw = VacuumClaw::GetInstance();

void VacuumClaw::Start()
{
    motor_stretch.Init(Hardware::hcan_main, 6, DJI_C620);
    motor_snap.Init(Hardware::hcan_main, 7, DJI_C620);

    // ADRC控制
    motor_stretch.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(5.4e-4f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(8000.0f)
        .ADRC_SOTF(0.42f)
        .Apply();
    // motor_stretch.driver.Enable();

    // ADRC控制
    motor_snap.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(5.4e-4f, 0.30f, 0.005f)
        .ADRC_Limit(10.0f)
        .SpdLimit(8000.0f)
        .ADRC_MaxPlannedVel(8000.0f)
        .ADRC_SOTF(0.42f)
        .Apply();
    // motor_snap.driver.Enable();

    /****  翻转 DM (hcan_sub, ID 10) ，零位：吸盘面竖直，总行程 0 到 -1.6 rad ****/
    motor_pitch.Init(Hardware::hcan_sub, 11, 21, POSANDVEL, DM_J4340);
    motor_pitch.SetAutoEnable(1);
    motor_pitch.SetFeedbackRange(2.0f, 20.0f, 20.0f);
    motor_pitch.SetPosLimit(pitch_lim_max, pitch_lim_min);
    motor_pitch.SetVelLimit(5.0f);
    motor_pitch.Enable();
}

void VacuumClaw::Update()
{
    if (!enabled) return;

    motor_stretch.SetPos(stretch_pos);

}

/**
  * @brief 伸出夹爪，保持开启状态
  * @details 水平快速伸出，同时Pitch变为水平状态，用于夹取的snap电机打开
  */
void VacuumClaw::StretchOut()
{
    stretch_pos = 110;
    pitch_pos = 1.6;

    motor_pitch.SetPosVel(-pitch_pos, 1.5);

}