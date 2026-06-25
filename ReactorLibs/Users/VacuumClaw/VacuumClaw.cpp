#include "VacuumClaw.hpp"

void VacuumClaw::Start()
{
    motor_stretch.Init(Hardware::hcan_main, 1, DJI_C620);
    motor_snap.Init(Hardware::hcan_main, 2, DJI_C620);

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

    motor_pitch.Init(Hardware::hcan_sub, 8, 0x13, POSANDVEL, DM_J4310);
    motor_pitch.SetAutoEnable(1);
    motor_pitch.SetFeedbackRange(12.5f, 35.0f, 20.0f);
    motor_pitch.SetPosLimit(pitch_lim_max, pitch_lim_min);
    motor_pitch.SetVelLimit(10.0f);
    motor_pitch.Enable();
}

void VacuumClaw::Update()
{
    if (!enabled) return;
}