#include "Lift_Leg.hpp"

void Lift_Leg::Start()
{
    enabled_ = false;

    // 初始化各个电机
    motor_back.Init(Hardware::hcan_main, 5, DJI_C620);
    motor_front_left.Init(Hardware::hcan_main, 6, DJI_C620);
    motor_front_right.Init(Hardware::hcan_main, 7, DJI_C620);

    motor_Test.Init(Hardware::hcan_main, 2, DJI_C620);
    

    // ADRC 简易初始化（统一参数，后续按实际机构再调）
    motor_front_left.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(8.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1500.0f)
        .ADRC_SOTF(0.67f)
        .Apply();

    motor_front_right.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(15.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1500.0f)
        .ADRC_SOTF(0.67f)
        .Apply();

    motor_back.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(8.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1500.0f)
        .ADRC_SOTF(0.67f)
        .Apply();
    
    motor_Test.ConfigADRC()
        .AsSpeedC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(8.0f)
        .SpdLimit(1500.0f)
        .ADRC_MaxPlannedVel(1500.0f)
        .ADRC_SOTF(0.67f)
        .Apply();

    // motor_front_left.driver.Enable();
    // motor_front_right.driver.Enable();
    // motor_back.driver.Enable();

}

void Lift_Leg::Update()
{

    if (!enabled_)
    {
        return;
    }

}
